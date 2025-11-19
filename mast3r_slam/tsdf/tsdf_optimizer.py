import math
from typing import Iterable, List, Sequence

import numpy as np
import torch
import lietorch


class TSDFPoseOptimizer:
    """Pose refinement using TSDF residuals."""

    def __init__(self, volume, keyframes, cfg, use_calib: bool, device: str) -> None:
        self.volume = volume
        self.keyframes = keyframes
        self.cfg = cfg
        self.use_calib = use_calib
        if isinstance(device, torch.device):
            self.device = device
        else:
            self.device = torch.device(device)
        self.samples_per_kf = int(cfg.get("samples_per_kf", 2000))
        self.min_conf = float(cfg.get("min_confidence", 0.05))
        self.max_iterations = int(cfg.get("max_iterations", 3))
        self.lambda_tsdf = float(cfg.get("lambda", 0.1))
        self.damping = float(cfg.get("damping", 1.0e-4))
        self.pre_icp_iters = int(cfg.get("pre_icp_iters", 0))

    # ------------------------------------------------------------------
    def pre_refine(self, kf_idx: int) -> None:
        if self.pre_icp_iters <= 0:
            return
        self._optimize_single(
            kf_idx,
            iterations=self.pre_icp_iters,
            sample_override=min(self.samples_per_kf // 2, 1000),
            log_prefix="[TSDF-ICP]",
        )

    def optimize_keyframes(self, indices: Sequence[int], context: str = "factor") -> None:
        if not indices:
            return
        for idx in indices:
            self._optimize_single(idx, iterations=self.max_iterations, log_prefix=f"[TSDF-{context}]")

    # ------------------------------------------------------------------
    def _optimize_single(
        self,
        idx: int,
        iterations: int,
        sample_override: int = 0,
        log_prefix: str = "[TSDF]",
    ) -> None:
        with self.keyframes.lock:
            if idx >= len(self.keyframes):
                return
            frame = self.keyframes[idx]
            X = frame.X_canon.detach().clone()
            C = frame.C.detach().clone()
            T = lietorch.Sim3(frame.T_WC.data.clone())

        points = X.view(-1, 3).cpu()
        conf = C.view(-1).cpu()
        valid_mask = conf > self.min_conf
        valid_idx = torch.nonzero(valid_mask).view(-1)
        if valid_idx.numel() == 0:
            return

        max_samples = sample_override if sample_override > 0 else self.samples_per_kf
        sample_count = min(max_samples, valid_idx.numel())
        choice = valid_idx[torch.randperm(valid_idx.numel())[:sample_count]]
        pts_cam = points[choice].to(self.device)
        conf_sel = conf[choice].numpy()

        pose = T.to(self.device)
        for it in range(iterations):
            p_world = pose.act(pts_cam).cpu().numpy()
            residuals, jacobians, weights = self._build_linear_system(p_world, conf_sel)
            if len(residuals) < 6:
                break
            H, b = self._accumulate_system(residuals, jacobians, weights)
            try:
                delta = np.linalg.solve(H + self.damping * np.eye(7), -b)
            except np.linalg.LinAlgError:
                break
            delta_torch = torch.from_numpy(delta).to(self.device, dtype=pts_cam.dtype).unsqueeze(0)
            pose = lietorch.Sim3.exp(delta_torch) * pose

        # Write back if updated
        updated = pose.to(self.keyframes.device if hasattr(self.keyframes, "device") else self.device)
        with self.keyframes.lock:
            if idx < len(self.keyframes):
                self.keyframes.T_WC[idx] = updated.data

    def _build_linear_system(self, points_world: np.ndarray, confidences: np.ndarray):
        residuals: List[float] = []
        jacobians: List[np.ndarray] = []
        weights: List[float] = []
        for p, w in zip(points_world, confidences):
            tsdf_val, grad = self.volume.query(p)
            if tsdf_val is None or grad is None:
                continue
            residuals.append(tsdf_val)
            jacobians.append(self._sim3_jacobian(p, grad))
            weights.append(self.lambda_tsdf * w)
        return residuals, jacobians, weights

    def _accumulate_system(self, residuals, jacobians, weights):
        H = np.zeros((7, 7), dtype=np.float64)
        b = np.zeros(7, dtype=np.float64)
        for r, J, w in zip(residuals, jacobians, weights):
            if not np.isfinite(r) or not np.all(np.isfinite(J)):
                continue
            Jw = math.sqrt(max(w, 1.0e-6)) * J
            H += np.outer(Jw, Jw)
            b += Jw * r * math.sqrt(max(w, 1.0e-6))
        return H, b

    @staticmethod
    def _sim3_jacobian(point_world: np.ndarray, grad: np.ndarray) -> np.ndarray:
        J = np.zeros(7, dtype=np.float64)
        J[:3] = grad
        J[3:6] = -np.cross(point_world, grad)
        J[6] = float(np.dot(point_world, grad))
        return J
