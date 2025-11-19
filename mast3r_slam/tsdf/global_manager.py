import queue
import threading
import time
from typing import Iterable, List, Optional

import numpy as np
import torch
import lietorch

from mast3r_slam.config import config

from .global_volume import TSDFVolume
from .tsdf_optimizer import TSDFPoseOptimizer


class TSDFGlobalIntegrator(threading.Thread):
    """Background thread that fuses keyframes into the global TSDF."""

    def __init__(
        self,
        volume: TSDFVolume,
        keyframes,
        cfg,
        optimizer: TSDFPoseOptimizer,
    ) -> None:
        super().__init__(daemon=True)
        self.volume = volume
        self.keyframes = keyframes
        self.cfg = cfg
        self.optimizer = optimizer
        self.next_idx = 0
        self.sleep_interval = float(cfg.get("queue_check_interval", 0.1))
        self.stop_flag = threading.Event()
        self.reintegration_queue: "queue.Queue[int]" = queue.Queue(
            maxsize=int(cfg.get("reintegration_queue", 256))
        )
        self.pending = set()
        self.max_points = int(cfg.get("max_points_per_kf", 40000))
        self.min_conf = float(cfg.get("min_confidence", 0.05))
        self.log_interval = float(cfg.get("log_interval", 30.0))
        self._last_log = time.time()

    def request_stop(self) -> None:
        self.stop_flag.set()

    def mark_pose_update(self, indices: Iterable[int]) -> None:
        for idx in indices:
            if idx in self.pending:
                continue
            try:
                self.reintegration_queue.put_nowait(idx)
                self.pending.add(idx)
            except queue.Full:
                break

    def run(self) -> None:
        while not self.stop_flag.is_set():
            self._integrate_new_keyframes()
            self._process_dirty_queue()
            self._maybe_log_stats()
            time.sleep(self.sleep_interval)

    # ------------------------------------------------------------------
    def _integrate_new_keyframes(self) -> None:
        total = len(self.keyframes)
        while self.next_idx < total and not self.stop_flag.is_set():
            self._integrate_single(self.next_idx)
            self.optimizer.pre_refine(self.next_idx)
            self.next_idx += 1

    def _process_dirty_queue(self) -> None:
        while not self.reintegration_queue.empty() and not self.stop_flag.is_set():
            try:
                idx = self.reintegration_queue.get_nowait()
            except queue.Empty:
                break
            self.pending.discard(idx)
            if idx < len(self.keyframes):
                self._integrate_single(idx)

    def _integrate_single(self, idx: int) -> None:
        with self.keyframes.lock:
            if idx >= len(self.keyframes):
                return
            frame = self.keyframes[idx]
            X = frame.X_canon.detach().clone()
            C = frame.C.detach().clone()
            T = lietorch.Sim3(frame.T_WC.data.clone())

        points = X.view(-1, 3)
        conf = C.view(-1)
        valid = conf > self.min_conf
        valid_idx = torch.nonzero(valid).view(-1)
        if valid_idx.numel() == 0:
            return
        sample_count = min(valid_idx.numel(), self.max_points)
        choice = valid_idx[torch.randperm(valid_idx.numel())[:sample_count]]
        pts_cam = points[choice]
        conf_sel = conf[choice].cpu().numpy().astype(np.float64)

        device = pts_cam.device
        pose = T.to(device)
        pts_world = pose.act(pts_cam).cpu().numpy()
        zero = torch.zeros(1, 3, device=device, dtype=pts_cam.dtype)
        cam_origin = pose.act(zero).squeeze(0).cpu().numpy()
        self.volume.integrate(pts_world, conf_sel, cam_origin)

    def _maybe_log_stats(self) -> None:
        if time.time() - self._last_log < self.log_interval:
            return
        self._last_log = time.time()
        stats = self.volume.stats()
        print(
            f"[TSDF-GLOBAL] voxels={stats['valid_voxels']}/{stats['total_voxels']}\tnext_idx={self.next_idx}"
        )


class TSDFGlobalOptThread(threading.Thread):
    def __init__(self, optimizer: TSDFPoseOptimizer, keyframes, cfg) -> None:
        super().__init__(daemon=True)
        self.optimizer = optimizer
        self.keyframes = keyframes
        self.cfg = cfg
        self.stop_flag = threading.Event()
        self.sleep_interval = float(cfg.get("queue_check_interval", 0.1))
        max_q = int(cfg.get("opt_queue", cfg.get("reintegration_queue", 256)))
        self.queue: "queue.Queue[int]" = queue.Queue(maxsize=max_q)
        self.pending = set()
        self.max_opt_batch = int(cfg.get("max_opt_batch", 1))
        self.cooldown_ms = float(cfg.get("opt_cooldown_ms", 50.0))
        self._stream = None
        try:
            # Use lower priority stream if CUDA available and optimizer on CUDA
            if torch.cuda.is_available() and str(getattr(self.optimizer, "device", "")).startswith("cuda"):
                # On most devices, smaller values indicate higher priority; we choose a low priority (positive)
                self._stream = torch.cuda.Stream(priority=1)
        except Exception:
            self._stream = None

    def request_stop(self) -> None:
        self.stop_flag.set()

    def enqueue(self, indices: Iterable[int]) -> None:
        for idx in indices:
            if idx in self.pending:
                continue
            try:
                self.queue.put_nowait(idx)
                self.pending.add(idx)
            except queue.Full:
                break

    def run(self) -> None:
        while not self.stop_flag.is_set():
            batch = []
            while not self.queue.empty() and not self.stop_flag.is_set() and len(batch) < self.max_opt_batch:
                try:
                    idx = self.queue.get_nowait()
                except queue.Empty:
                    break
                if idx not in batch:
                    batch.append(idx)
            if batch:
                try:
                    if self._stream is not None:
                        # Schedule GPU ops on a low-priority stream to reduce contention
                        with torch.cuda.stream(self._stream):
                            self.optimizer.optimize_keyframes(batch, context="async")
                    else:
                        self.optimizer.optimize_keyframes(batch, context="async")
                finally:
                    for i in batch:
                        self.pending.discard(i)
            # cooldown to avoid contention with other services (e.g., quality/refine)
            time.sleep(self.sleep_interval + self.cooldown_ms / 1000.0)


class TSDFGlobalManager:
    """High-level orchestrator for global TSDF fusion and optimization."""

    def __init__(self, keyframes, cfg, use_calib: bool, device: str) -> None:
        self.enabled = bool(cfg.get("enabled", False))
        self.keyframes = keyframes
        self.cfg = cfg
        self.volume = TSDFVolume(
            voxel_size=cfg.get("voxel_size", 0.03),
            truncation=cfg.get("trunc_dist", 0.12),
            max_weight=cfg.get("max_weight", 100.0),
            min_weight=cfg.get("min_tsdf_weight", 1.0e-3),
        )
        self.optimizer = TSDFPoseOptimizer(self.volume, keyframes, cfg, use_calib, device)
        self.integrator = TSDFGlobalIntegrator(self.volume, keyframes, cfg, self.optimizer)
        self.async_opt = bool(cfg.get("async_optimize", True))
        self.opt_worker = TSDFGlobalOptThread(self.optimizer, keyframes, cfg) if self.async_opt else None

    def start(self) -> None:
        if not self.enabled:
            return
        if not self.integrator.is_alive():
            self.integrator.start()
        if self.opt_worker is not None and not self.opt_worker.is_alive():
            self.opt_worker.start()

    def shutdown(self) -> None:
        if not self.enabled:
            return
        self.integrator.request_stop()
        self.integrator.join(timeout=2.0)
        if self.opt_worker is not None:
            self.opt_worker.request_stop()
            self.opt_worker.join(timeout=2.0)

    # ------------------------------------------------------------------
    def on_after_backend_solve(self, factor_graph) -> None:
        if not self.enabled:
            return
        idx_tensor = getattr(factor_graph, "last_unique_kf_idx", None)
        if idx_tensor is None:
            return
        indices = [int(i) for i in idx_tensor.cpu().tolist()]
        pin = int(config.get("local_opt", {}).get("pin", 1))
        indices = [i for i in indices if i >= pin]
        if not indices:
            return
        self.integrator.mark_pose_update(indices)
        if self.async_opt and self.opt_worker is not None:
            self.opt_worker.enqueue(indices)
        else:
            self.optimizer.optimize_keyframes(indices, context="factor")
