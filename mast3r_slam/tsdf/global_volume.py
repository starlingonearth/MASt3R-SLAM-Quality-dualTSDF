import math
import threading
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np


@dataclass
class VoxelData:
    tsdf: float = 1.0
    weight: float = 0.0


class TSDFVolume:
    """Sparse TSDF volume (voxel hashing via python dict)."""

    def __init__(
        self,
        voxel_size: float,
        truncation: float,
        max_weight: float = 100.0,
        min_weight: float = 1.0e-3,
    ) -> None:
        self.voxel_size = float(voxel_size)
        self.truncation = float(truncation)
        self.max_weight = float(max_weight)
        self.min_weight = float(min_weight)
        self._voxels: Dict[Tuple[int, int, int], VoxelData] = {}
        self._lock = threading.RLock()

    # ------------------------------------------------------------------
    # Integration
    # ------------------------------------------------------------------
    def integrate(
        self,
        points_world: np.ndarray,
        confidences: np.ndarray,
        cam_origin: np.ndarray,
        step_scale: float = 0.5,
    ) -> int:
        """Fuse a batch of world-space points along their viewing rays."""

        if points_world.size == 0:
            return 0

        cam_origin = cam_origin.reshape(3)
        step = max(self.voxel_size * step_scale, 1.0e-4)
        trunc = self.truncation
        fused = 0

        with self._lock:
            for point, conf in zip(points_world, confidences):
                ray = point - cam_origin
                ray_length = np.linalg.norm(ray)
                if not np.isfinite(ray_length) or ray_length < 1.0e-4:
                    continue
                direction = ray / ray_length
                max_distance = ray_length + trunc
                num_samples = max(1, int(max_distance / step))
                distances = np.linspace(0.0, max_distance, num_samples)

                for dist in distances:
                    sample = cam_origin + dist * direction
                    sdf = ray_length - dist
                    if abs(sdf) > trunc:
                        continue
                    tsdf_value = np.clip(sdf / trunc, -1.0, 1.0)
                    weight = conf * math.exp(-abs(sdf) / trunc)
                    self._update_voxel(sample, tsdf_value, weight)
                fused += 1
        return fused

    def _update_voxel(self, point_world: np.ndarray, tsdf: float, weight: float) -> None:
        if weight <= 0.0:
            return
        key = self._world_to_voxel(point_world)
        voxel = self._voxels.get(key)
        if voxel is None:
            voxel = VoxelData(tsdf=tsdf, weight=weight)
            self._voxels[key] = voxel
            return

        total_weight = min(voxel.weight + weight, self.max_weight)
        voxel.tsdf = (voxel.tsdf * voxel.weight + tsdf * weight) / max(
            1.0e-9, total_weight
        )
        voxel.weight = total_weight

    # ------------------------------------------------------------------
    # Query helpers
    # ------------------------------------------------------------------
    def query(self, point_world: np.ndarray) -> Tuple[Optional[float], Optional[np.ndarray]]:
        """Return TSDF value and gradient for the given world point."""

        with self._lock:
            coord = self._world_to_voxel(point_world)
            voxel = self._voxels.get(coord)
            if voxel is None or voxel.weight < self.min_weight:
                return None, None
            tsdf_value = voxel.tsdf
            gradient = self._estimate_gradient(point_world)
            if gradient is None:
                return tsdf_value, None
            return tsdf_value, gradient

    def _estimate_gradient(self, point_world: np.ndarray) -> Optional[np.ndarray]:
        offsets = np.eye(3, dtype=int)
        grad = np.zeros(3, dtype=np.float64)
        denom = 0.0
        base_coord = self._world_to_voxel(point_world)
        for axis in range(3):
            pos_key = tuple(base_coord[i] + offsets[axis, i] for i in range(3))
            neg_key = tuple(base_coord[i] - offsets[axis, i] for i in range(3))
            v_pos = self._voxels.get(pos_key)
            v_neg = self._voxels.get(neg_key)
            if v_pos is None or v_neg is None:
                continue
            if v_pos.weight < self.min_weight or v_neg.weight < self.min_weight:
                continue
            grad[axis] = (v_pos.tsdf - v_neg.tsdf) / (2.0 * self.voxel_size)
            denom += 1.0
        if denom == 0.0:
            return None
        norm = np.linalg.norm(grad)
        if norm < 1.0e-9:
            return None
        return grad / norm

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    def _world_to_voxel(self, point_world: np.ndarray) -> Tuple[int, int, int]:
        return tuple(np.floor(point_world / self.voxel_size).astype(int).tolist())

    def stats(self) -> Dict[str, float]:
        with self._lock:
            valid = sum(1 for v in self._voxels.values() if v.weight >= self.min_weight)
            total = len(self._voxels)
        return {"valid_voxels": valid, "total_voxels": total}
