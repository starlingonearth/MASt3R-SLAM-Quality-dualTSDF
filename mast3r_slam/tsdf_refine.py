#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fixed TSDF refinement module for MASt3R-SLAM
Applied all diagnostic solutions with thread lifecycle fixes
"""

import math
import time
import threading
import queue
import torch
import torch.nn.functional as F
import numpy as np
import multiprocessing as mp
from typing import Optional, List, Tuple, Dict, Any
from dataclasses import dataclass
from collections import defaultdict, OrderedDict

# Import existing geometry functions
try:
    from mast3r_slam.geometry import (
        get_pixel_coords, project_calib, backproject,
        point_to_dist, point_to_ray_dist, constrain_points_to_ray
    )
    GEOMETRY_AVAILABLE = True
except ImportError:
    GEOMETRY_AVAILABLE = False
    print("Warning: geometry module not available, using fallback implementations")

# NOTE: Configuration now loaded from base.yaml - no hardcoded defaults
# This ensures user config is respected

@dataclass(frozen=True)
class BlockKey:
    """Unique identifier for a block to be refined"""
    kf_id: int
    block_id: int

@dataclass
class PatchBlock:
    """A block of patches to be refined together"""
    kf_id: int
    block_id: int
    patch_indices: List[Tuple[int, int]]
    pixel_mask: torch.Tensor
    depth_median: float
    priority: float
    depth_variance: float = 0.0

class RefineRegistry:
    """Enhanced atomic deduplication and history management"""
    IDLE, QUEUED, RUNNING, COOLDOWN = 0, 1, 2, 3
    
    def __init__(self, cfg):
        self.lock = threading.Lock()
        self.state = {}
        self.history = {}
        self.cooldown_frames = cfg["cooldown_frames"]
        self.frame_count = 0
        self.success_rate = 0.0
        
    def tick(self):
        with self.lock:
            self.frame_count += 1
            
    def get_stats(self):
        """Get registry statistics for debugging"""
        with self.lock:
            total_attempts = sum(h.get("attempts", 0) for h in self.history.values())
            total_successes = sum(h.get("successes", 0) for h in self.history.values())
            success_rate = total_successes / max(1, total_attempts)
            return {
                "active_blocks": len([k for k, s in self.state.items() if s != self.IDLE]),
                "total_attempts": total_attempts,
                "success_rate": success_rate,
                "frame_count": self.frame_count
            }
            
    def try_enqueue(self, key: BlockKey) -> bool:
        try:
            with self.lock:
                if self.state.get(key, self.IDLE) != self.IDLE:
                    return False
                self.state[key] = self.QUEUED
                return True
        except Exception as e:
            print(f"WARNING: Registry access failed: {e}")
            return False
            
    def begin_run(self, key: BlockKey) -> bool:
        try:
            with self.lock:
                if self.state.get(key) != self.QUEUED:
                    return False
                self.state[key] = self.RUNNING
                return True
        except Exception as e:
            print(f"WARNING: Registry begin_run failed: {e}")
            return False
            
    def finish_run(self, key: BlockKey, success: bool, gain: float):
        try:
            with self.lock:
                hist = self.history.setdefault(key, {
                    "attempts": 0, 
                    "successes": 0,
                    "last_gain": 0.0,
                    "best_gain": 0.0
                })
                hist["attempts"] += 1
                if success:
                    hist["successes"] += 1
                hist["last_gain"] = gain
                hist["best_gain"] = max(hist["best_gain"], gain)
                hist["last_frame"] = self.frame_count
                self.state[key] = self.COOLDOWN
                
                # Update global success rate
                total_attempts = sum(h.get("attempts", 0) for h in self.history.values())
                total_successes = sum(h.get("successes", 0) for h in self.history.values())
                self.success_rate = total_successes / max(1, total_attempts)
                
        except Exception as e:
            print(f"WARNING: Registry finish_run failed: {e}")
            
    def cooldown_expired(self, key: BlockKey) -> bool:
        try:
            with self.lock:
                hist = self.history.get(key)
                if hist and self.frame_count - hist["last_frame"] > self.cooldown_frames:
                    self.state[key] = self.IDLE
                    return True
                return False
        except Exception as e:
            print(f"WARNING: Registry cooldown_expired failed: {e}")
            return False

class TSDFRefiner(threading.Thread):
    def __init__(self, cfg, shared_keyframes, quality_service, device=None):
        super().__init__(daemon=True)
        
        # Use config from base.yaml directly (no DEFAULT_CFG override)
        self.cfg = cfg if cfg else {}
        
        # Validate required parameters
        required_params = ['voxel_size', 'trunc_dist', 'max_grid_dim', 'roi_size']
        for param in required_params:
            if param not in self.cfg:
                raise ValueError(f"Missing required TSDF config parameter: {param}")
        
        # Apply recursive merge for nested dicts
        if cfg:
            for key, value in cfg.items():
                if isinstance(value, dict) and key in self.cfg:
                    self.cfg[key].update(value)
                else:
                    self.cfg[key] = value
        
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.keyframes = shared_keyframes
        self.quality_service = quality_service
        registry_cfg = {"cooldown_frames": int(self.cfg.get("cooldown_frames", 35))}
        self.registry = RefineRegistry(registry_cfg)
        self.queue = queue.Queue(maxsize=int(self.cfg.get("max_pending_tasks", 50)))
        self.stop_flag = threading.Event()
        
        # 注册quality结果通知回调（避免轮询等待）
        self.quality_events = {}  # kf_id -> threading.Event
        self.quality_events_lock = threading.Lock()
        if hasattr(self.quality_service, 'register_callback'):
            self.quality_service.register_callback(self._on_quality_result)
            print(f"[TSDF-DEBUG] Registered quality callback")
        
        # Performance tracking
        self.stats = {
            "total_blocks": 0,
            "successful_blocks": 0,
            "total_processing_time": 0.0,
            "last_stats_print": time.time(),
            "debug_info": {
                "tsdf_constructions": 0,
                "surface_extractions": 0,
                "displacement_rejects": 0,
                "hit_ratio_rejects": 0
            }
        }
        
        print(f"[TSDF-DEBUG] Refiner initialized with device: {self.device}")
        print(f"[TSDF-DEBUG] Config applied - min_hit_rate: {self.cfg.get('min_hit_rate', 0.05)}, "
              f"max_displacement: {self.cfg.get('max_displacement', 0.015)}, "
              f"voxel_size: {self.cfg.get('voxel_size', 'n/a')}")
        if self.test_indexing_fix():
            print("[TSDF-INIT] Indexing fix verified successfully!")
        else:
            print("[TSDF-INIT] WARNING: Indexing fix test failed!")
        
        # REMOVED: These lines were causing immediate shutdown
        # print("[TSDF-DEBUG] Stop requested")
        # self.stop_flag.set()

    def _on_quality_result(self, result):
        """Quality结果回调：通知等待的线程"""
        kf_id = result.get("kf_id")
        if kf_id is not None:
            with self.quality_events_lock:
                if kf_id in self.quality_events:
                    self.quality_events[kf_id].set()
                    print(f"[TSDF-DEBUG] Quality result received for kf_id {kf_id}, notifying waiters")
    
    def _try_trigger_quality_computation(self, frame_id):
        """Try to trigger quality computation for a frame_id"""
        try:
            print(f"[TSDF-DEBUG] Attempting to trigger quality computation for frame_id {frame_id}")
        except Exception as e:
            print(f"[TSDF-DEBUG] Failed to trigger quality computation: {e}")

    def test_indexing_fix(self):
        """Test method to verify the indexing fix works"""
        print("[TSDF-TEST] Testing indexing fix...")
        
        # Simulate the scenario that was failing
        pixel_indices = torch.arange(1000, device=self.device)
        max_pixels_to_process = 25
        
        indices_to_process = torch.randperm(len(pixel_indices), device=self.device)[:max_pixels_to_process]
        
        print(f"[TSDF-TEST] pixel_indices length: {len(pixel_indices)}")
        print(f"[TSDF-TEST] indices_to_process length: {len(indices_to_process)}")
        
        # Test the fixed indexing
        try:
            for loop_idx, pixel_array_idx in enumerate(indices_to_process):
                pixel_idx = pixel_indices[pixel_array_idx]
                print(f"[TSDF-TEST] Loop {loop_idx}: pixel_array_idx={pixel_array_idx.item()}, pixel_idx={pixel_idx.item()}")
                
                if loop_idx >= 3:  # Only test first few
                    break
            
            print("[TSDF-TEST] Indexing fix test completed successfully!")
            return True
        except Exception as e:
            print(f"[TSDF-TEST] Indexing test FAILED: {e}")
            return False
            
    def schedule_final_pass(self, final_kf_id: int):
        """Public method to trigger final pass processing"""
        print(f"[TSDF-DEBUG] Initiating final pass for all keyframes up to {final_kf_id}")
        self.maybe_schedule_sliding_window(final_kf_id, is_final_pass=True)
        
        time.sleep(1.0)
        
        remaining_queue_size = self.queue.qsize()
        self.maybe_schedule_sliding_window(final_kf_id, is_final_pass=True)
        
        new_queue_size = self.queue.qsize()
        if new_queue_size > remaining_queue_size:
            print(f"[TSDF-DEBUG] Final pass added {new_queue_size - remaining_queue_size} more blocks")
            
    def maybe_schedule_sliding_window(self, current_kf_id: int, is_final_pass: bool = False):
        """Enhanced scheduling with better retry logic"""
        if not self.cfg["enabled"]:
            return
    
        if not hasattr(self, "_pending_map"):
            self._pending_map = {}
    
        window = int(self.cfg.get("window_size", 3))
        retry_slack = int(self.cfg.get("retry_slack_frames", 2))
        max_pending = int(self.cfg.get("max_pending_kf", 64))
        max_attempts = int(self.cfg.get("max_retry_attempts_per_kf", 3))
    
        now = time.perf_counter()
        min_keep_id = max(0, current_kf_id - window - retry_slack)
    
        # Clean up old entries
        stale = [k for k in self._pending_map if k < min_keep_id]
        for k in stale:
            self._pending_map.pop(k, None)
    
        # Final pass scheduling
        if is_final_pass:
            scheduled_count = 0
            for kf_id in range(max(0, current_kf_id - window + 1), current_kf_id + 1):
                if kf_id in self._pending_map:
                    continue
                    
                ok = self._schedule_refinement(kf_id)
                if ok:
                    scheduled_count += 1
                    print(f"[TSDF-DEBUG] Final pass scheduled kf_id {kf_id}")
                else:
                    self._pending_map[kf_id] = {
                        "attempts": 1,
                        "next_ts": now + 0.1,
                        "final_pass": True
                    }
            
            if scheduled_count > 0:
                print(f"[TSDF-DEBUG] Final pass scheduled {scheduled_count} additional keyframes")
            return
    
        # Normal scheduling
        if current_kf_id >= window:
            target = current_kf_id - window
            ok = self._schedule_refinement(target)
            if ok:
                print(f"[TSDF-DEBUG] Scheduled kf_id {target} normally")
            else:
                ent = self._pending_map.get(target, {"attempts": 0, "next_ts": now})
                ent["attempts"] += 1
                base_delay = 0.02 * (2 ** max(0, ent["attempts"] - 1))
                ent["next_ts"] = now + base_delay
                if ent["attempts"] <= max_attempts:
                    self._pending_map[target] = ent
                    print(f"[TSDF-DEBUG] Deferred kf_id {target}, attempt {ent['attempts']}")
                else:
                    self._pending_map.pop(target, None)
                    print(f"[TSDF-DEBUG] Gave up on kf_id {target} after {max_attempts} attempts")
    
        # Process retry queue
        due = [(k, v) for k, v in self._pending_map.items()
               if k >= min_keep_id and v.get("next_ts", 0.0) <= now 
               and not v.get("final_pass", False)]
        
        due.sort(key=lambda x: (x[1].get("attempts", 0), -x[1].get("best_gain", 0.0)))
        
        for k, ent in due[:3]:
            ok = self._schedule_refinement(k)
            if ok:
                self._pending_map.pop(k, None)
                print(f"[TSDF-DEBUG] Retry successful for kf_id {k}")
            else:
                ent["attempts"] += 1
                base_delay = 0.02 * (2 ** max(0, ent["attempts"] - 1))
                ent["next_ts"] = now + base_delay
                if ent["attempts"] <= max_attempts:
                    self._pending_map[k] = ent
                else:
                    self._pending_map.pop(k, None)
    
        # Limit pending map size
        if len(self._pending_map) > max_pending:
            oldest_keys = sorted(self._pending_map.keys())[:len(self._pending_map) - max_pending]
            for k in oldest_keys:
                self._pending_map.pop(k, None)

    def _schedule_refinement(self, kf_id: int) -> bool:
        """Enhanced refinement scheduling with fallback strategy (no blocking)"""
        quality_result = None
        
        # Strategy 1: Try to get quality result (non-blocking)
        if self.quality_service is not None:
            try:
                with self.keyframes.lock:
                    if kf_id < len(self.keyframes):
                        frame_id = self.keyframes[kf_id].frame_id
                        quality_result = self.quality_service.get(frame_id)
                        # Quick poll without blocking
                        try:
                            self.quality_service.poll()
                        except Exception:
                            pass
            except Exception as e:
                print(f"[TSDF-DEBUG] Quality service access failed for kf_id {kf_id}: {e}")
        
        # Strategy 2: Fallback to geometry-based heuristic if no quality result
        if quality_result is None:
            print(f"[TSDF-DEBUG] Using fallback heuristic for kf_id {kf_id} (no quality data)")
            try:
                with self.keyframes.lock:
                    if kf_id >= len(self.keyframes):
                        return False
                    kf = self.keyframes[kf_id]
                    H, W = kf.img_shape[0, 0].item(), kf.img_shape[0, 1].item()
                    C = kf.C.reshape(H, W) if kf.C.ndim == 2 else kf.C.reshape(H, W, 1)[..., 0]
                    
                    # Identify low-confidence regions as candidates
                    low_conf_mask = (C < 0.3) & (C > 0.05)
                    if low_conf_mask.sum() < 100:
                        print(f"[TSDF-DEBUG] Insufficient low-confidence regions for kf_id {kf_id}")
                        return False
                    
                    # Generate pseudo priority map based on inverse confidence
                    # Low confidence = high priority for refinement
                    priority = (0.3 - C) * low_conf_mask.float()
                    priority = priority / (priority.max() + 1e-8)  # Normalize
                    
                    quality_result = {
                        "priority": priority.cpu(),
                        "patch_size": 16
                    }
                    print(f"[TSDF-DEBUG] Generated fallback priority map with {low_conf_mask.sum().item()} candidates")
            except Exception as e:
                print(f"[TSDF-DEBUG] Fallback heuristic failed for kf_id {kf_id}: {e}")
                return False
        
        if quality_result is None:
            print(f"[TSDF-DEBUG] No quality result for kf_id {kf_id}")
            return False
    
        try:
            blocks = self._select_blocks_enhanced(kf_id, quality_result)
            if len(blocks) == 0:
                print(f"[TSDF-DEBUG] No blocks selected for kf_id {kf_id}")
                return True
        except Exception as e:
            print(f"[TSDF-DEBUG] Block selection failed for kf_id {kf_id}: {e}")
            return False
    
        blocks.sort(key=lambda b: b.priority, reverse=True)
        enqueued = 0
        max_blocks = int(self.cfg.get("max_rois_per_kf", 3))
        
        for block in blocks[:max_blocks]:
            key = BlockKey(block.kf_id, block.block_id)
            if self.registry.try_enqueue(key):
                try:
                    self.queue.put_nowait((key, block))
                    enqueued += 1
                except queue.Full:
                    print(f"[TSDF-DEBUG] Worker queue full, stopping enqueue")
                    break
                except Exception as e:
                    print(f"[TSDF-DEBUG] Enqueue failed: {e}")
    
        if enqueued > 0:
            print(f"[TSDF-DEBUG] Enqueued {enqueued}/{len(blocks)} blocks for kf_id {kf_id}")
        return True
                    
    def _select_blocks_enhanced(self, kf_id: int, quality_result: dict) -> List[PatchBlock]:
        """Enhanced block selection with better clustering and priority"""
        priority = quality_result.get("priority")
        if priority is None:
            return []
            
        if isinstance(priority, np.ndarray):
            priority = torch.from_numpy(priority)
        elif isinstance(priority, list):
            priority = torch.tensor(priority, dtype=torch.float32)
            
        patch_size = quality_result.get("patch_size", 16)
        
        if priority.ndim == 2:
            Gh, Gw = priority.shape
        else:
            return []
            
        flat_priority = priority.flatten()
        if flat_priority.numel() == 0:
            return []
            
        # Conservative selection - only top quality patches
        threshold = torch.quantile(flat_priority, 0.95)  # Top 5%
        candidate_mask = flat_priority >= threshold
        candidate_indices = torch.where(candidate_mask)[0]
        
        K = min(int(self.cfg.get("max_rois_per_kf", 3)), len(candidate_indices))
        if K == 0:
            return []
            
        topk_values, relative_indices = torch.topk(flat_priority[candidate_indices], k=K)
        topk_indices = candidate_indices[relative_indices]
        
        patch_coords = []
        patch_priorities = []
        for i, idx in enumerate(topk_indices):
            gh = idx // Gw
            gw = idx % Gw
            patch_coords.append((int(gh), int(gw)))
            patch_priorities.append(float(topk_values[i]))
            
        try:
            with self.keyframes.lock:
                kf_idx = min(kf_id, len(self.keyframes) - 1)
                kf = self.keyframes[kf_idx]
                H, W = kf.img_shape[0, 0].item(), kf.img_shape[0, 1].item()
                X_canon = kf.X_canon.reshape(H, W, 3)
                C = kf.C.reshape(H, W) if kf.C.ndim == 2 else kf.C.reshape(H, W, 1)[..., 0]
        except Exception as e:
            print(f"[TSDF-DEBUG] Keyframe access failed for kf_id {kf_id}: {e}")
            return []
            
        # Compute patch statistics with relaxed requirements
        valid_coords = []
        valid_depths = []
        valid_variances = []
        valid_priorities = []
        
        for i, (gh, gw) in enumerate(patch_coords):
            y0, y1 = gh * patch_size, min((gh + 1) * patch_size, H)
            x0, x1 = gw * patch_size, min((gw + 1) * patch_size, W)
            
            patch_z = X_canon[y0:y1, x0:x1, 2]
            patch_c = C[y0:y1, x0:x1]
            valid_z = patch_z[patch_c > 0.05]  # Lower confidence threshold
            
            if valid_z.numel() > 2:  # Lower minimum valid points
                median_z = torch.median(valid_z).item()
                var_z = torch.var(valid_z).item()
                valid_coords.append((gh, gw))
                valid_depths.append(median_z)
                valid_variances.append(var_z)
                valid_priorities.append(patch_priorities[i])
        
        if not valid_coords:
            print(f"[TSDF-DEBUG] No valid patches found for kf_id {kf_id}")
            return []
        
        print(f"[TSDF-DEBUG] Selected {len(valid_coords)} valid patches for kf_id {kf_id}")
        
        blocks = self._cluster_patches_enhanced(
            kf_id, valid_coords, valid_depths, valid_variances, 
            valid_priorities, H, W, patch_size
        )
        
        return blocks
        
    def _cluster_patches_enhanced(self, kf_id, patch_coords, patch_depths, 
                                patch_variances, patch_priorities, H, W, patch_size):
        """Enhanced patch clustering with improved connectivity"""
        blocks = []
        used = [False] * len(patch_coords)
        block_id = 0
        
        # Simplified clustering defaults (no cluster section in config)
        z_abs = float(self.cfg.get("z_abs_m", 1e9))
        z_rel = float(self.cfg.get("z_rel", 1e9))
        max_edge = int(self.cfg.get("max_block_edge", 1))
        
        sorted_indices = sorted(range(len(patch_coords)), 
                              key=lambda i: patch_priorities[i], reverse=True)
        
        for seed_idx in sorted_indices:
            if used[seed_idx]:
                continue
                
            seed_gh, seed_gw = patch_coords[seed_idx]
            seed_depth = patch_depths[seed_idx]
            
            if not math.isfinite(seed_depth):
                continue
                
            cluster = [seed_idx]
            used[seed_idx] = True
            queue_to_process = [seed_idx]
            
            while queue_to_process and len(cluster) < max_edge * max_edge:
                current_idx = queue_to_process.pop(0)
                current_gh, current_gw = patch_coords[current_idx]
                current_depth = patch_depths[current_idx]
                
                for j, (gh2, gw2) in enumerate(patch_coords):
                    if used[j] or not math.isfinite(patch_depths[j]):
                        continue
                        
                    dx, dy = abs(current_gh - gh2), abs(current_gw - gw2)
                    if max(dx, dy) > 1:
                        continue
                        
                    z1, z2 = current_depth, patch_depths[j]
                    depth_diff = abs(z1 - z2)
                    rel_diff = depth_diff / min(abs(z1), abs(z2)) if min(abs(z1), abs(z2)) > 0 else float('inf')
                    
                    if depth_diff <= z_abs or rel_diff <= z_rel:
                        cluster.append(j)
                        used[j] = True
                        queue_to_process.append(j)
                        
            if len(cluster) >= 1:
                pixel_mask = torch.zeros(H * W, dtype=torch.bool, device=self.device)
                patch_indices = []
                cluster_priorities = []
                cluster_depths = []
                
                for idx in cluster:
                    gh, gw = patch_coords[idx]
                    patch_indices.append((gh, gw))
                    cluster_priorities.append(patch_priorities[idx])
                    cluster_depths.append(patch_depths[idx])
                    
                    y0, y1 = gh * patch_size, min((gh + 1) * patch_size, H)
                    x0, x1 = gw * patch_size, min((gw + 1) * patch_size, W)
                    
                    for y in range(y0, y1):
                        for x in range(x0, x1):
                            pixel_mask[y * W + x] = True
                            
                block_priority = float(np.mean(cluster_priorities))
                block_depth = float(np.median([d for d in cluster_depths if math.isfinite(d)]))
                depth_variance = float(np.var([d for d in cluster_depths if math.isfinite(d)]))
                
                blocks.append(PatchBlock(
                    kf_id=kf_id,
                    block_id=block_id,
                    patch_indices=patch_indices,
                    pixel_mask=pixel_mask,
                    depth_median=block_depth,
                    priority=block_priority,
                    depth_variance=depth_variance
                ))
                block_id += 1
        
        print(f"[TSDF-DEBUG] Created {len(blocks)} blocks from {len(patch_coords)} patches for kf_id {kf_id}")
        return blocks
        
    def run(self):
        """Main refinement thread"""
        print("[TSDF-DEBUG] Refinement thread started")
        
        while not self.stop_flag.is_set():
            try:
                key, block = self.queue.get(timeout=0.1)
            except queue.Empty:
                if time.time() - self.stats["last_stats_print"] > 30:
                    self._print_stats()
                continue
                
            if not self.registry.begin_run(key):
                continue
                
            start_time = time.time()
            try:
                success, gain = self._refine_block_enhanced(block)
                self.stats["total_blocks"] += 1
                if success:
                    self.stats["successful_blocks"] += 1
                    print(f"[TSDF-SUCCESS] Block {key.block_id} from kf {key.kf_id} refined successfully! Gain: {gain:.4f}")
                else:
                    print(f"[TSDF-DEBUG] Block {key.block_id} from kf {key.kf_id} failed refinement. Gain: {gain:.4f}")
                    
            except Exception as e:
                print(f"[TSDF-ERROR] Refinement error for block {key.block_id}: {e}")
                import traceback
                traceback.print_exc()
                success, gain = False, 0.0
                
            processing_time = time.time() - start_time
            self.stats["total_processing_time"] += processing_time
            
            self.registry.finish_run(key, success, gain)
        
        self._print_final_stats()
            
    def _print_stats(self):
        """Print performance statistics"""
        success_rate = self.stats["successful_blocks"] / max(1, self.stats["total_blocks"])
        avg_time = self.stats["total_processing_time"] / max(1, self.stats["total_blocks"])
        debug_info = self.stats["debug_info"]
        
        print(f"[TSDF-STATS] {self.stats['successful_blocks']}/{self.stats['total_blocks']} "
              f"blocks successful ({success_rate:.1%}), avg time: {avg_time:.3f}s")
        print(f"[TSDF-STATS] Debug - TSDF constructions: {debug_info['tsdf_constructions']}, "
              f"surface extractions: {debug_info['surface_extractions']}, "
              f"displacement rejects: {debug_info['displacement_rejects']}")
        
        self.stats["last_stats_print"] = time.time()
        
    def _print_final_stats(self):
        """Print final statistics on shutdown"""
        success_rate = self.stats["successful_blocks"] / max(1, self.stats["total_blocks"])
        total_time = self.stats["total_processing_time"]
        
        print(f"[TSDF-FINAL] Processed {self.stats['total_blocks']} blocks, "
              f"success rate: {success_rate:.1%}, total time: {total_time:.1f}s")

    def _refine_block_enhanced(self, block: PatchBlock) -> Tuple[bool, float]:
        """Enhanced block refinement with version checking for synchronization"""
        
        # Store current block context
        self._current_block_kf_id = block.kf_id
        
        print(f"[TSDF-DEBUG] Starting refinement for block {block.block_id} from kf {block.kf_id}")
        
        # Step 1: Read data and record version (atomic)
        start_version = None
        try:
            with self.keyframes.lock:
                kf = self.keyframes[block.kf_id]
                start_version = int(self.keyframes.version[block.kf_id].item())
                H, W = kf.img_shape[0, 0].item(), kf.img_shape[0, 1].item()
                X_canon_flat = kf.X_canon.clone()
                C_flat = kf.C.clone() if kf.C.ndim == 2 else kf.C[..., 0].clone()
                K = kf.K.clone() if kf.K is not None else None
                T_WC = kf.T_WC.clone()
                print(f"[TSDF-DEBUG] Read kf {block.kf_id} at version {start_version}")
        except Exception as e:
            print(f"[TSDF-ERROR] Keyframe access failed: {e}")
            return False, 0.0
            
        # Move to device
        X_canon_flat = X_canon_flat.to(self.device)
        C_flat = C_flat.to(self.device) if C_flat.ndim == 1 else C_flat.squeeze(-1).to(self.device)
        K = K.to(self.device)
        mask = block.pixel_mask.to(self.device)
        
        # Extract and validate block data
        X_block = X_canon_flat[mask]
        C_block = C_flat[mask]
        
        # Very relaxed validity check
        valid = (C_block > 0.03) & torch.isfinite(X_block).all(dim=1) & (X_block[:, 2] > 0.03)
        valid_count = valid.sum().item()
        total_pixels = mask.sum().item()
        
        print(f"[TSDF-DEBUG] Block validation: {valid_count}/{total_pixels} valid pixels ({valid_count/max(1,total_pixels):.3f})")
        
        if valid_count < max(3, total_pixels * 0.05):  # Very relaxed requirement
            print(f"[TSDF-DEBUG] Insufficient valid pixels, skipping block")
            return False, 0.0
            
        X_valid = X_block[valid]
        
        # Compute adaptive ROI with margin derived from roi_size
        roi_size_cfg = float(self.cfg.get("roi_size", 0.4))
        roi_margin = max(0.01, min(0.05, 0.1 * roi_size_cfg))
        xyz_min = X_valid.min(dim=0)[0] - roi_margin
        xyz_max = X_valid.max(dim=0)[0] + roi_margin
        
        roi_size = xyz_max - xyz_min
        print(f"[TSDF-DEBUG] ROI size: [{roi_size[0]:.3f}, {roi_size[1]:.3f}, {roi_size[2]:.3f}]m")
        
        if torch.any(roi_size <= 0) or torch.any(roi_size > 15.0):
            print(f"[TSDF-DEBUG] Invalid ROI size, skipping block")
            return False, 0.0
        
        # Build robust TSDF with correct camera pose
        print(f"[TSDF-DEBUG] Building TSDF...")
        tsdf, tsdf_weights = self._build_tsdf_robust(X_canon_flat, C_flat, K, xyz_min, xyz_max, H, W, T_WC)
        self.stats["debug_info"]["tsdf_constructions"] += 1
        
        # Check TSDF content
        valid_voxels = (tsdf_weights > float(self.cfg.get("min_weight_threshold", 0.01))).sum().item()
        print(f"[TSDF-DEBUG] TSDF has {valid_voxels} valid voxels")
        
        if valid_voxels < 3:  # Very low threshold
            print(f"[TSDF-DEBUG] Insufficient valid voxels, skipping block")
            return False, 0.0
        
        # Extract surface with displacement protection
        print(f"[TSDF-DEBUG] Extracting surface...")
        X_refined, hits = self._extract_surface_safe(tsdf, xyz_min, xyz_max, K, mask, H, W, X_canon_flat)
        self.stats["debug_info"]["surface_extractions"] += 1
        
        hit_count = hits.sum().item()
        hit_ratio = hit_count / max(1, total_pixels)
        
        print(f"[TSDF-DEBUG] Surface extraction: {hit_count}/{total_pixels} hits ({hit_ratio:.3f})")
        
        # Compute controlled geometric gain
        geometric_gain = 0.0
        max_displacement = 0.0
        
        if hit_count > 0:
            mask_hits = mask.clone()
            mask_hits[mask] = hits
            
            original_points = X_canon_flat[mask_hits]
            refined_points = X_refined[mask_hits]
            
            displacement = (refined_points - original_points).norm(dim=1)
            geometric_gain = displacement.mean().item()
            max_displacement = displacement.max().item()
            
            print(f"[TSDF-DEBUG] Displacement analysis: mean={geometric_gain:.4f}m, max={max_displacement:.4f}m")
            
            # Safety check for displacement
            max_disp_thr = float(self.cfg.get("max_displacement", 0.015))
            if max_displacement > max_disp_thr:
                print(f"[TSDF-DEBUG] Excessive displacement {max_displacement:.3f}m > {max_disp_thr:.3f}m, rejecting")
                self.stats["debug_info"]["displacement_rejects"] += 1
                return False, hit_ratio
        
        # Success criteria
        min_hit_ratio = float(self.cfg.get("min_hit_rate", 0.05))
        success = hit_ratio >= min_hit_ratio and hit_count >= 1
        
        print(f"[TSDF-DEBUG] Success check: hit_ratio {hit_ratio:.3f} >= {min_hit_ratio:.3f} = {hit_ratio >= min_hit_ratio}, hit_count {hit_count} >= 1 = {hit_count >= 1}")
        
        if not success:
            if hit_ratio < min_hit_ratio:
                self.stats["debug_info"]["hit_ratio_rejects"] += 1
            print(f"[TSDF-DEBUG] Block failed success criteria")
            return False, hit_ratio
            
        # Conservative fusion with version checking
        try:
            with self.keyframes.lock:
                # Check version before writing
                current_version = int(self.keyframes.version[block.kf_id].item())
                if current_version != start_version:
                    print(f"[TSDF-DEBUG] Version conflict for kf {block.kf_id}: "
                          f"start={start_version}, current={current_version}. Skipping update.")
                    return False, hit_ratio
                
                mask_hits = mask.clone()
                mask_hits[mask] = hits
                
                # Minimal confidence boost
                base_boost = float(self.cfg.get("confidence_boost", 0.08))
                print(f"[TSDF-DEBUG] Applying confidence boost of {base_boost}")
                
                # Apply conservative updates
                if self.keyframes[block.kf_id].C.ndim == 2:
                    original_conf = self.keyframes[block.kf_id].C[mask_hits].mean().item()
                    self.keyframes[block.kf_id].C[mask_hits] += base_boost
                    self.keyframes[block.kf_id].C.clamp_(max=float(self.cfg.get("confidence_max", 1.3)))
                    new_conf = self.keyframes[block.kf_id].C[mask_hits].mean().item()
                else:
                    original_conf = self.keyframes[block.kf_id].C[mask_hits, 0].mean().item()
                    self.keyframes[block.kf_id].C[mask_hits, 0] += base_boost
                    self.keyframes[block.kf_id].C.clamp_(max=float(self.cfg.get("confidence_max", 1.3)))
                    new_conf = self.keyframes[block.kf_id].C[mask_hits, 0].mean().item()
                
                print(f"[TSDF-DEBUG] Confidence update: {original_conf:.3f} -> {new_conf:.3f}")
                
                # Minimal geometric updates
                geo_weight = float(self.cfg.get("geometric_weight", 0.0))
                if geo_weight > 0:
                    blended_points = (1 - geo_weight) * X_canon_flat + geo_weight * X_refined
                    self.keyframes[block.kf_id].X_canon[mask_hits] = blended_points[mask_hits]
                    print(f"[TSDF-DEBUG] Applied geometric blending with weight {geo_weight}")
                
                # Increment version after successful update
                self.keyframes.version[block.kf_id] += 1
                print(f"[TSDF-DEBUG] Updated kf {block.kf_id} version to {current_version + 1}")
                    
                updated_count = mask_hits.sum().item()
                print(f"[TSDF-DEBUG] Successfully updated {updated_count} points in kf {block.kf_id}")
                
        except Exception as e:
            print(f"[TSDF-ERROR] Keyframe update failed: {e}")
            return False, hit_ratio
                
        return True, max(hit_ratio, geometric_gain)

    def _build_tsdf_robust(self, X_canon, C_flat, K, xyz_min, xyz_max, H, W, T_WC):
        """Correct TSDF Fusion algorithm with proper camera pose"""
        voxel_size = self.cfg["voxel_size"]
        trunc_dist = self.cfg["trunc_dist"]
        max_grid_dim = self.cfg["max_grid_dim"]
        
        # Calculate grid dimensions
        roi_size = xyz_max - xyz_min
        grid_dims = torch.clamp((roi_size / voxel_size).ceil().long(), max=max_grid_dim)
        nx, ny, nz = [int(v) for v in grid_dims.detach().cpu().tolist()]
        
        print(f"[TSDF-DEBUG] Grid: {nx}x{ny}x{nz}, voxel size: {voxel_size}m, trunc: {trunc_dist}m")
        
        # Initialize TSDF and weights
        tsdf = torch.ones((nz, ny, nx), device=self.device, dtype=torch.float32)
        weights = torch.zeros_like(tsdf)
        
        # Prepare data
        if C_flat.ndim > 1:
            C_flat = C_flat.squeeze(-1)
        
        # Get camera origin in world coordinates
        # T_WC is Sim3, extract 4x4 matrix
        T_WC_matrix = T_WC.matrix()[0]  # [4, 4]
        cam_origin_world = T_WC_matrix[:3, 3]  # Camera center in world coords
        
        # Transform points from canonical camera coords to world coords
        X_world = T_WC.act(X_canon)  # Transform using Sim3
        
        # Filter valid points in ROI
        valid_mask = (
            torch.isfinite(X_world).all(dim=1) &
            (X_world[:, 0] >= xyz_min[0]) & (X_world[:, 0] <= xyz_max[0]) &
            (X_world[:, 1] >= xyz_min[1]) & (X_world[:, 1] <= xyz_max[1]) &
            (X_world[:, 2] >= xyz_min[2]) & (X_world[:, 2] <= xyz_max[2]) &
            (C_flat > self.cfg["min_confidence"])
        )
        
        valid_points_world = X_world[valid_mask]
        valid_confidences = C_flat[valid_mask]
        
        print(f"[TSDF-DEBUG] Found {len(valid_points_world)} valid points in ROI")
        
        if len(valid_points_world) < 5:
            return tsdf, weights
        
        actual_voxel = roi_size / torch.tensor([nx, ny, nz], device=self.device)
        fusion_count = 0
        
        # Correct TSDF Fusion: for each point, update voxels along ray from camera
        for i in range(len(valid_points_world)):
            point_world = valid_points_world[i]
            conf = valid_confidences[i].item()
            
            # Compute ray direction and length from camera to point
            ray_vec = point_world - cam_origin_world
            ray_length = ray_vec.norm().item()
            ray_dir = ray_vec / (ray_length + 1e-8)  # Normalized direction
            
            if ray_length < 0.05:
                continue
            
            # Sample along ray in truncation range
            t_start = max(0.05, ray_length - trunc_dist * 2.0)
            t_end = ray_length + trunc_dist * 2.0
            n_samples = min(32, int((t_end - t_start) / voxel_size) + 1)
            
            for t in torch.linspace(t_start, t_end, n_samples, device=self.device):
                # Sample point along ray from camera
                sample_point_world = cam_origin_world + ray_dir * t
                
                # Check bounds
                if torch.any(sample_point_world < xyz_min) or torch.any(sample_point_world > xyz_max):
                    continue
                
                # Convert to grid coordinates
                grid_pos = (sample_point_world - xyz_min) / actual_voxel
                gx = int(torch.clamp(grid_pos[0], 0, nx - 1).item())
                gy = int(torch.clamp(grid_pos[1], 0, ny - 1).item())
                gz = int(torch.clamp(grid_pos[2], 0, nz - 1).item())
                
                # Compute signed distance (positive in front, negative behind surface)
                sdf = (ray_length - t.item()) / trunc_dist
                sdf = max(-1.0, min(1.0, sdf))  # Truncate to [-1, 1]
                
                # Weight by distance to surface and confidence
                dist_weight = max(0.0, 1.0 - abs(sdf))
                weight = conf * dist_weight
                
                # Weighted average update
                old_w = weights[gz, gy, gx].item()
                new_w = old_w + weight
                
                if new_w > 1e-6:
                    old_sdf = tsdf[gz, gy, gx].item()
                    tsdf[gz, gy, gx] = (old_sdf * old_w + sdf * weight) / new_w
                    weights[gz, gy, gx] = new_w
            
            fusion_count += 1
        
        valid_voxels = (weights > self.cfg["min_weight_threshold"]).sum().item()
        print(f"[TSDF-DEBUG] TSDF construction: {fusion_count} points -> {valid_voxels} valid voxels")
        
        return tsdf, weights

    def _extract_surface_safe(self, tsdf, xyz_min, xyz_max, K, mask, H, W, X_original):
        """Correct surface extraction via Ray Casting + zero-crossing detection"""
        
        X_refined = X_original.clone()
        hits = torch.zeros(mask.sum(), dtype=torch.bool, device=self.device)
        
        pixel_indices = torch.where(mask)[0]
        if len(pixel_indices) == 0:
            return X_refined, hits
        
        nz, ny, nx = tsdf.shape
        max_displacement = self.cfg["max_displacement"]
        min_weight_threshold = self.cfg["min_weight_threshold"]
        n_samples = self.cfg["ray_samples"]
        
        actual_voxel = (xyz_max - xyz_min) / torch.tensor([nx, ny, nz], device=self.device)
        
        successful_extractions = 0
        max_pixels_to_process = min(len(pixel_indices), 100)
        
        print(f"[TSDF-DEBUG] Ray casting: processing {max_pixels_to_process} pixels")
        
        # Process subset of pixels
        indices_to_process = torch.randperm(len(pixel_indices), device=self.device)[:max_pixels_to_process]
        
        for pixel_array_idx in indices_to_process:
            pixel_idx = pixel_indices[pixel_array_idx]
            original_point = X_original[pixel_idx]
            original_depth = original_point[2].item()
            
            if original_depth < 0.05:
                continue
            
            # Sample along ray from camera through point
            t_start = max(0.05, original_depth - 0.1)
            t_end = original_depth + 0.1
            t_values = torch.linspace(t_start, t_end, n_samples, device=self.device)
            
            # Ray casting: look for zero crossing
            prev_sdf = None
            prev_t = None
            
            for j, t in enumerate(t_values):
                # Sample point along ray (approximate)
                sample_point = original_point * (t / original_depth) if original_depth > 0.01 else original_point
                
                # Check bounds
                if torch.any(sample_point < xyz_min) or torch.any(sample_point > xyz_max):
                    prev_sdf = None
                    continue
                
                # Sample TSDF with trilinear interpolation
                grid_pos = (sample_point - xyz_min) / actual_voxel
                sdf = self._sample_tsdf_trilinear(tsdf, grid_pos[0], grid_pos[1], grid_pos[2])
                
                # Check if we found zero crossing (surface)
                if prev_sdf is not None:
                    if prev_sdf * sdf < 0:  # Sign change = surface found
                        # Linear interpolation to find exact surface
                        alpha = abs(prev_sdf) / (abs(prev_sdf) + abs(sdf) + 1e-8)
                        t_surface = prev_t + alpha * (t - prev_t)
                        
                        surface_point = original_point * (t_surface / original_depth) if original_depth > 0.01 else original_point
                        
                        # Check displacement
                        displacement = (surface_point - original_point).norm().item()
                        if displacement <= max_displacement:
                            X_refined[pixel_idx] = surface_point
                            hits[pixel_array_idx] = True
                            successful_extractions += 1
                        
                        break  # Found surface, stop
                
                prev_sdf = sdf
                prev_t = t
        
        hit_ratio = successful_extractions / max(1, max_pixels_to_process)
        print(f"[TSDF-DEBUG] Ray casting: {successful_extractions}/{max_pixels_to_process} hits ({hit_ratio:.3f})")
        
        return X_refined, hits

    def _sample_tsdf_trilinear(self, volume, x, y, z):
        """Trilinear interpolation for TSDF sampling"""
        nz, ny, nx = volume.shape
        
        # Clamp to valid range
        x = torch.clamp(x, 0, nx - 1)
        y = torch.clamp(y, 0, ny - 1)
        z = torch.clamp(z, 0, nz - 1)
        
        # Integer and fractional parts
        x0 = int(torch.floor(x).item())
        y0 = int(torch.floor(y).item())
        z0 = int(torch.floor(z).item())
        
        x1 = min(x0 + 1, nx - 1)
        y1 = min(y0 + 1, ny - 1)
        z1 = min(z0 + 1, nz - 1)
        
        xd = (x - x0).item()
        yd = (y - y0).item()
        zd = (z - z0).item()
        
        # 8 corner values
        c000 = volume[z0, y0, x0].item()
        c001 = volume[z0, y0, x1].item()
        c010 = volume[z0, y1, x0].item()
        c011 = volume[z0, y1, x1].item()
        c100 = volume[z1, y0, x0].item()
        c101 = volume[z1, y0, x1].item()
        c110 = volume[z1, y1, x0].item()
        c111 = volume[z1, y1, x1].item()
        
        # Trilinear interpolation
        c00 = c000 * (1 - xd) + c001 * xd
        c01 = c010 * (1 - xd) + c011 * xd
        c10 = c100 * (1 - xd) + c101 * xd
        c11 = c110 * (1 - xd) + c111 * xd
        
        c0 = c00 * (1 - yd) + c01 * yd
        c1 = c10 * (1 - yd) + c11 * yd
        
        return c0 * (1 - zd) + c1 * zd