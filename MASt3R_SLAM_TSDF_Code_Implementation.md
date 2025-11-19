# MASt3R-SLAM 局部 TSDF 精化：代码实现概览

本文档对应当前代码库（`mast3r_slam/tsdf_refine.py` 等）中 **质量驱动局部 TSDF 精化** 的实现细节，适合作为阅读源码的导览。

---

## 1. 入口与配置

```python
# main.py
from mast3r_slam.tsdf_refine import TSDFRefiner

tsdf_cfg = config.get("tsdf_refine", {})
tsdf_enabled = tsdf_cfg.get("enabled", False)

if tsdf_enabled:
    tsdf_refiner = TSDFRefiner(
        cfg=tsdf_cfg,
        shared_keyframes=keyframes,
        quality_service=quality_service,
        device=device,
    )
    tsdf_refiner.start()
```

- 通过 `config/base.yaml` 下的 `tsdf_refine` 段落控制线程开关、滑动窗口、体素参数、shutdown 行为等。
- 主循环在每个新关键帧写入 `SharedKeyframes` 后调用 `tsdf_refiner.maybe_schedule_sliding_window(kf_id)`，并在退出前执行 `schedule_final_pass → drain queue → stop_flag`（`main.py:224-420`）。

---

## 2. 数据结构与服务契约

### 2.1 SharedKeyframes & Quality Service
- `SharedKeyframes` 承载关键帧的 pointmap (`X_canon`)、置信度 (`C`) 以及姿态 (`T_WC`)；TSDFRefiner 的读写都在持锁状态下进行，避免与 tracker 冲突。
- `AsynchronousQualityService` 通过 multiprocessing 队列接收 tracker 提交的 patch 级残差与置信度，批量计算优先级热图并缓存。TSDFRefiner 在调度阶段会轮询 `quality_service.get(frame_id)`，超时会放弃该关键帧的调度（`mast3r_slam/tsdf_refine.py:331-365`）。

### 2.2 Dataclass 封装
```python
@dataclass(frozen=True)
class BlockKey:
    kf_id: int
    block_id: int

@dataclass
class PatchBlock:
    kf_id: int
    block_id: int
    patch_indices: List[Tuple[int, int]]
    pixel_mask: torch.Tensor
    depth_median: float
    priority: float
    depth_variance: float = 0.0
```
- `BlockKey` 作为去重标识，`PatchBlock` 则携带 ROI 像素掩码、深度统计与优先级，方便后续 TSDF 构建。

### 2.3 RefineRegistry
- 维护 `(kf_id, block_id)` 的状态机：`IDLE → QUEUED → RUNNING → COOLDOWN`。
- 记录历史成功率与最佳增益，用于调度策略和调试统计。

---

## 3. TSDFRefiner 线程

### 3.1 初始化
```python
class TSDFRefiner(threading.Thread):
    def __init__(self, cfg, shared_keyframes, quality_service, device=None):
        super().__init__(daemon=True)
        self.cfg = cfg
        self.keyframes = shared_keyframes
        self.quality_service = quality_service
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.registry = RefineRegistry({...})
        self.queue = queue.Queue(maxsize=cfg.get("max_pending_tasks", 50))
        self.stop_flag = threading.Event()
        self.stats = {...}
```
- 启动时打印配置摘要并执行一次 `_test_indexing_fix()`，确保后续像素索引逻辑安全。

### 3.2 调度：`maybe_schedule_sliding_window`
1. 维持 `self._pending_map` 记录尚未调度成功的关键帧（含重试次数、下一次调度时间、final-pass 标记）。
2. 对目标关键帧调用 `_schedule_refinement(kf_id)`：
   - 等待质量结果（指数退避，最长 `quality_wait_ms`）。
   - 调用 `_select_blocks_enhanced` 根据热图选取 Top-K patch，生成 `PatchBlock` 列表。
   - 逐个 block 申请 `RefineRegistry.try_enqueue`，写入线程队列。

### 3.3 线程循环：`run`
```python
while not self.stop_flag.is_set():
    try:
        key, block = self.queue.get(timeout=0.1)
    except queue.Empty:
        maybe_print_stats()
        continue

    if not self.registry.begin_run(key):
        continue

    start = time.time()
    try:
        success, gain = self._refine_block_enhanced(block)
        update_stats(success, gain, time.time() - start)
    finally:
        self.registry.finish_run(key, success, gain)
```
- 定期输出 `TSDF-STATS`，在 `stop_flag` 置位后打印最终统计。

---

## 4. `_refine_block_enhanced` 关键步骤

1. **读取关键帧数据**：在锁内拷贝 pointmap (`X_canon_flat`)、置信度 (`C_flat`)、相机内参（如果存在）。
2. **像素筛选**：对 block 掩码下的点检查置信度、有限性、深度下限，保证有效像素数不少于阈值。
3. **ROI 估计**：依据有效点的 min/max，加上 `roi_margin` 得到包围盒，确保尺寸合理（排除过大/过小 ROI）。
4. **TSDF 构建**：调用 `_build_tsdf_robust`：
   - 在 ROI 内根据 `voxel_size` 计算网格维度（限制 `max_grid_dim`）。
   - 滤除低置信度点，遍历有效点沿射线采样，采用标准截断 TSDF 融合策略：
     ```python
     sdf = (depth - t) / trunc_dist
     weight = conf * max(0, 1 - |sdf|)
     tsdf_voxel = (old_sdf * old_w + sdf * weight) / (old_w + weight)
     ```
5. **射线回投**：`_extract_surface_safe` 对 ROI 内像素进行 ray marching（`ray_samples` 次），检测 TSDF 零交叉，统计命中次数和位移：
   - 若 `hit_ratio < min_hit_rate` 或最大位移超过 `max_displacement`，放弃该 block。
6. **写回**：当命中条件满足时，获取关键帧写锁，执行：
   - 置信度提升：`C[mask_hits] = clamp(C + confidence_boost, max=confidence_max)`。
   - （可选）几何混合：`X_canon = (1 - geo_w) * X_original + geo_w * X_refined`。

最终返回 `(success, gain)`，其中 `gain` 取 `max(hit_ratio, geometric_gain)`，供统计与调试使用。

---

## 5. 质量驱动 ROI 选择 `_select_blocks_enhanced`

1. 将质量热图 `priority` 展平，提取 95 百分位以上的候选 patch。
2. 结合 patch 深度中位数、方差、置信度阈值过滤无效 patch。
3. 通过 `_cluster_patches_enhanced` 把邻接 patch 合并为 block：
   - 以优先级排序，广度优先搜索扩张，限制 block 边长和深度差。
   - 生成像素掩码、patch 列表、block 统计等。
4. 返回 block 列表供调度阶段入队。

---

## 6. Shutdown 流程
- 主线程在退出前：
  1. 调用 `schedule_final_pass(final_kf_id)` 确保窗口内关键帧被再次覆盖。
  2. 循环等待队列清空（`qsize == 0` 且命中率无增长），过程中打印进度并遵循 `max_shutdown_wait_s / progress_stall_s` 策略。
  3. 打印总计统计并设置 `stop_flag` → `join()` 线程。

---

## 7. 与后续工作的接口
- 若未来需要引入 **全局 TSDF** 或 **位姿精化**，可以在 `_refine_block_enhanced` 成功后把 ROI TSDF/mesh 结果交给其它模块，或在 `FactorGraph` 中新增 TSDF 残差项。
- 若要在世界坐标系运行，可在 ROI 构建前将 `X_canon` 通过 `T_WC` 变换到世界系，并在 `_build_tsdf_robust` 中记录 TSDF 原点与方向。

该实现以“质量→局部 TSDF → 置信度调整”为主线，与当前仓库代码保持一致，可直接对应源码与配置。
