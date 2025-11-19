# main.py 新增变更说明 (Detailed Changes)

本文档详细记录了 `main.py` 相对于原始基线版本的所有新增内容。

## 目录
1. [导入模块变更](#1-导入模块变更)
2. [后端进程变更](#2-后端进程变更)
3. [主程序初始化变更](#3-主程序初始化变更)
4. [主循环变更](#4-主循环变更)
5. [TSDF关闭流程](#5-tsdf关闭流程)
6. [结果保存变更](#6-结果保存变更)
7. [进程关闭变更](#7-进程关闭变更)

---

## 1. 导入模块变更

### 新增导入 (Line 25)
```python
from mast3r_slam.quality import AsynchronousQualityService  # NEW
```

**说明**: 引入异步质量评估服务，用于评估和管理点云质量属性。

---

## 2. 后端进程变更

### 2.1 `run_backend` 函数签名变更 (Line 72)

**原始版本**:
```python
def run_backend(cfg, model, states, keyframes, K):
```

**新版本**:
```python
def run_backend(cfg, model, states, keyframes, K, tsdf_global_cfg):  # NEW: Added tsdf_global_cfg parameter
```

**说明**: 新增 `tsdf_global_cfg` 参数，用于配置全局TSDF管理器。

### 2.2 TSDF Global Manager 初始化 (Lines 78-89)

```python
# NEW: TSDF Global Manager initialization
tsdf_manager = None
if tsdf_global_cfg.get("enabled", False):
    from mast3r_slam.tsdf import TSDFGlobalManager

    tsdf_manager = TSDFGlobalManager(
        keyframes=keyframes,
        cfg=tsdf_global_cfg,
        use_calib=config.get("use_calib", False),
        device=device,
    )
    tsdf_manager.start()
```

**说明**: 
- 根据配置初始化全局TSDF管理器
- 用于在后端优化后进行TSDF融合和重建

### 2.3 后端优化后回调 (Lines 150-155)

```python
# NEW: TSDF Global Manager post-solve callback
if tsdf_manager is not None:
    try:
        tsdf_manager.on_after_backend_solve(factor_graph)
    except Exception as exc:
        print(f"[TSDF-GLOBAL] Optimization failed: {exc}")
```

**说明**: 
- 在因子图优化完成后调用TSDF管理器
- 更新TSDF体积以反映最新的相机位姿

### 2.4 TSDF Manager 关闭 (Lines 160-162)

```python
# NEW: TSDF Global Manager shutdown
if tsdf_manager is not None:
    tsdf_manager.shutdown()
```

**说明**: 在后端进程结束时正确关闭TSDF管理器。

---

## 3. 主程序初始化变更

### 3.1 质量服务初始化 (Line 187)

```python
# NEW: Asynchronous Quality Service initialization
quality_service = AsynchronousQualityService(manager=manager)
```

**说明**: 
- 创建异步质量评估服务
- 用于计算和管理点云的质量指标（如误差、置信度等）

### 3.2 质量服务附加到跟踪器 (Line 245)

```python
# NEW: Attach quality service to tracker
tracker.quality_service = quality_service
```

**说明**: 将质量服务附加到跟踪器，使跟踪过程中可以进行质量评估。

### 3.3 TSDF Refiner 初始化 (Lines 247-288)

```python
# NEW: TSDF Refiner initialization section
# ------------------------------
# TSDF Refiner initialization
# ------------------------------
tsdf_refiner = None
tsdf_enabled = config.get("tsdf_refine", {}).get("enabled", False)
tsdf_cfg = config.get("tsdf_refine", {})

# Shutdown parameters (can be overridden in config)
MAX_SHUTDOWN_WAIT_S  = float(tsdf_cfg.get("max_shutdown_wait_s", -1.0))  # <=0 unlimited
MIN_SHUTDOWN_WAIT_S  = float(tsdf_cfg.get("min_shutdown_wait_s", 0.0))
DEFAULT_BLOCK_TIME_S = float(tsdf_cfg.get("default_block_time_s", 12.0))
PROGRESS_STALL_S     = float(tsdf_cfg.get("progress_stall_s", 45.0))
GRACE_EMPTY_S        = float(tsdf_cfg.get("grace_empty_s", 3.0))

UNLIMITED_WAIT = MAX_SHUTDOWN_WAIT_S <= 0

if tsdf_enabled:
    print("[MAIN] Initializing TSDF Refiner...")
    print(f"[MAIN] TSDF config: {tsdf_cfg}")
    try:
        from mast3r_slam.tsdf_refine import TSDFRefiner
        tsdf_refiner = TSDFRefiner(
            cfg=tsdf_cfg,
            shared_keyframes=keyframes,
            quality_service=quality_service,
            device=device,
        )
        tsdf_refiner.start()
        if tsdf_refiner.is_alive():
            print("[MAIN] TSDF refiner thread started successfully")
        else:
            print("[MAIN] WARNING: TSDF refiner thread failed to start")
            tsdf_refiner = None
    except Exception as e:
        print(f"[MAIN] Failed to initialize TSDF refiner: {e}")
        import traceback
        traceback.print_exc()
        tsdf_refiner = None
else:
    print("[MAIN] TSDF refinement disabled in config")
```

**说明**: 
- 初始化TSDF精化器（TSDFRefiner）
- 配置关闭参数，控制程序结束时的等待策略
- TSDF精化器在独立线程中运行，对关键帧窗口进行TSDF重建和点云精化

### 3.4 后端进程启动参数变更 (Lines 290-302)

```python
# NEW: Backend process now includes TSDF global config parameter
backend = mp.Process(
    target=run_backend,
    args=(
        config,
        model,
        states,
        keyframes,
        K,
        config.get("tsdf_global", {}),  # NEW: TSDF global config
    ),
)
backend.start()
```

**说明**: 向后端进程传递TSDF全局配置参数。

---

## 4. 主循环变更

### 4.1 TSDF跟踪变量初始化 (Lines 304-322)

```python
# NEW: Main loop section with TSDF tracking variables
# ------------------------------
# Main loop
# ------------------------------
i = 0
fps_timer = time.time()
frames = []

tsdf_total_scheduled = 0
tsdf_last_success_count = 0
tsdf_shutdown_in_progress = False  # prevent scheduling during shutdown
final_pass_scheduled_once = False  # idempotency guard

def _queue_size_safe(refiner):
    try:
        return refiner.queue.qsize()
    except Exception:
        return None

print("Starting main processing loop...")
```

**说明**: 
- 新增TSDF相关的跟踪变量
- `tsdf_total_scheduled`: 已调度的TSDF块总数
- `tsdf_last_success_count`: 上次成功处理的块数
- `tsdf_shutdown_in_progress`: 关闭标志，防止关闭期间继续调度
- `final_pass_scheduled_once`: 确保最终遍历只调度一次
- `_queue_size_safe`: 安全获取队列大小的辅助函数

### 4.2 单线程模式等待重定位 (Lines 379-384)

```python
# NEW: Single-threaded mode wait for relocalization
while config["single_thread"]:
    with states.lock:
        if states.reloc_sem.value == 0:
            break
    time.sleep(0.01)
```

**说明**: 在单线程模式下，等待重定位完成后再继续。

### 4.3 单线程模式等待后端 (Lines 393-398)

```python
# NEW: Single-threaded mode wait for backend
while config["single_thread"]:
    with states.lock:
        if len(states.global_optimizer_tasks) == 0:
            break
    time.sleep(0.01)
```

**说明**: 在单线程模式下，等待后端优化完成后再继续。

### 4.4 TSDF精化调度逻辑 (Lines 400-420)

```python
# NEW: TSDF refinement scheduling logic
# schedule TSDF refinement only if not shutting down and refiner is healthy
sf = getattr(tsdf_refiner, "stop_flag", None)
if (
    tsdf_refiner is not None
    and tsdf_refiner.is_alive()
    and not tsdf_shutdown_in_progress
    and not (sf is not None and sf.is_set())
):
    current_kf_id = len(keyframes) - 1
    old_queue_size = _queue_size_safe(tsdf_refiner)
    try:
        tsdf_refiner.maybe_schedule_sliding_window(current_kf_id)
    except Exception as e:
        print(f"[TSDF] Scheduling error: {e}")
    new_queue_size = _queue_size_safe(tsdf_refiner)
    if old_queue_size is not None and new_queue_size is not None:
        if new_queue_size > old_queue_size:
            blocks_added = new_queue_size - old_queue_size
            tsdf_total_scheduled += blocks_added
            print(f"[TSDF] Scheduled {blocks_added} blocks for keyframe {current_kf_id}")
```

**说明**: 
- 在添加新关键帧后，调度TSDF精化任务
- 检查精化器状态，确保其正常运行且未在关闭过程中
- 使用滑动窗口策略调度TSDF块
- 跟踪调度的块数量

### 4.5 增强的状态日志 (Lines 422-443)

```python
# NEW: Enhanced status logging with TSDF statistics
if i % 30 == 0:
    FPS = i / (time.time() - fps_timer + 1e-6)
    status_msg = f"[STATUS] Frame {i}, FPS: {FPS:.2f}"

    if tsdf_refiner is not None and tsdf_refiner.is_alive():
        try:
            stats = tsdf_refiner.stats
            success_count = stats.get("successful_blocks", 0)
            total_count = stats.get("total_blocks", 0)
            qsz = _queue_size_safe(tsdf_refiner)
            if success_count > tsdf_last_success_count:
                tsdf_last_success_count = success_count
                print(f"[TSDF] New successes! Total: {success_count}")
            if total_count > 0:
                success_rate = success_count / max(1, total_count)
                qinfo = f", Queue: {qsz}" if qsz is not None else ""
                status_msg += f", TSDF: {success_count}/{total_count} ({success_rate:.1%}){qinfo}"
        except Exception:
            pass

    print(status_msg)
```

**说明**: 
- 每30帧打印一次状态信息
- 包含TSDF精化的统计信息（成功率、队列大小等）
- 当有新的成功精化时打印通知

---

## 5. TSDF关闭流程

### 完整的TSDF Refiner关闭流程 (Lines 449-568)

```python
# NEW: TSDF Refiner shutdown section
# ------------------------------
# TSDF shutdown: schedule final pass drain queue until empty  print stats  stop
# ------------------------------
if tsdf_refiner is not None:
    tsdf_shutdown_in_progress = True  # block any further scheduling

    print("\n" + "=" * 60)
    print("TSDF REFINER SHUTDOWN")
    print("=" * 60)
    try:
        # (1) schedule final pass once
        if len(keyframes) > 0 and not final_pass_scheduled_once:
            final_kf_id = len(keyframes) - 1
            print(f"[TSDF] Triggering final pass for {final_kf_id} keyframes...")
            try:
                tsdf_refiner.schedule_final_pass(final_kf_id)
            except Exception as e:
                print(f"[TSDF] schedule_final_pass error: {e}")
            final_pass_scheduled_once = True

        # (2) drain until queue is empty (time-unlimited if configured)
        start = time.time()
        last_report = start
        last_success = int(getattr(tsdf_refiner, "stats", {}).get("successful_blocks", 0))

        if MIN_SHUTDOWN_WAIT_S > 0:
            time.sleep(MIN_SHUTDOWN_WAIT_S)

        while True:
            qsz = _queue_size_safe(tsdf_refiner)
            stats = getattr(tsdf_refiner, "stats", {}) or {}
            success = int(stats.get("successful_blocks", 0))

            # periodic progress print
            if time.time() - last_report >= 5.0:
                print(f"[TSDF] Draining... queue={qsz}, successes={success}")
                last_report = time.time()

            # if queue size known and empty, give a short grace and break
            if qsz is not None and qsz == 0:
                if success > last_success:
                    last_success = success
                    last_report = time.time()
                time.sleep(GRACE_EMPTY_S)
                qsz2 = _queue_size_safe(tsdf_refiner)
                if qsz2 == 0:
                    print("[TSDF] Queue drained.")
                    break

            # fallback for unknown queue size
            if qsz is None:
                if not UNLIMITED_WAIT and (time.time() - start) >= MAX_SHUTDOWN_WAIT_S:
                    print("[TSDF] Reached shutdown wait budget (fallback path).")
                    break
                time.sleep(0.5)
                if success > last_success:
                    last_success = success
                    last_report = time.time()
                elif (time.time() - last_report) >= PROGRESS_STALL_S:
                    print(f"[TSDF] No progress for {PROGRESS_STALL_S:.1f}s (fallback); assuming drained.")
                    break
            else:
                # known queue size and not empty: keep waiting
                if not UNLIMITED_WAIT and (time.time() - start) >= MAX_SHUTDOWN_WAIT_S:
                    print("[TSDF] Reached shutdown wait budget.")
                    break
                time.sleep(0.5)

        # (3) final statistics
        if hasattr(tsdf_refiner, "stats"):
            stats = tsdf_refiner.stats
            success_count = stats.get("successful_blocks", 0)
            total_count = stats.get("total_blocks", 0)
            print(f"[TSDF] Final Statistics:")
            print(f"  - Total blocks processed: {total_count}")
            print(f"  - Successful refinements: {success_count}")
            if total_count > 0:
                success_rate = success_count / max(1, total_count)
                avg_time = stats.get("total_processing_time", 0.0) / max(1, total_count)
                print(f"  - Success rate: {success_rate:.1%}")
                print(f"  - Average time per block: {avg_time:.3f}s")
            debug_info = stats.get("debug_info", {}) or {}
            print(f"  - TSDF constructions: {debug_info.get('tsdf_constructions', 0)}")
            print(f"  - Surface extractions: {debug_info.get('surface_extractions', 0)}")
            print(f"  - Displacement rejects: {debug_info.get('displacement_rejects', 0)}")
            print(f"  - Hit ratio rejects: {debug_info.get('hit_ratio_rejects', 0)}")

        # (4) stop thread
        print("[TSDF] Stopping refiner thread...")
        try:
            tsdf_refiner.stop_flag.set()
        except Exception:
            pass
        tsdf_refiner.join(timeout=None if UNLIMITED_WAIT else 60.0)
        if tsdf_refiner.is_alive():
            print("[TSDF] WARNING: Thread did not stop cleanly")
        else:
            print("[TSDF] Shutdown completed successfully")

        # (5) summary
        if hasattr(tsdf_refiner, "stats"):
            final_success = tsdf_refiner.stats.get("successful_blocks", 0)
            if final_success > 0:
                print(f"[TSDF] ✓ Successfully refined {final_success} blocks!")
            else:
                print("[TSDF] No successful refinements in this run")

    except Exception as e:
        print(f"[TSDF] Shutdown error: {e}")
        import traceback
        traceback.print_exc()

    print("=" * 60)
```

**说明**: 
完整的TSDF精化器关闭流程，包括：
1. **最终遍历调度**: 调度所有剩余关键帧的精化任务
2. **队列排空**: 等待所有任务完成，支持无限等待或超时
3. **统计输出**: 打印详细的处理统计信息
4. **线程停止**: 正确停止精化器线程
5. **总结**: 输出最终成功数量

---

## 6. 结果保存变更

### 6.1 增强的结果保存 (Lines 570-595)

```python
# NEW: Enhanced save results section with quality PLY export
# ------------------------------
# Save results
# ------------------------------
if dataset.save_results:
    print("\nSaving results...")
    save_dir, seq_name = eval.prepare_savedir(args, dataset)
    eval.save_traj(save_dir, f"{seq_name}.txt", dataset.timestamps, keyframes)
    eval.save_reconstruction(
        save_dir,
        f"{seq_name}.ply",
        keyframes,
        0.0,  # NEW: Changed from last_msg.C_conf_threshold to 0.0
    )
    # NEW: Save PLY with quality attributes
    eval.save_ply_with_quality(
        save_dir,
        f"{seq_name}_quality.ply",
        keyframes,
        0.0,
        quality_service,
    )
    eval.save_keyframes(
        save_dir / "keyframes" / seq_name, dataset.timestamps, keyframes
    )
    print(f"Results saved to {save_dir}")
```

**说明**: 
- 新增 `save_ply_with_quality` 调用，保存带质量属性的点云
- 质量属性包括误差、置信度等指标
- 生成 `{seq_name}_quality.ply` 文件，可用于质量分析和可视化

---

## 7. 进程关闭变更

### 增强的关闭序列 (Lines 607-617)

```python
# NEW: Enhanced shutdown sequence with quality service
print("\nShutting down services...")
quality_service.shutdown()
backend.join()

if not args.no_viz:
    viz.join()

print("\n" + "=" * 60)
print("ALL PROCESSES SHUTDOWN COMPLETE")
print("=" * 60)
```

**说明**: 
- 新增质量服务的关闭调用
- 添加更详细的关闭日志
- 确保所有服务正确关闭

---

## 总结

### 主要新增功能

1. **异步质量评估服务 (AsynchronousQualityService)**
   - 实时评估点云质量
   - 支持多进程异步计算
   - 提供质量属性导出

2. **TSDF全局管理器 (TSDFGlobalManager)**
   - 在后端优化后更新TSDF体积
   - 全局3D重建和融合

3. **TSDF精化器 (TSDFRefiner)**
   - 独立线程运行
   - 滑动窗口策略
   - 点云精化和质量提升
   - 完善的关闭和统计机制

4. **增强的日志和监控**
   - 实时TSDF处理统计
   - 详细的关闭流程日志
   - 质量评估进度跟踪

5. **单线程模式支持**
   - 确保在单线程模式下的正确同步
   - 等待后端和重定位完成

### 配置参数

新增的配置项：
- `tsdf_global`: 全局TSDF配置
- `tsdf_refine`: TSDF精化器配置
  - `enabled`: 是否启用
  - `max_shutdown_wait_s`: 最大关闭等待时间
  - `min_shutdown_wait_s`: 最小关闭等待时间
  - `progress_stall_s`: 进度停滞超时
  - `grace_empty_s`: 队列空后的宽限时间

### 文件输出

新增输出文件：
- `{seq_name}_quality.ply`: 带质量属性的点云文件

---

**文档版本**: 1.0  
**更新日期**: 2025-11-19  
**作者**: MASt3R-SLAM Team
