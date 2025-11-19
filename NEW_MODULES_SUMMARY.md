# 新增功能模块说明 (New Modules Summary)

本文档详细说明 MASt3R-SLAM 项目中新增的所有功能模块。

## 模块概览

| 模块 | 文件 | 行数 | 功能 |
|------|------|------|------|
| 质量评估服务 | quality_async.py | 303 | 异步质量计算和管理 |
| 质量核心算法 | quality_core.py | 140 | 质量指标计算 |
| TSDF 精化器 | tsdf_refine.py | 1064 | 滑动窗口 TSDF 精化 |
| TSDF 全局管理 | tsdf/global_manager.py | 230 | 全局 TSDF 融合协调 |
| TSDF 体积 | tsdf/global_volume.py | 141 | 稀疏 TSDF 体积 |
| TSDF 优化器 | tsdf/tsdf_optimizer.py | 125 | 基于 TSDF 的位姿优化 |

---

## 1. 质量评估系统

### 1.1 quality_async.py - 异步质量服务

**核心类**: `AsynchronousQualityService`

**主要功能**:
- 多线程异步质量计算
- 双索引缓存 (kf_id 和 frame_id)
- EWMA 状态持久化
- 回调通知机制
- 全局统计管理

**关键特性**:
```python
# 使用 threading 而非 multiprocessing，支持 GPU
self.device = "cuda" if torch.cuda.is_available() else "cpu"

# 双索引缓存
self.cache_by_kf_id = {}
self.cache_by_frame_id = {}

# EWMA 状态持久化
self.ewma_state = {}  # kf_id -> ewma tensor
```

**配置参数**:
- `patch_size`: 16 (补丁大小)
- `batch_size`: 4 (批处理大小)
- `alpha`: 0.8 (EWMA 平滑系数)
- `max_wait_ms`: 20 (批次收集最大等待时间)

### 1.2 quality_core.py - 质量核心算法

**主要函数**:
1. `reduce_grid`: 网格降采样
2. `u_from_CQ`: 从置信度和质量计算不确定性
3. `classify`: 质量分类 (0-3 级)
4. `compute_batch`: 批量质量计算

**质量指标**:
- `delta_cov`: 协方差变化
- `r`: 重投影误差
- `u`: 不确定性
- `class_id`: 质量分类 (0=优秀, 1=良好, 2=中等, 3=差)
- `priority`: 精化优先级

---

## 2. TSDF 精化系统

### 2.1 tsdf_refine.py - TSDF 精化器

**核心类**: `TSDFRefiner(threading.Thread)`

**主要功能**:
- 滑动窗口策略
- 基于质量的块选择
- TSDF 构建和表面提取
- 点云精化和更新

**工作流程**:
```
1. 调度: maybe_schedule_sliding_window()
   ↓
2. 块选择: _select_blocks_enhanced()
   ↓
3. 块聚类: _cluster_patches_enhanced()
   ↓
4. 精化: _refine_block_enhanced()
   ↓
5. TSDF 构建: _build_tsdf_robust()
   ↓
6. 表面提取: _extract_surface_safe()
   ↓
7. 更新关键帧
```

**关键参数**:
- `window_size`: 3-5 (滑动窗口大小)
- `max_rois_per_kf`: 3 (每个关键帧最大 ROI 数)
- `voxel_size`: 0.01-0.02 (体素大小)
- `max_displacement`: 0.015 (最大位移阈值)

### 2.2 RefineRegistry - 去重和历史管理

**状态管理**:
- IDLE: 空闲
- QUEUED: 已排队
- RUNNING: 运行中
- COOLDOWN: 冷却期

**功能**:
- 原子性去重
- 冷却期管理
- 成功率统计

---

## 3. TSDF 全局系统

### 3.1 tsdf/global_manager.py - 全局管理器

**三个核心类**:

#### TSDFGlobalIntegrator
- 后台线程融合关键帧到全局 TSDF
- 处理位姿更新后的重新融合
- 采样和置信度过滤

#### TSDFGlobalOptThread
- 异步位姿优化线程
- 使用低优先级 CUDA 流
- 批量处理优化任务

#### TSDFGlobalManager
- 高层协调器
- 管理 integrator 和 optimizer
- 响应后端优化事件

**数据流**:
```
FactorGraph.solve()
   ↓
on_after_backend_solve()
   ↓
mark_pose_update() → 重新融合队列
   ↓
enqueue() → 优化队列
```

### 3.2 tsdf/global_volume.py - TSDF 体积

**核心类**: `TSDFVolume`

**实现方式**:
- 稀疏体素哈希 (Python dict)
- 线程安全 (RLock)
- 射线积分融合

**主要方法**:
- `integrate()`: 融合点云
- `query()`: 查询 TSDF 值和梯度
- `_estimate_gradient()`: 估计梯度

**参数**:
- `voxel_size`: 0.03 (体素大小)
- `truncation`: 0.12 (截断距离)
- `max_weight`: 100.0 (最大权重)

### 3.3 tsdf/tsdf_optimizer.py - TSDF 优化器

**核心类**: `TSDFPoseOptimizer`

**功能**:
- 基于 TSDF 残差的位姿优化
- Gauss-Newton 迭代
- Sim3 位姿参数化

**优化流程**:
```
1. 采样有效点
2. 变换到世界坐标
3. 查询 TSDF 值和梯度
4. 构建线性系统 (H, b)
5. 求解增量 delta
6. 更新位姿
```

---

## 4. 模块间交互

### 4.1 质量服务 ↔ 跟踪器
```python
# tracker.py 提交质量任务
tracker.quality_service.submit(job)

# quality_async.py 异步计算
worker_loop() → compute_batch() → 返回结果
```

### 4.2 质量服务 ↔ TSDF 精化器
```python
# tsdf_refine.py 获取质量结果
quality_result = quality_service.get(frame_id)

# 使用 priority 选择块
blocks = _select_blocks_enhanced(kf_id, quality_result)
```

### 4.3 TSDF 全局 ↔ 后端优化
```python
# main.py 后端优化后回调
tsdf_manager.on_after_backend_solve(factor_graph)

# global_manager.py 处理
indices = factor_graph.last_unique_kf_idx
integrator.mark_pose_update(indices)
opt_worker.enqueue(indices)
```

---

## 5. 配置参数汇总

### 5.1 质量评估配置
```yaml
quality:
  patch_size: 16
  batch_size: 4
  max_wait_ms: 20
  metrics:
    coverage:
      alpha_ema: 0.8
      b0: 0.15
      theta0_deg: 10.0
  thresholds:
    z_r: 1.0
    z_u: 1.0
    d_cov: 0.02
```

### 5.2 TSDF 精化配置
```yaml
tsdf_refine:
  enabled: true
  window_size: 5
  voxel_size: 0.01
  trunc_dist: 0.04
  max_rois_per_kf: 3
  max_displacement: 0.015
  min_hit_rate: 0.05
  cooldown_frames: 35
```

### 5.3 TSDF 全局配置
```yaml
tsdf_global:
  enabled: true
  voxel_size: 0.03
  trunc_dist: 0.12
  max_weight: 100.0
  samples_per_kf: 2000
  max_iterations: 3
  async_optimize: true
```

---

## 6. 性能特征

### 6.1 内存使用
- 质量服务: ~50 MB
- TSDF 精化器: ~200 MB
- TSDF 全局: ~100 MB
- **总计**: ~350 MB

### 6.2 计算时间
- 质量计算: ~10 ms/帧 (异步)
- TSDF 精化: ~50 ms/块 (后台)
- TSDF 全局融合: ~20 ms/关键帧 (后台)

### 6.3 线程使用
- 质量服务: 1 个 worker 线程
- TSDF 精化器: 1 个 refinement 线程
- TSDF 全局: 2 个线程 (integrator + optimizer)
- **总计**: 4 个后台线程

---

## 7. 关键设计决策

### 7.1 为什么使用 threading 而非 multiprocessing?
- 支持 GPU 加速
- 共享内存访问更高效
- 避免序列化开销

### 7.2 为什么使用稀疏 TSDF?
- 内存效率高
- 适合大场景
- 动态扩展

### 7.3 为什么使用滑动窗口?
- 限制计算量
- 关注最新数据
- 避免重复精化

---

## 8. 故障排除

### 8.1 质量服务未启动
**症状**: 无质量数据  
**解决**: 检查 `quality_service` 初始化

### 8.2 TSDF 精化器无输出
**症状**: 无精化块  
**解决**: 检查质量阈值和配置

### 8.3 内存不足
**症状**: CUDA OOM  
**解决**: 减小 `voxel_size`, `max_rois_per_kf`

---

**文档版本**: 1.0  
**更新日期**: 2025-11-19  
**作者**: MASt3R-SLAM Team
