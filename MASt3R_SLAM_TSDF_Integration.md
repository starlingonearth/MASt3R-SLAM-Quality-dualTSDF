# MASt3R-SLAM 局部 TSDF 精化方案

本文件描述当前 MASt3R-SLAM-dualTSDF 分支中已经实现的 **质量驱动局部 TSDF 精化** 架构，目标是让方案描述与实际代码保持一致，方便后续演进。

---

## 1. 背景与设计目标

- **实时密集跟踪保持不变**：前端依旧使用 MASt3R 生成 pointmap，并依靠 ray-based 误差完成姿态估计与关键帧管理。
- **全局体素未落地**：当前实现没有维护世界坐标的全局 TSDF 体素，也不会输出 marching cubes 网格。因此文档聚焦于「局部自适应修补」能力，而不是全局建图。
- **新增需求**：在不拖慢主线程的前提下，根据质量评估结果挑选局部区域，构建小尺度 TSDF，对关键帧的点云与置信度做保守修补，以提升后端优化的鲁棒性。

---

## 2. 系统架构

```
┌───────────────┐      ┌────────────────────────────┐
│  Tracker      │      │  AsynchronousQualityService │
│  (ray-based)  │      │  (multiprocess CPU)         │
└─────┬─────────┘      └──────────┬──────────────────┘
      │ keyframe + matches                   │ patch scores / masks
      ▼                                      ▼
┌───────────────┐      sliding window    ┌────────────────────┐
│ SharedKeyframes│◀──────────────────────│  TSDFRefiner Thread │
└─────┬─────────┘   enqueue blocks       └──────────┬─────────┘
      │ pose / pointmap updates                      │
      ▼                                              ▼
┌────────────────────────────┐       ┌───────────────────────────────┐
│ FactorGraph + eval scripts │       │ Updated per-kf confidences & │
│ (仍为 ray/objectives)      │       │ pointmap (局部)               │
└────────────────────────────┘       └───────────────────────────────┘
```

- **主线程 (`main.py`)**：生成关键帧后，除了加入全局优化队列外，还调用 `tsdf_refiner.maybe_schedule_sliding_window`（`main.py:240-281`），按窗口大小把关键帧 ID 推给 TSDF 精化线程。
- **质量服务 (`mast3r_slam/quality_async.py`)**：Tracker 在每次位姿求解后提交像素残差、置信度等信息（`mast3r_slam/tracker.py:94-154`），独立进程批量计算 patch 级指标，结果缓存供 TSDFRefiner 查询。
- **TSDFRefiner (`mast3r_slam/tsdf_refine.py`)**：后台线程，负责任务调度、ROI 选择、局部 TSDF 构建、射线回投与置信度更新。其生命周期由配置文件 `config/base.yaml:65-119` 控制，在程序退出时执行排空与统计（`main.py:318-420`）。

---

## 3. 关键帧生命周期

1. **关键帧判定**：Tracker 判定 `new_kf=True` 时写入 `SharedKeyframes` 并触发全局优化任务。
2. **质量计算**：Tracker 同步把残差批次提交到 `quality_service`。服务端返回 patch 大小为 `quality.patch_size` 的优先级热图。
3. **TSDF 调度**：
   - `TSDFRefiner` 维护 window + retry 队列（`maybe_schedule_sliding_window`）。
   - 只有拿到质量结果的关键帧才会切分 patch，生成若干 `PatchBlock`，进入本地队列。
4. **局部 TSDF 精化**：
   - 对每个 block，提取掩码内像素的 canonical pointmap 与置信度，基于深度统计裁剪 ROI。
   - 构建 **临时 TSDF 体素**（通常 ≤64³），采用 KinectFusion 的截断距离更新。
   - 通过局部射线投射寻找零交叉；若命中率和位移阈值满足条件，则对对应像素的 `X_canon` 与 `C` 做保守更新。
5. **结果写回**：更新操作在 `SharedKeyframes` 上持有锁完成，确保 tracker/back-end 读取到最新置信度。

此流程强调：**TSDF 结果只作用于关键帧自身，不会生成全球地图或直接改变姿态**。

---

## 4. TSDFRefiner 内部结构

### 4.1 调度与去重
- `RefineRegistry`（`mast3r_slam/tsdf_refine.py:64-137`）对同一 `(kf_id, block_id)` 进行状态管理，避免重复排队。
- 支持滑动窗口、重试冷却与最终排空，使线程可以长期运行且在退出时清空队列。

### 4.2 ROI 选择
- 在质量热图中选取 Top-K patch（默认 3 个 ROI），并根据置信度、深度方差过滤异常 patch。
- 通过 `_cluster_patches_enhanced` 将相邻 patch 合并成 block，并生成像素掩码与统计量。

### 4.3 局部 TSDF 构建
- 使用 canonical points（关键帧相机坐标）推导 ROI 边界，按 `voxel_size / trunc_dist / max_grid_dim` 生成稠密网格。
- 对每个有效点沿其射线在 ROI 内采样，执行截断 TSDF 加权平均，得到体素值与权重。

### 4.4 射线回投与判据
- `_extract_surface_safe` 对 block 掩码内像素进行射线步进，寻找 TSDF 零交叉。
- 统计命中率与位移，若 `hit_ratio >= min_hit_rate` 且最大位移不超阈值，则认为这一 block 的 TSDF 可信。

### 4.5 保守更新策略
- 仅对命中像素：
  - 增加置信度（`confidence_boost`，并在 `confidence_max` 内截断）。
  - 可选地做几何混合（`geometric_weight`），目前默认 0，保持点坐标不被大幅修改。
- 写回操作受全局锁保护，确保与 tracker 互斥。

---

## 5. 配置接口（`config/base.yaml:65-119`）

| 字段 | 作用 |
|------|------|
| `enabled` | 是否启动 TSDFRefiner。
| `window_size` | 调度窗口，控制关键帧延迟。
| `quality_wait_ms` | 等待质量结果的超时时间。
| `voxel_size / trunc_dist / max_grid_dim / roi_size` | 局部 TSDF 体素参数。
| `ray_samples / max_displacement / min_hit_rate` | 射线采样与验收阈值。
| `confidence_boost / confidence_max` | 写回置信度的上限与增量。
| `max_rois_per_kf / min_confidence` | ROI 选择约束。
| `max_shutdown_wait_s ...` | 程序结束时 drain 队列的策略。

---

## 6. 与全局优化/评估的关系

- Factor graph 仍然只最小化 ray error（`mast3r_slam/global_opt.py:12-200`），TSDFRefiner 的输出通过更稳定的 pointmap/confidence 间接影响优化。
- 评估脚本 (`mast3r_slam/evaluate.py` 等) 仍使用 pointmap 导出 PLY；目前没有 TSDF mesh 导出路径。

---

## 7. 现有限制与后续方向

1. **缺乏世界级 TSDF**：如果未来需要 marching cubes 或 TSDF 残差约束，需要新建全局体素结构并在 `SharedKeyframes` → world 空间完成坐标变换。
2. **姿态未直接受益**：目前 TSDF 仅修改点/置信度，可考虑在 `FactorGraph` 或 tracker 中加入 TSDF ICP 步骤。
3. **ROI 坐标系假设**：ROI 计算仍在关键帧相机坐标内，适合局部小范围修补。若要处理大规模场景，需要在世界坐标构建更稳健的 ROI。
4. **质量指标依赖**：调度完全依赖 `quality_service`，需要确保该服务稳定且延迟可控。

该方案准确反映了现有代码的行为，是后续演进（例如升级为真正的全局 TSDF）可以参考的基线。

---

## 8. 面向后端 FactorGraph 的全局 TSDF 设计（新方案）

为让 TSDF 直接优化关键帧位姿，同时尽量不干扰 MASt3R 前端跟踪，可在现有架构上新增以下模块。该设计分阶段实现，优先在 **FactorGraph** 引入 TSDF 残差，前端只承担数据采集任务。

### 8.1 目标与原则
- **最大化位姿约束**：TSDF point-to-surface 残差直接写入后端 Hessian，与 ray-based factor 共同求解。
- **前端最小侵入**：Tracker 仍只依赖 ray error；TSDF 融合、采样、残差计算全部在后台线程/子服务完成。
- **可扩展 volume**：维护一个世界坐标的稀疏 TSDF（voxel hashing / block pool），支持长序列增量更新与 mesh 导出。
- **与局部 refiner 共存**：允许保留质量驱动的局部修补，用于改善观测质量；全局 volume 则提供跨帧一致性与姿态约束。

### 8.2 数据流概览

```
Tracker (前端) ──关键帧 X,C,T──▶ SharedKeyframes
                                │
                                ▼
                ┌────────────────────────────┐
                │ Global TSDF Service        │
                │  • 世界坐标体素结构        │
                │  • 融合/采样接口           │
                └──────────┬─────────────────┘
                           │ TSDF samples + grads
                           ▼
                ┌────────────────────────────┐
                │ FactorGraph (后端)         │
                │  • Ray factors (现有)      │
                │  • TSDF factors (新增)     │
                └────────────────────────────┘
```

### 8.3 全局 TSDF Volume 管理
1. **表示**：
   - 使用 voxel hashing 或固定尺寸 block pool（例如 16³ block）管理稀疏体素，键为世界坐标下的 block 索引。
   - 每个体素存储 `tsdf`, `weight`, 可选 `color`, `avg_ray`（沿用现有局部设计）。

2. **融合**：
   - 每个新关键帧写入 `SharedKeyframes` 后，由后台 `TSDFIntegrator` 线程读取其 pointmap 与 `T_WC`，将点转换到世界坐标并执行 ray carving 融合。
   - 可沿用质量服务结果决定是否跳过低质量关键帧。融合顺序与 FactorGraph 窗口同步，以保证 TSDF 反映最新 pose（或在 pose 更新后触发增量修正）。

3. **查询接口**：
   - `query_tsdf(point_world)` 返回 TSDF 值与梯度（通过三线性插值 + 中心差分）。
   - `sample_tsdf(points_world)` 批量接口供 FactorGraph 使用，返回 `tsdf`, `grad`, `voxel_weight`。

### 8.4 FactorGraph 中的 TSDF 残差
1. **采样策略**：
   - 每个关键帧在加入 FactorGraph 窗口时，从其高置信度点中随机/基于质量热图采样 `N_tsdf` 个点。
   - 过滤掉不在 TSDF 体素范围内或 TSDF 值绝对值 > trunc 的点。

2. **残差定义**：
   - 对关键帧 `k` 的采样点 `X_k`，残差为 `r = TSDF(T_WC_k · X_k_canon)`；理想情况下为 0。
   - 权重 `w = λ_tsdf * C_k * voxel_weight`，`λ_tsdf` 为全局系数，可通过配置调整与 ray factor 的相对影响。

3. **Jacobian**：
   - 令 `p = T_WC_k · X_k`，TSDF 梯度 `g = ∇TSDF(p)`。
   - SE(3) 雅可比：`∂p/∂ξ = [I, -[Rp]×]`，因此 `J = g^T · [I, -[Rp]×]`（Sim3 情况需增加尺度项）。
   - 在 `gauss_newton_rays` 的求解过程中，将 TSDF 残差、雅可比、权重打包成与 ray factor 同规格的稀疏块，累加到 Hessian 与梯度向量。

4. **求解流程**：
   - FactorGraph 扩展出 `solve_GN_rays_tsdf`：
     1. 计算原有 ray factor；
     2. 获取所有活跃关键帧的 TSDF 样本与雅可比，累加 `E_tsdf`；
     3. 解线性系统并回写姿态；
     4. 完成后，把 pose 更新通知 TSDF volume（以便必要时做 pose 修正或重新积分）。

### 8.5 调度与一致性
- **TSDF 融合线程** 与 **FactorGraph** 在不同线程运行，通过轻量队列或版本号协调：
  - 关键帧 pose 更新后，设置 “dirty” 标记，提示 TSDF 线程重新积分或执行 pose-based warp。
  - 在 FactorGraph 每轮迭代开始前 snapshot 当前 TSDF volume 的版本，确保残差计算期间体素不被写入（可用读写锁或 Copy-on-write block 缓存）。

- **局部 Refiner** 可继续运行，但写回的 pointmap 需要在下一次 TSDF 融合前冻结，以免 pose 更新导致数据不一致。可以通过 `SharedKeyframes.is_dirty` 标记协同：当局部 refiner 更新点云后，通知 TSDF 线程优先重新积分该关键帧。

### 8.6 前端影响最小化策略
- Tracker 仍只执行 MASt3R ray optimization；TSDF 融合/采样完全在后台异步执行，主线程只负责：
  1. 将关键帧加入融合队列；
  2. 在 FactorGraph 完成一次 GN 后，把新的姿态广播给 TSDF 线程用于 pose 修正。
- 若需要额外的 TSDF ICP，只在关键帧加入 FactorGraph 前后台异步运行一次，结果作为初始姿态种子，而不是阻塞 tracker。

### 8.7 实施步骤建议
1. **模块化 TSDF Volume**：新建 `mast3r_slam/tsdf/volume.py`，实现稀疏体素管理、融合与查询。
2. **后台融合线程**：复用现有 TSDFRefiner 的调度框架，新增 `TSDFIntegrator`，专门负责把关键帧写入全局 volume，并维护 version/dirty 状态。
3. **FactorGraph 扩展**：在 `global_opt.py` 中创建 TSDF factor 管线，支持配置 `λ_tsdf`、采样数、截断阈值，保证与现有 CUDA 后端接口兼容（必要时在 `mast3r_slam_backends` 增加 TSDF kernel）。
4. **同步机制**：为 `SharedKeyframes` 添加 pose/version 字段，确保 TSDF volume 与 FactorGraph 使用的姿态一致；必要时采用 double-buffer 来隔离写入。
5. **配置与监控**：在 `config/base.yaml` 添加 `tsdf_global` 段落，包含体素参数、λ、采样数、版本同步策略，并在日志中输出 TSDF factor 的残差 RMS、命中率等指标。

该设计使 TSDF 约束集中在后端因子图，既充分利用 TSDF 的全局一致性，又避免对 MASt3R 前端造成额外延迟，是引入“真正的全局 TSDF + 位姿优化”时的推荐路线。

---

## 9. 含相机矫正管线的 TSDF 细化与位姿优化方案

MASt3R-SLAM 提供带内参/畸变矫正的运行模式（`use_calib=True`）。与无标定版本相比，所有射线、投影与深度都可在真实相机模型下精确计算，这为 TSDF 融合与 point-to-surface 残差提供了更稳定的几何基础。本节给出“**前端保持原流程，后端 + TSDF**”的设计细节，专门针对有矫正参数的管线。

### 9.1 输入与预处理
1. **相机模型**：
   - 在数据加载阶段读取 `intrinsics.yaml`，获得 `K`（3×3）与畸变参数，重投影前对图像做矫正或记录 `dist_coeffs` 供 TSDF 采样使用。
   - Tracker 中调用 `constrain_points_to_ray(..., K)`，确保 `X_canon` 落在真实射线平面上。

2. **关键帧数据包**：每个关键帧存储
   - `T_WC`（Sim3，但在有标定时通常退化为 SE3+scale）
   - `X_canon`（矫正后的 3D 点）
   - `C`（置信度）
   - `K`、`img_shape`
   这些信息将被 TSDF 融合与残差模块复用。

### 9.2 TSDF 融合（有内参）
1. **射线生成**：通过 `K^-1 [u,v,1]^T` 直接得到单位射线，避免无标定版中的归一化估算误差。
2. **体素更新**：与第 8 节类似，但在计算有符号距离时使用精确的相机中心 `C = T_WC[:3,3]` 与射线方向 `R_WC @ ray_cam`：
   ```python
   ray_world = R_WC @ normalize(K^-1 [u,v,1]^T)
   sdf = dot(voxel_center - C, ray_world) - depth
   ```
   截断、权重与写入规则保持一致。
3. **畸变补偿**：若图像未经畸变校正，可在射线生成前调用反畸变函数；TSDF 模块应暴露 `undistort_pixel(u,v, dist_coeffs)` 接口。

### 9.3 TSDF → 位姿反馈路径

#### 9.3.1 关键帧插入前的局部 ICP（可选）
1. 在关键帧确定后、进入 FactorGraph 前，后台触发一次 TSDF ICP：
   - 使用矫正射线将 `X_canon` 投到世界 TSDF。
   - 采用 point-to-surface 残差 `TSDF(T_WC · X)` 与解析雅可比 `J = ∇TSDF · ∂(T_WC · X)/∂ξ`。
   - 迭代次数保持 5~10 次，仅输出 `ΔT` 作为关键帧入图的初始姿态；若失败则回退到 tracker 结果。
2. 该步骤在独立线程执行，不会阻塞 MASt3R 前端。

#### 9.3.2 FactorGraph 内的 TSDF 因子（主路径）
1. **采样点选择**：
   - 依据置信度 `C`、质量热图或图像梯度从关键帧均匀采样 `N_tsdf_calib` 个点。
   - 使用 `K` 将像素坐标转换到相机系，再乘以 `T_WC` 得到世界坐标。
2. **残差与雅可比**：
   - 残差：`r_i = TSDF(T_WC_k · X_i)`
   - 雅可比：`J_i = ∇TSDF(p_i) · [I, -[R·x]×, -p_i]`（Sim3 情况，最后一列对应尺度）；其中 `p_i = T_WC_k · X_i`。
3. **权重**：
   - `w_i = λ_tsdf * C_i * voxel_weight * f(angle)`，其中 `f(angle)` 用于抑制射线与表面呈切线关系时的数值不稳定。
4. **求解**：在 `solve_GN_calib` 路径中新增 TSDF 因子：
   - 构建扩展 Hessian `H = [H_ray + λ_tsdf H_tsdf]`。
   - 通过 GPU/CPU 后端统一解线性系统。
   - 更新姿态后通知 TSDF volume，必要时重新积分受影响的关键帧（pose-damped integration）。

### 9.4 线程与资源调度
1. **TSDF Volume Service**：维持读写锁，FactorGraph 查询使用读锁，融合使用写锁；可选双缓冲以减少锁冲突。
2. **Calib-aware TSDF Integrator**：继承现有 `TSDFRefiner` 的调度机制，但任务内容改为“插入关键帧到全局 volume +（可选）局部 TSDF 修补”。
3. **FactorGraph 扩展**：新增配置：
   ```yaml
   tsdf_global:
     enabled: true
     lambda: 0.1
     samples_per_kf: 2000
     voxel_size: 0.02
     trunc_distance: 0.08
     use_calib_projection: true
   ```

### 9.5 推进步骤
1. **阶段 1**：实现 Calib-aware 全局 TSDF volume + 融合线程，验证 mesh/深度渲染正确性。
2. **阶段 2**：实现可选的局部 TSDF ICP（插入前），确保关键帧姿态在入图前已与 TSDF 对齐。
3. **阶段 3**：在 `FactorGraph.solve_GN_calib` 中加入 TSDF 因子，调优 `λ_tsdf`、采样数与残差过滤规则。
4. **阶段 4**：整合监控与回退机制（例如 TSDF 残差过大时自动降低权重），并在评估脚本中输出 TSDF 相关指标。

通过上述步骤，可在“使用相机矫正参数”的管线上实现 TSDF 融合与位姿优化：前端保持原有的 MASt3R ray-based 跟踪，后端 FactorGraph 则因 TSDF 残差获得更强的几何一致性，从而在保持实时性的同时显著提高姿态与建图精度。
