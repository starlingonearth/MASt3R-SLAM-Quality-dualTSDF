# 局部 TSDF 精化执行摘要

## 🎯 目标
- **保持实时性**：主线程仍以 MASt3R pointmap + ray-based tracking 为核心，不引入重型全局 TSDF。
- **质量驱动的局部修补**：仅在质量评分低的局部区域构建临时 TSDF，对关键帧的点云与置信度进行保守更新，以提升后端优化的稳健性。
- **异步执行**：所有 TSDF 相关计算都在独立线程/进程完成，对主轨迹求解没有阻塞。

## 🧱 架构要点
1. **Tracker** 负责姿态解算并为每个关键帧提交像素残差、置信度等质量数据。
2. **AsynchronousQualityService**（多进程）批量计算 patch 级优先级热图。
3. **TSDFRefiner 线程**：
   - 按滑动窗口与质量结果挑选 `PatchBlock`，进入局部任务队列。
   - 为每个 block 构建临时 TSDF（KinectFusion 融合），通过 ray casting 检查命中率和位移。 
   - 命中通过后，对对应像素施加置信度提升（以及可选的几何混合），结果写回 `SharedKeyframes`。
4. **FactorGraph / Eval** 不直接使用 TSDF 残差，但受益于更稳健的 pointmap 与置信度。

## ⚙️ 配置入口（`config/base.yaml::tsdf_refine`）
- `enabled` / `window_size`：线程总开关与调度窗口。
- `voxel_size`, `trunc_dist`, `max_grid_dim`, `roi_size`：局部 TSDF 体素参数。
- `ray_samples`, `max_displacement`, `min_hit_rate`：射线回投与验收阈值。
- `confidence_boost`, `confidence_max`, `geometric_weight`：写回策略。
- `max_shutdown_wait_s` 等：退出阶段的 drain 策略。

## ✅ 已实现收益
- **对主线程零侵入**：主循环仅负责排队与收尾，不受 TSDF 计算影响。
- **可观测统计**：TSDFRefiner 输出成功率、平均耗时、拒绝原因，方便调参和回归。
- **局部鲁棒性增强**：劣质 patch 的置信度会被抑制或修正，间接改善回环与全局优化的可用观测。

## ⚠️ 已知限制
- **无全局 TSDF/Mesh**：当前仅在关键帧相机坐标内构建临时体素，不输出网格。
- **姿态未直接优化**：TSDF 结果暂未反馈到 tracker / factor graph；若需要 TSDF ICP，需要新增优化链路。
- **ROI 受限**：ROI 依赖质量热图与深度统计，面对大尺度或跨帧一致性问题时作用有限。

本摘要与现有代码完全一致，可作为后续扩展（例如升级为全局 TSDF 或加入 TSDF 残差）的基准说明。
