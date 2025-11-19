# MASt3R-SLAM 配置文件详解 (Configuration Guide)

本文档详细说明 `config/base.yaml` 中的所有配置参数。

## 目录
1. [全局配置](#1-全局配置)
2. [数据集配置](#2-数据集配置)
3. [匹配配置](#3-匹配配置)
4. [跟踪配置](#4-跟踪配置)
5. [局部优化配置](#5-局部优化配置)
6. [检索配置](#6-检索配置)
7. [重定位配置](#7-重定位配置)
8. [质量评估配置](#8-质量评估配置)
9. [TSDF精化配置](#9-tsdf精化配置)
10. [TSDF全局配置](#10-tsdf全局配置)

---

## 1. 全局配置

```yaml
use_calib: False
single_thread: False
```

### use_calib
- **类型**: Boolean
- **默认值**: False
- **说明**: 是否使用相机标定
  - `True`: 使用标定模式，需要相机内参和畸变参数
  - `False`: 非标定模式，使用射线距离表示
- **影响**: 
  - 跟踪器使用不同的残差计算方式
  - 全局优化使用不同的投影模型

### single_thread
- **类型**: Boolean
- **默认值**: False
- **说明**: 是否使用单线程模式
  - `True`: 串行执行，便于调试
  - `False`: 多线程并行，正常运行模式
- **用途**: 调试和性能分析

---

## 2. 数据集配置

```yaml
dataset:
  subsample: 1
  img_downsample: 1
  center_principle_point: True
```

### subsample
- **类型**: Integer
- **默认值**: 1
- **范围**: ≥ 1
- **说明**: 帧采样间隔
  - `1`: 使用所有帧
  - `2`: 每隔一帧
  - `N`: 每隔 N-1 帧
- **用途**: 降低计算量，加快处理速度

### img_downsample
- **类型**: Integer
- **默认值**: 1
- **范围**: ≥ 1
- **说明**: 图像下采样因子
  - `1`: 原始分辨率
  - `2`: 分辨率减半
- **影响**: 内存使用和计算速度

### center_principle_point
- **类型**: Boolean
- **默认值**: True
- **说明**: 去畸变后是否将主点居中
- **用途**: 标定模式下的图像矫正

---

## 3. 匹配配置

```yaml
matching:
  max_iter: 10
  lambda_init: 1e-8
  convergence_thresh: 1e-6
  dist_thresh: 1e-1
  radius: 3
  dilation_max: 5
```

### max_iter
- **类型**: Integer
- **默认值**: 10
- **说明**: 匹配优化的最大迭代次数

### lambda_init
- **类型**: Float
- **默认值**: 1e-8
- **说明**: Levenberg-Marquardt 初始阻尼系数

### convergence_thresh
- **类型**: Float
- **默认值**: 1e-6
- **说明**: 收敛阈值

### dist_thresh
- **类型**: Float
- **默认值**: 0.1 米
- **说明**: 3D 空间距离阈值

### radius
- **类型**: Integer
- **默认值**: 3
- **说明**: 搜索半径（像素）

### dilation_max
- **类型**: Integer
- **默认值**: 5
- **说明**: 最大膨胀半径
- **行为**: 从 max 开始递减到 1

---

## 4. 跟踪配置

```yaml
tracking:
  min_match_frac: 0.05
  max_iters: 50
  C_conf: 0.0
  Q_conf: 1.5
  rel_error: 1e-3
  delta_norm: 1e-3
  huber: 1.345
  match_frac_thresh: 0.333
  sigma_ray: 0.003
  sigma_dist: 1e+1
  sigma_pixel: 1.0
  sigma_depth: 1e+1
  sigma_point: 0.05
  pixel_border: -10
  depth_eps: 1e-6
  filtering_mode: weighted_pointmap
  filtering_score: median
```

### min_match_frac
- **类型**: Float
- **默认值**: 0.05
- **范围**: [0, 1]
- **说明**: 最小匹配比例阈值
- **用途**: 判断跟踪是否成功

### max_iters
- **类型**: Integer
- **默认值**: 50
- **说明**: 位姿优化最大迭代次数

### C_conf / Q_conf
- **类型**: Float
- **默认值**: C_conf=0.0, Q_conf=1.5
- **说明**: 置信度和质量阈值
  - C_conf: MASt3R 置信度阈值
  - Q_conf: 匹配质量阈值

### rel_error / delta_norm
- **类型**: Float
- **默认值**: 1e-3
- **说明**: 收敛判断标准
  - rel_error: 相对误差
  - delta_norm: 增量范数

### huber
- **类型**: Float
- **默认值**: 1.345
- **说明**: Huber 损失函数参数
- **用途**: 鲁棒估计，降低外点影响

### match_frac_thresh
- **类型**: Float
- **默认值**: 0.333
- **说明**: 关键帧选择的匹配比例阈值

### sigma_ray / sigma_dist
- **类型**: Float
- **默认值**: sigma_ray=0.003, sigma_dist=10.0
- **说明**: 非标定模式的噪声参数
  - sigma_ray: 射线方向噪声
  - sigma_dist: 距离噪声

### sigma_pixel / sigma_depth
- **类型**: Float
- **默认值**: sigma_pixel=1.0, sigma_depth=10.0
- **说明**: 标定模式的噪声参数
  - sigma_pixel: 像素噪声（像素）
  - sigma_depth: 对数深度噪声

### sigma_point
- **类型**: Float
- **默认值**: 0.05
- **说明**: 点位置噪声（米）

### pixel_border
- **类型**: Integer
- **默认值**: -10
- **说明**: 像素边界（标定模式）
  - 负值: 允许像素超出图像边界的距离

### depth_eps
- **类型**: Float
- **默认值**: 1e-6
- **说明**: 深度最小值（标定模式）

### filtering_mode
- **类型**: String
- **默认值**: weighted_pointmap
- **选项**:
  - `recent`: 使用最新点云
  - `first`: 使用首次观测
  - `best_score`: 使用最高分数
  - `weighted_pointmap`: 加权点云融合
  - `weighted_spherical`: 球坐标加权融合
  - `indep_conf`: 独立置信度更新
- **说明**: 点云更新策略

### filtering_score
- **类型**: String
- **默认值**: median
- **选项**: `median`, `mean`
- **说明**: 分数计算方式（仅用于 best_score 模式）

---

## 5. 局部优化配置

```yaml
local_opt:
  pin: 1
  window_size: 1e+6
  C_conf: 0.0
  Q_conf: 1.5
  min_match_frac: 0.1
  pixel_border: -10
  depth_eps: 1e-6
  max_iters: 10
  sigma_ray: 0.003
  sigma_dist: 1e+1
  sigma_pixel: 1.0
  sigma_depth: 1e+1
  sigma_point: 0.05
  delta_norm: 1e-8
  use_cuda: True
```

### pin
- **类型**: Integer
- **默认值**: 1
- **说明**: 固定的初始关键帧数量
- **用途**: 前 N 个关键帧位姿固定不优化

### window_size
- **类型**: Float
- **默认值**: 1e+6 (实际上是无限大)
- **说明**: 优化窗口大小
- **用途**: 限制参与优化的关键帧数量

### use_cuda
- **类型**: Boolean
- **默认值**: True
- **说明**: 是否使用 CUDA 加速

**其他参数**: 与 tracking 部分类似，但用于全局优化

---

## 6. 检索配置

```yaml
retrieval:
  k: 3
  min_thresh: 5e-3
```

### k
- **类型**: Integer
- **默认值**: 3
- **说明**: 检索的候选关键帧数量

### min_thresh
- **类型**: Float
- **默认值**: 0.005
- **说明**: 最小相似度阈值

---

## 7. 重定位配置

```yaml
reloc:
  min_match_frac: 0.3
  strict: True
```

### min_match_frac
- **类型**: Float
- **默认值**: 0.3
- **说明**: 重定位的最小匹配比例

### strict
- **类型**: Boolean
- **默认值**: True
- **说明**: 是否使用严格模式
  - `True`: 要求更高的匹配质量
  - `False`: 宽松的重定位条件

---

## 8. 质量评估配置

```yaml
quality:
  patch_size: 16
  batch_size: 4
  max_wait_ms: 10
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

### patch_size
- **类型**: Integer
- **默认值**: 16
- **说明**: 质量评估的补丁大小（像素）

### batch_size
- **类型**: Integer
- **默认值**: 4
- **说明**: 批处理大小

### max_wait_ms
- **类型**: Float
- **默认值**: 10
- **说明**: 批次收集最大等待时间（毫秒）

### metrics.coverage.alpha_ema
- **类型**: Float
- **默认值**: 0.8
- **范围**: [0, 1]
- **说明**: EWMA 平滑系数
- **公式**: `new = α * prev + (1-α) * current`

### metrics.coverage.b0
- **类型**: Float
- **默认值**: 0.15
- **说明**: 基线距离阈值（米）

### metrics.coverage.theta0_deg
- **类型**: Float
- **默认值**: 10.0
- **说明**: 基线角度阈值（度）

### thresholds.z_r
- **类型**: Float
- **默认值**: 1.0
- **说明**: 重投影误差 z-score 阈值

### thresholds.z_u
- **类型**: Float
- **默认值**: 1.0
- **说明**: 不确定性 z-score 阈值

### thresholds.d_cov
- **类型**: Float
- **默认值**: 0.02
- **说明**: 协方差变化阈值

---

## 9. TSDF精化配置

```yaml
tsdf_refine:
  enabled: true
  window_size: 5
  quality_wait_ms: 500
  max_pending_tasks: 50
  voxel_size: 0.02
  trunc_dist: 0.08
  max_grid_dim: 64
  roi_size: 0.4
  ray_samples: 64
  max_displacement: 0.015
  min_weight_threshold: 0.01
  confidence_boost: 0.08
  confidence_max: 1.3
  min_hit_rate: 0.05
  max_rois_per_kf: 3
  min_confidence: 0.2
  pose_stable_thresh: 0.01
  max_shutdown_wait_s: 60
  min_shutdown_wait_s: 2
```

### enabled
- **类型**: Boolean
- **默认值**: true
- **说明**: 是否启用 TSDF 精化

### window_size
- **类型**: Integer
- **默认值**: 5
- **说明**: 滑动窗口大小
- **用途**: 只精化最近 N 个关键帧

### quality_wait_ms
- **类型**: Float
- **默认值**: 500
- **说明**: 等待质量结果的超时时间（毫秒）

### max_pending_tasks
- **类型**: Integer
- **默认值**: 50
- **说明**: 任务队列最大大小

### voxel_size
- **类型**: Float
- **默认值**: 0.02 (2cm)
- **说明**: TSDF 体素大小（米）
- **建议**: 0.01-0.03

### trunc_dist
- **类型**: Float
- **默认值**: 0.08 (8cm)
- **说明**: TSDF 截断距离（米）
- **建议**: 4 × voxel_size

### max_grid_dim
- **类型**: Integer
- **默认值**: 64
- **说明**: 最大网格维度

### roi_size
- **类型**: Float
- **默认值**: 0.4 (40cm)
- **说明**: 感兴趣区域大小（米）

### ray_samples
- **类型**: Integer
- **默认值**: 64
- **说明**: 每条射线的采样数

### max_displacement
- **类型**: Float
- **默认值**: 0.015 (1.5cm)
- **说明**: 最大允许位移（米）
- **用途**: 防止过度精化破坏跟踪

### min_weight_threshold
- **类型**: Float
- **默认值**: 0.01
- **说明**: 最小 TSDF 权重阈值

### confidence_boost
- **类型**: Float
- **默认值**: 0.08
- **说明**: 置信度提升量
- **用途**: 精化后增加点的置信度

### confidence_max
- **类型**: Float
- **默认值**: 1.3
- **说明**: 置信度上限

### min_hit_rate
- **类型**: Float
- **默认值**: 0.05 (5%)
- **说明**: 最小命中率
- **用途**: 低于此值不融合精化结果

### max_rois_per_kf
- **类型**: Integer
- **默认值**: 3
- **说明**: 每个关键帧最大 ROI 数量

### min_confidence
- **类型**: Float
- **默认值**: 0.2
- **说明**: 最小置信度阈值

### pose_stable_thresh
- **类型**: Float
- **默认值**: 0.01
- **说明**: 位姿稳定性阈值
- **用途**: 检测后端位姿更新

### max_shutdown_wait_s / min_shutdown_wait_s
- **类型**: Float
- **默认值**: 60 / 2
- **说明**: 关闭时的等待时间（秒）

---

## 10. TSDF全局配置

```yaml
tsdf_global:
  enabled: false
  voxel_size: 0.03
  trunc_dist: 0.12
  max_weight: 100.0
  min_tsdf_weight: 1.0e-3
  max_points_per_kf: 40000
  min_confidence: 0.05
  samples_per_kf: 2000
  lambda: 0.15
  max_iterations: 3
  pre_icp_iters: 2
  damping: 1.0e-4
  queue_check_interval: 0.1
  reintegration_queue: 256
  log_interval: 30
```

### enabled
- **类型**: Boolean
- **默认值**: false
- **说明**: 是否启用全局 TSDF
- **注意**: 默认禁用，需要时手动启用

### voxel_size
- **类型**: Float
- **默认值**: 0.03 (3cm)
- **说明**: 全局 TSDF 体素大小（米）

### trunc_dist
- **类型**: Float
- **默认值**: 0.12 (12cm)
- **说明**: TSDF 截断距离（米）

### max_weight
- **类型**: Float
- **默认值**: 100.0
- **说明**: 每个体素的最大融合权重

### min_tsdf_weight
- **类型**: Float
- **默认值**: 0.001
- **说明**: 有效体素的最小权重阈值

### max_points_per_kf
- **类型**: Integer
- **默认值**: 40000
- **说明**: 每个关键帧融合的最大点数
- **用途**: 采样降低计算量

### min_confidence
- **类型**: Float
- **默认值**: 0.05
- **说明**: 忽略低于此置信度的点

### samples_per_kf
- **类型**: Integer
- **默认值**: 2000
- **说明**: TSDF 优化时每个关键帧的采样点数

### lambda
- **类型**: Float
- **默认值**: 0.15
- **说明**: TSDF 残差在优化器中的权重

### max_iterations
- **类型**: Integer
- **默认值**: 3
- **说明**: Gauss-Newton 迭代次数

### pre_icp_iters
- **类型**: Integer
- **默认值**: 2
- **说明**: 因子图优化前的 ICP 迭代次数

### damping
- **类型**: Float
- **默认值**: 1e-4
- **说明**: Levenberg 阻尼系数

### queue_check_interval
- **类型**: Float
- **默认值**: 0.1
- **说明**: 队列检查间隔（秒）

### reintegration_queue
- **类型**: Integer
- **默认值**: 256
- **说明**: 重新融合队列大小

### log_interval
- **类型**: Float
- **默认值**: 30
- **说明**: 日志输出间隔（秒）

---

## 配置建议

### 快速模式（低精度，高速度）
```yaml
dataset:
  subsample: 2
  img_downsample: 2

tracking:
  max_iters: 20

tsdf_refine:
  enabled: false
```

### 高质量模式（高精度，低速度）
```yaml
dataset:
  subsample: 1
  img_downsample: 1

tracking:
  max_iters: 100

tsdf_refine:
  enabled: true
  voxel_size: 0.01
  max_rois_per_kf: 5
```

### 标定模式
```yaml
use_calib: true

tracking:
  sigma_pixel: 1.0
  sigma_depth: 1.0
  pixel_border: 5
```

---

**文档版本**: 1.0  
**更新日期**: 2025-11-19  
**作者**: MASt3R-SLAM Team
