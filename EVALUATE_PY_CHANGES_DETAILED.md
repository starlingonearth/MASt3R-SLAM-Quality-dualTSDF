# evaluate.py 新增变更说明 (Detailed Changes)

本文档详细记录了 `mast3r_slam/evaluate.py` 相对于原始基线版本的所有新增内容。

## 目录
1. [新增函数概述](#1-新增函数概述)
2. [save_ply_with_quality 详解](#2-save_ply_with_quality-详解)
3. [与质量服务集成](#3-与质量服务集成)
4. [总结](#4-总结)

---

## 1. 新增函数概述

### 1.1 新增函数

**完整新增函数** (Lines 108-186):
```python
# NEW: Save PLY with quality attributes (lines 108-186)
def save_ply_with_quality(savedir, filename, keyframes, c_conf_threshold, quality_service, patch_size=16):
```

**功能**:
- 保存带有质量属性的点云文件
- 扩展标准 PLY 格式，包含质量评估数据
- 支持可视化和分析点云质量

**与原有函数的关系**:
| 函数 | 功能 | 输出属性 |
|------|------|----------|
| `save_reconstruction` | 保存基本点云 | x, y, z, red, green, blue |
| `save_ply_with_quality` | 保存带质量的点云 | x, y, z, red, green, blue, **r, delta_cov, u, class_id, priority** |

---

## 2. save_ply_with_quality 详解

### 2.1 函数签名

```python
def save_ply_with_quality(
    savedir,              # 保存目录
    filename,             # 文件名
    keyframes,            # SharedKeyframes 对象
    c_conf_threshold,     # 置信度阈值
    quality_service,      # 质量服务对象
    patch_size=16         # 补丁大小（未使用）
):
```

**参数说明**:
- `savedir`: 输出目录路径
- `filename`: PLY 文件名
- `keyframes`: 包含所有关键帧的共享数据结构
- `c_conf_threshold`: 过滤低置信度点的阈值
- `quality_service`: `AsynchronousQualityService` 实例，提供质量数据
- `patch_size`: 预留参数，当前未使用

### 2.2 主要流程

#### 2.2.1 初始化 (Lines 110-113)

```python
savedir = pathlib.Path(savedir)
savedir.mkdir(exist_ok=True, parents=True)
points_all, colors_all = [], []
r_all, dc_all, u_all, cid_all, pri_all = [], [], [], [], []
```

**说明**:
- 创建输出目录
- 初始化数据累积列表：
  - `points_all`: 3D 点坐标
  - `colors_all`: RGB 颜色
  - `r_all`: 重投影误差 (residual)
  - `dc_all`: 协方差变化 (delta covariance)
  - `u_all`: 不确定性 (uncertainty)
  - `cid_all`: 分类 ID (class ID)
  - `pri_all`: 优先级 (priority)

#### 2.2.2 遍历关键帧 (Lines 114-162)

```python
for i in range(len(keyframes)):
    kf = keyframes[i]
```

**每个关键帧的处理步骤**:

##### Step 1: 点云约束 (Lines 116-118)
```python
if config["use_calib"]:
    X_canon = constrain_points_to_ray(kf.img_shape.flatten()[:2], kf.X_canon[None], kf.K)
    kf.X_canon = X_canon.squeeze(0)
```

**说明**:
- 如果使用相机标定，将点约束到相机射线上
- 确保点云几何一致性

##### Step 2: 转换到世界坐标 (Lines 119-120)
```python
pW = kf.T_WC.act(kf.X_canon).cpu().numpy().reshape(-1, 3)
col = (kf.uimg.cpu().numpy() * 255).astype(np.uint8).reshape(-1, 3)
valid = (kf.get_average_conf().cpu().numpy().astype(np.float32).reshape(-1) > c_conf_threshold)
```

**说明**:
- `pW`: 世界坐标系中的点云 (N×3)
- `col`: RGB 颜色 (N×3)
- `valid`: 置信度过滤掩码 (N,)

##### Step 3: 获取质量数据 (Lines 122-154)

**获取图像尺寸**:
```python
H, W = int(kf.img_shape.flatten()[0]), int(kf.img_shape.flatten()[1])
res = quality_service.get(kf.frame_id) if quality_service is not None else None
```

**定义上采样函数** (Lines 127-137):
```python
def up(g, mode):
    # Handle both tensor and numpy array
    if torch.is_tensor(g):
        gnp = g.detach().cpu().numpy().astype(np.float32)
    elif isinstance(g, np.ndarray):
        gnp = g.astype(np.float32)
    else:
        gnp = np.array(g, dtype=np.float32)
    gh, gw = gnp.shape[-2], gnp.shape[-1]
    out = cv2.resize(gnp, (W, H), interpolation=cv2.INTER_NEAREST if mode=="nearest" else cv2.INTER_LINEAR)
    return out.reshape(-1)
```

**功能**:
- 将质量图从低分辨率上采样到原始图像分辨率
- 支持两种插值模式：
  - `"linear"`: 双线性插值（用于连续值）
  - `"nearest"`: 最近邻插值（用于离散值）
- 兼容 PyTorch tensor 和 NumPy array

**上采样质量属性** (Lines 138-147):
```python
if res is not None:
    dc = up(res["delta_cov"], "linear")
    rr = up(res["r"], "linear")
    uu = up(res["u"], "linear")
    
    # Handle class_id conversion
    if torch.is_tensor(res["class_id"]):
        class_id_float = res["class_id"].float()
    else:
        class_id_float = res["class_id"].astype(np.float32)
    cc = up(class_id_float, "nearest").astype(np.uint8)
    pp = up(res["priority"], "linear")
```

**质量属性说明**:
- `delta_cov`: 协方差变化，反映几何不确定性
- `r`: 重投影误差，反映观测一致性
- `u`: 综合不确定性度量
- `class_id`: 质量分类（0=优秀, 1=良好, 2=中等, 3=差）
- `priority`: 精化优先级

**默认值处理** (Lines 148-154):
```python
else:
    n = H * W
    dc = np.zeros((n,), np.float32)
    rr = np.zeros((n,), np.float32)
    uu = np.zeros((n,), np.float32)
    cc = np.zeros((n,), np.uint8)
    pp = np.zeros((n,), np.float32)
```

**说明**:
- 如果质量服务未提供数据，使用零值
- 确保所有关键帧都有完整的属性

##### Step 4: 累积数据 (Lines 156-162)
```python
points_all.append(pW[valid])
colors_all.append(col[valid])
r_all.append(rr[valid])
dc_all.append(dc[valid])
u_all.append(uu[valid])
cid_all.append(cc[valid])
pri_all.append(pp[valid])
```

**说明**:
- 只保留置信度高于阈值的点
- 所有属性都应用相同的掩码

#### 2.2.3 合并和保存 (Lines 164-186)

**合并所有关键帧的数据**:
```python
points = np.concatenate(points_all, 0)
colors = np.concatenate(colors_all, 0)
r = np.concatenate(r_all, 0).astype(np.float32)
dc = np.concatenate(dc_all, 0).astype(np.float32)
u = np.concatenate(u_all, 0).astype(np.float32)
cid = np.concatenate(cid_all, 0).astype(np.uint8)
pri = np.concatenate(pri_all, 0).astype(np.float32)
```

**创建结构化数组**:
```python
pcd = np.empty(points.shape[0], dtype=[
    ("x","f4"),("y","f4"),("z","f4"),
    ("red","u1"),("green","u1"),("blue","u1"),
    ("r","f4"),("delta_cov","f4"),("u","f4"),
    ("class_id","u1"),("priority","f4"),
])
```

**PLY 属性定义**:
| 属性名 | 类型 | 字节数 | 说明 |
|--------|------|--------|------|
| x, y, z | f4 | 4 | 3D 坐标 (float32) |
| red, green, blue | u1 | 1 | RGB 颜色 (uint8) |
| r | f4 | 4 | 重投影误差 (float32) |
| delta_cov | f4 | 4 | 协方差变化 (float32) |
| u | f4 | 4 | 不确定性 (float32) |
| class_id | u1 | 1 | 质量分类 (uint8) |
| priority | f4 | 4 | 优先级 (float32) |

**总大小**: 3×4 + 3×1 + 5×4 = 35 字节/点

**填充数据并写入**:
```python
pcd["x"], pcd["y"], pcd["z"] = points.T
pcd["red"], pcd["green"], pcd["blue"] = colors.T
pcd["r"] = r
pcd["delta_cov"] = dc
pcd["u"] = u
pcd["class_id"] = cid
pcd["priority"] = pri
vertex_element = PlyElement.describe(pcd, "vertex")
PlyData([vertex_element], text=False).write(savedir / filename)
```

---

## 3. 与质量服务集成

### 3.1 质量服务接口

**AsynchronousQualityService.get() 方法**:
```python
def get(self, frame_id):
    """
    获取指定帧的质量数据
    
    Returns:
        dict or None: {
            "delta_cov": torch.Tensor or np.ndarray,  # (H', W')
            "r": torch.Tensor or np.ndarray,          # (H', W')
            "u": torch.Tensor or np.ndarray,          # (H', W')
            "class_id": torch.Tensor or np.ndarray,   # (H', W')
            "priority": torch.Tensor or np.ndarray,   # (H', W')
        }
    """
```

**注意**:
- 质量图的分辨率 (H', W') 可能与原始图像 (H, W) 不同
- 需要上采样到原始分辨率

### 3.2 数据流图

```
关键帧 (H×W)
    ↓
质量服务计算 (可能在低分辨率 H'×W')
    ↓
质量数据存储
    ↓
save_ply_with_quality()
    ↓
quality_service.get(frame_id)
    ↓
上采样到 H×W
    ↓
应用置信度掩码
    ↓
合并所有关键帧
    ↓
保存 PLY 文件
```

### 3.3 调用示例

**在 main.py 中**:
```python
# 初始化质量服务
quality_service = AsynchronousQualityService(manager=manager)

# ... SLAM 运行 ...

# 保存结果
if dataset.save_results:
    save_dir, seq_name = eval.prepare_savedir(args, dataset)
    
    # 保存标准点云
    eval.save_reconstruction(
        save_dir,
        f"{seq_name}.ply",
        keyframes,
        0.0,
    )
    
    # 保存带质量的点云
    eval.save_ply_with_quality(
        save_dir,
        f"{seq_name}_quality.ply",
        keyframes,
        0.0,
        quality_service,
    )
```

---

## 4. 总结

### 4.1 主要变更

| 变更项 | 类型 | 功能 |
|--------|------|------|
| `save_ply_with_quality` | 新增函数 | 保存带质量属性的 PLY |
| 上采样逻辑 | 新增功能 | 处理不同分辨率的质量图 |
| 类型兼容性 | 新增功能 | 支持 tensor 和 numpy array |

### 4.2 功能增强

#### 4.2.1 质量可视化
**使用 CloudCompare 或 MeshLab 可视化**:
```bash
# 打开 PLY 文件
cloudcompare sequence_quality.ply

# 按质量属性着色
# - 选择 "Scalar Field" → "r" (重投影误差)
# - 选择 "Scalar Field" → "class_id" (质量分类)
# - 选择 "Scalar Field" → "priority" (优先级)
```

**颜色映射示例**:
- `r` (重投影误差): 蓝色(低) → 红色(高)
- `class_id`: 绿色(0) → 黄色(1) → 橙色(2) → 红色(3)
- `priority`: 灰色(低) → 白色(高)

#### 4.2.2 质量分析
**Python 分析脚本**:
```python
from plyfile import PlyData

# 读取 PLY
ply = PlyData.read("sequence_quality.ply")
vertices = ply['vertex']

# 提取质量属性
r = vertices['r']
delta_cov = vertices['delta_cov']
u = vertices['u']
class_id = vertices['class_id']
priority = vertices['priority']

# 统计分析
print(f"Mean reprojection error: {r.mean():.4f}")
print(f"Std reprojection error: {r.std():.4f}")
print(f"Quality distribution:")
for i in range(4):
    count = (class_id == i).sum()
    percent = count / len(class_id) * 100
    print(f"  Class {i}: {count} ({percent:.1f}%)")
```

#### 4.2.3 质量过滤
**基于质量属性过滤点云**:
```python
# 只保留高质量点
high_quality_mask = (class_id <= 1) & (r < 2.0)
filtered_points = vertices[high_quality_mask]

# 保存过滤后的点云
save_filtered_ply("high_quality.ply", filtered_points)
```

### 4.3 文件大小对比

**假设场景**: 100 个关键帧，每个 224×224 像素，50% 有效点

**标准 PLY**:
- 点数: 100 × 224 × 224 × 0.5 = 2,508,800 点
- 每点: 3×4 (xyz) + 3×1 (rgb) = 15 字节
- 总大小: ~36 MB

**质量 PLY**:
- 点数: 2,508,800 点
- 每点: 15 + 5×4 + 1×1 = 36 字节
- 总大小: ~86 MB

**增加**: ~50 MB (2.4倍)

### 4.4 使用建议

#### 4.4.1 性能优化
```python
# 如果不需要质量数据，使用标准保存
if not need_quality_analysis:
    save_reconstruction(...)  # 更快，文件更小
else:
    save_ply_with_quality(...)  # 更详细，但更慢更大
```

#### 4.4.2 置信度阈值选择
```python
# 低阈值: 保留更多点，但可能包含噪声
save_ply_with_quality(..., c_conf_threshold=0.0)

# 中等阈值: 平衡质量和密度
save_ply_with_quality(..., c_conf_threshold=0.5)

# 高阈值: 只保留高质量点
save_ply_with_quality(..., c_conf_threshold=1.0)
```

#### 4.4.3 质量服务检查
```python
# 确保质量服务已初始化
if quality_service is None:
    print("Warning: Quality service not available, using default values")

# 检查质量数据可用性
for i in range(len(keyframes)):
    kf = keyframes[i]
    res = quality_service.get(kf.frame_id)
    if res is None:
        print(f"Warning: No quality data for frame {kf.frame_id}")
```

### 4.5 潜在问题和解决方案

#### 4.5.1 分辨率不匹配
**问题**: 质量图分辨率与图像不匹配

**解决**: 上采样函数自动处理
```python
def up(g, mode):
    # 自动检测并调整分辨率
    gh, gw = gnp.shape[-2], gnp.shape[-1]
    out = cv2.resize(gnp, (W, H), ...)
```

#### 4.5.2 类型不兼容
**问题**: 质量数据可能是 tensor 或 numpy array

**解决**: 类型检查和转换
```python
if torch.is_tensor(g):
    gnp = g.detach().cpu().numpy()
elif isinstance(g, np.ndarray):
    gnp = g
else:
    gnp = np.array(g)
```

#### 4.5.3 内存占用
**问题**: 大规模点云可能导致内存不足

**解决**: 分批处理
```python
# 修改为分批保存
def save_ply_with_quality_batched(savedir, filename, keyframes, batch_size=10):
    for start in range(0, len(keyframes), batch_size):
        end = min(start + batch_size, len(keyframes))
        batch_kfs = keyframes[start:end]
        # 处理批次...
```

### 4.6 扩展功能

#### 4.6.1 自定义质量属性
```python
def save_ply_with_custom_attributes(savedir, filename, keyframes, attributes):
    """
    attributes: dict of {name: (data, dtype)}
    """
    dtype_list = [
        ("x","f4"),("y","f4"),("z","f4"),
        ("red","u1"),("green","u1"),("blue","u1"),
    ]
    for name, (data, dtype) in attributes.items():
        dtype_list.append((name, dtype))
    
    pcd = np.empty(n_points, dtype=dtype_list)
    # ... 填充数据 ...
```

#### 4.6.2 质量报告生成
```python
def generate_quality_report(ply_file):
    """生成质量分析报告"""
    ply = PlyData.read(ply_file)
    vertices = ply['vertex']
    
    report = {
        'total_points': len(vertices),
        'mean_error': vertices['r'].mean(),
        'std_error': vertices['r'].std(),
        'quality_distribution': {},
    }
    
    for i in range(4):
        count = (vertices['class_id'] == i).sum()
        report['quality_distribution'][i] = {
            'count': count,
            'percentage': count / len(vertices) * 100
        }
    
    return report
```

---

## 附录：完整的质量属性说明

### A.1 重投影误差 (r)
- **定义**: 观测点与重投影点之间的距离
- **单位**: 像素（标定模式）或射线距离（非标定模式）
- **范围**: [0, ∞)
- **解释**:
  - r < 1.0: 优秀
  - 1.0 ≤ r < 2.0: 良好
  - 2.0 ≤ r < 5.0: 中等
  - r ≥ 5.0: 差

### A.2 协方差变化 (delta_cov)
- **定义**: 点云协方差矩阵的变化量
- **单位**: 无量纲
- **范围**: [0, ∞)
- **解释**: 值越大，几何不确定性越高

### A.3 不确定性 (u)
- **定义**: 综合不确定性度量
- **计算**: 结合重投影误差、协方差、观测数量等
- **范围**: [0, 1]
- **解释**: 0=确定，1=不确定

### A.4 质量分类 (class_id)
- **定义**: 离散的质量等级
- **值**:
  - 0: 优秀 (Excellent)
  - 1: 良好 (Good)
  - 2: 中等 (Fair)
  - 3: 差 (Poor)
- **用途**: 快速过滤和可视化

### A.5 优先级 (priority)
- **定义**: TSDF 精化的优先级
- **范围**: [0, ∞)
- **解释**: 值越高，越需要精化
- **用途**: 指导 TSDF 精化器的调度策略

---

**文档版本**: 1.0  
**更新日期**: 2025-11-19  
**作者**: MASt3R-SLAM Team
