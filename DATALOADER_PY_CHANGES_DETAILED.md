# dataloader.py 新增变更说明 (Detailed Changes)

本文档详细记录了 `mast3r_slam/dataloader.py` 相对于原始基线版本的所有新增内容。

## 目录
1. [新增导入](#1-新增导入)
2. [路径兼容性改进](#2-路径兼容性改进)
3. [ReplicaDataset 新增类](#3-replicadataset-新增类)
4. [load_dataset 函数重构](#4-load_dataset-函数重构)
5. [总结](#5-总结)

---

## 1. 新增导入

### 1.1 导入 json 和 os (Lines 9-10)

```python
import json  # NEW: For ReplicaDataset cam_params.json parsing
import os    # NEW: For load_dataset path extension handling
```

**说明**:
- `json`: 用于解析 Replica 数据集的相机参数文件
- `os`: 用于 `load_dataset` 中的路径扩展名处理

---

## 2. 路径兼容性改进

### 2.1 MonocularDataset.read_img (Line 45)

```python
def read_img(self, idx):
    img = cv2.imread(str(self.rgb_files[idx]))  # NEW: Added str() for pathlib compatibility
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
```

### 2.2 EurocDataset.read_img (Line 117)

```python
def read_img(self, idx):
    img = cv2.imread(str(self.rgb_files[idx]), cv2.IMREAD_GRAYSCALE)  # NEW: Added str() for pathlib compatibility
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
```

**说明**:
- 添加 `str()` 转换确保 pathlib.Path 对象可以被 cv2.imread 正确处理
- 提高跨平台兼容性

---

## 3. ReplicaDataset 新增类

### 3.1 类概述 (Lines 278-472)

完整的 Replica 数据集适配器，支持灵活的目录结构和多种相机参数格式。

**数据集结构**:
```
datasets/Replica/
  cam_params.json
  office0/
    results/
      frame000000.jpg
      frame000001.jpg
      ...
    traj.txt  # optional
```

### 3.2 主要功能

#### 图像目录解析
- 自动查找 `results/`, `results/color/`, `results/rgb/`
- 支持递归搜索
- 排除深度图像

#### 时间戳处理
- 优先使用 `traj.txt` 第一列
- 回退到 30 FPS 合成时间戳

#### 相机内参解析
支持多种 JSON 格式：
1. 分离的标量键: `{"fx":..., "fy":..., "cx":..., "cy":...}`
2. 内参向量: `{"intrinsics":[fx, fy, cx, cy]}`
3. K 矩阵: `{"K":[[fx,0,cx],[0,fy,cy],[0,0,1]]}`

### 3.3 关键方法

**_parse_cam_dict**: 解析相机参数字典
- 支持 3 种常见格式
- 返回 (fx, fy, cx, cy, distortion)

---

## 4. load_dataset 函数重构

### 4.1 原始版本

```python
def load_dataset(dataset_path):
    split_dataset_type = dataset_path.split("/")
    if "tum" in split_dataset_type:
        return TUMDataset(dataset_path)
    # ...
    ext = split_dataset_type[-1].split(".")[-1]
    if ext in ["mp4", "avi", "MOV", "mov"]:
        return MP4Dataset(dataset_path)
    return RGBFiles(dataset_path)
```

### 4.2 新版本 (Lines 518-541)

```python
# NEW: Refactored load_dataset function (lines 518-541)
def load_dataset(dataset_path: str):
    tokens = [s.lower() for s in re.split(r'[\\/]+', dataset_path)]
    # match Replica first
    if 'replica' in tokens:
        return ReplicaDataset(dataset_path)

    if 'tum' in tokens:
        return TUMDataset(dataset_path)
    if 'euroc' in tokens:
        return EurocDataset(dataset_path)
    if 'eth3d' in tokens:
        return ETH3DDataset(dataset_path)
    if '7-scenes' in tokens or '7scenes' in tokens or '7_scenes' in tokens:
        return SevenScenesDataset(dataset_path)
    if 'realsense' in tokens:
        return RealsenseDataset()
    if 'webcam' in tokens:
        return Webcam()

    ext = os.path.splitext(dataset_path)[1].lower()
    if ext in ['.mp4', '.avi', '.mov']:
        return MP4Dataset(dataset_path)
    return RGBFiles(dataset_path)
```

### 4.3 改进点

1. **跨平台路径分割**: `re.split(r'[\\/]+', dataset_path)` 支持 Windows 和 Unix
2. **新增 Replica 支持**: 优先匹配 Replica 数据集
3. **7-Scenes 变体**: 支持 `7-scenes`, `7scenes`, `7_scenes`
4. **标准化扩展名**: 使用 `os.path.splitext` 和 `.lower()`
5. **类型提示**: 添加 `dataset_path: str`

---

## 5. 总结

### 5.1 主要变更

| 变更项 | 类型 | 影响 |
|--------|------|------|
| json/os 导入 | 新增 | 支持新功能 |
| str() 转换 | 修复 | 提高兼容性 |
| ReplicaDataset | 新增类 | 支持 Replica 数据集 |
| load_dataset 重构 | 改进 | 更健壮的路径处理 |

### 5.2 使用示例

```python
# Replica 数据集
dataset = load_dataset("datasets/Replica/office0")

# 自动检测并加载
dataset = load_dataset("path/to/replica/scene")
```

### 5.3 兼容性

- ✅ Windows 路径 (`C:\datasets\...`)
- ✅ Unix 路径 (`/datasets/...`)
- ✅ pathlib.Path 对象
- ✅ 多种数据集格式

---

**文档版本**: 1.0  
**更新日期**: 2025-11-19  
**作者**: MASt3R-SLAM Team
