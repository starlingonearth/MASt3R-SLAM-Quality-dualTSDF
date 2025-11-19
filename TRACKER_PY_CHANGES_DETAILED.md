# tracker.py 新增变更说明 (Detailed Changes)

本文档详细记录了 `mast3r_slam/tracker.py` 相对于原始基线版本的所有新增内容。

## 目录
1. [质量评估集成](#1-质量评估集成)
2. [求解器优化](#2-求解器优化)
3. [总结](#3-总结)

---

## 1. 质量评估集成

### 1.1 质量评估主要逻辑 (Lines 95-142)

这是 `tracker.py` 中最主要的新增功能，在位姿优化成功后计算质量指标。

```python
# NEW: Quality evaluation section (lines 95-142)
# quality
if hasattr(self, "quality_service") and (self.quality_service is not None):
    print(f"DEBUG: Processing quality for keyframe {keyframe.frame_id}")
    try:
        with torch.no_grad():
            if config["use_calib"]:
                # Calibrated mode: compute pixel and depth residuals
                Xf_Ck = act_Sim3(T_CkCf, Xf, jacobian=False)
                pzf_Ck, valid_proj = project_calib(Xf_Ck, K, img_size)
                du = (meas_k[..., 0] - pzf_Ck[..., 0])
                dv = (meas_k[..., 1] - pzf_Ck[..., 1])
                dz = (meas_k[..., 2] - pzf_Ck[..., 2])
                lam = (self.cfg["sigma_pixel"] ** 2) / (self.cfg["sigma_depth"] ** 2 + 1e-8)
                r_pix = torch.sqrt(du * du + dv * dv + lam * dz * dz)
                valid_submit = (valid_kf & valid_proj & valid_meas_k).view(-1)
            else:
                # Uncalibrated mode: compute ray-distance residuals
                Xf_Ck = act_Sim3(T_CkCf, Xf, jacobian=False)
                rd_k = point_to_ray_dist(Xk, jacobian=False)
                rd_f = point_to_ray_dist(Xf_Ck, jacobian=False)
                r_pix = torch.linalg.norm(rd_k - rd_f, dim=1)
                valid_submit = valid_kf.view(-1)

            # Extract pose parameters for quality metrics
            vec = T_CkCf.data.view(-1, 8)
            t = vec[..., :3]
            q = vec[..., 3:7]
            w = q[..., -1].clamp(-1.0, 1.0).abs()
            theta = 2.0 * torch.arccos(w)
            t_norm = t.norm(dim=-1)

            # Submit quality job with keyframe and frame IDs
            current_kf_id = len(self.keyframes) - 1
            job = {
                "kf_id": int(current_kf_id),  # Keyframe index
                "frame_id": int(keyframe.frame_id),  # Original frame ID
                "H": int(img_size[0]),
                "W": int(img_size[1]),
                "valid_kf": valid_submit.cpu().numpy(),
                "r_pix": r_pix.view(-1).cpu().numpy(),
                "Ck": Ck.view(-1).cpu().numpy(),
                "Qk": Qk.view(-1).cpu().numpy(),
                "t_norm": float(t_norm.mean().item()),
                "theta": float(theta.mean().item()),
            }
            self.quality_service.submit(job)
    except Exception as e:
        print(f"ERROR in quality submission: {e}")
        import traceback
        traceback.print_exc()
```

**说明**:

#### 1.1.1 功能概述
- 在位姿跟踪成功后，计算当前帧与关键帧之间的质量指标
- 支持标定和非标定两种模式
- 将质量数据提交给异步质量服务进行处理

#### 1.1.2 标定模式 (use_calib=True)
计算像素和深度残差：

1. **点变换**: 将当前帧点云变换到关键帧坐标系
   ```python
   Xf_Ck = act_Sim3(T_CkCf, Xf, jacobian=False)
   ```

2. **投影**: 使用相机内参投影点云
   ```python
   pzf_Ck, valid_proj = project_calib(Xf_Ck, K, img_size)
   ```

3. **残差计算**: 计算像素坐标和深度的差异
   ```python
   du = (meas_k[..., 0] - pzf_Ck[..., 0])  # u方向残差
   dv = (meas_k[..., 1] - pzf_Ck[..., 1])  # v方向残差
   dz = (meas_k[..., 2] - pzf_Ck[..., 2])  # 深度残差
   ```

4. **加权残差**: 使用配置的噪声参数加权
   ```python
   lam = (self.cfg["sigma_pixel"] ** 2) / (self.cfg["sigma_depth"] ** 2 + 1e-8)
   r_pix = torch.sqrt(du * du + dv * dv + lam * dz * dz)
   ```

5. **有效性掩码**: 结合多个有效性条件
   ```python
   valid_submit = (valid_kf & valid_proj & valid_meas_k).view(-1)
   ```

#### 1.1.3 非标定模式 (use_calib=False)
计算射线-距离残差：

1. **点变换**: 同标定模式
   ```python
   Xf_Ck = act_Sim3(T_CkCf, Xf, jacobian=False)
   ```

2. **射线距离计算**: 计算点到射线的距离和射线方向
   ```python
   rd_k = point_to_ray_dist(Xk, jacobian=False)  # 关键帧
   rd_f = point_to_ray_dist(Xf_Ck, jacobian=False)  # 当前帧
   ```

3. **残差计算**: 计算两个射线距离表示的差异
   ```python
   r_pix = torch.linalg.norm(rd_k - rd_f, dim=1)
   ```

4. **有效性掩码**: 仅使用关键帧有效性
   ```python
   valid_submit = valid_kf.view(-1)
   ```

#### 1.1.4 位姿参数提取
从相对位姿中提取平移和旋转信息：

```python
vec = T_CkCf.data.view(-1, 8)  # Sim3参数: [t(3), q(4), s(1)]
t = vec[..., :3]  # 平移向量
q = vec[..., 3:7]  # 四元数
w = q[..., -1].clamp(-1.0, 1.0).abs()  # 四元数实部
theta = 2.0 * torch.arccos(w)  # 旋转角度
t_norm = t.norm(dim=-1)  # 平移范数
```

**位姿参数用途**:
- `t_norm`: 平移距离，反映相机运动幅度
- `theta`: 旋转角度，反映相机旋转幅度
- 这些参数用于质量评估中的几何一致性检查

#### 1.1.5 质量任务提交
构建质量评估任务并提交给异步服务：

```python
job = {
    "kf_id": int(current_kf_id),        # 关键帧在列表中的索引
    "frame_id": int(keyframe.frame_id),  # 关键帧的原始帧ID
    "H": int(img_size[0]),               # 图像高度
    "W": int(img_size[1]),               # 图像宽度
    "valid_kf": valid_submit.cpu().numpy(),  # 有效点掩码 (H*W,)
    "r_pix": r_pix.view(-1).cpu().numpy(),   # 像素残差 (H*W,)
    "Ck": Ck.view(-1).cpu().numpy(),         # 置信度 (H*W,)
    "Qk": Qk.view(-1).cpu().numpy(),         # 质量分数 (H*W,)
    "t_norm": float(t_norm.mean().item()),   # 平移范数
    "theta": float(theta.mean().item()),     # 旋转角度
}
self.quality_service.submit(job)
```

**任务字段说明**:
- `kf_id`: 用于索引 `SharedKeyframes` 中的关键帧
- `frame_id`: 原始数据集中的帧编号
- `valid_kf`: 布尔数组，标记哪些像素有有效的质量数据
- `r_pix`: 重投影误差或射线距离误差
- `Ck`: MASt3R模型输出的置信度
- `Qk`: 匹配质量分数
- `t_norm`, `theta`: 用于检测运动退化情况

#### 1.1.6 异常处理
```python
except Exception as e:
    print(f"ERROR in quality submission: {e}")
    import traceback
    traceback.print_exc()
```

**说明**:
- 质量评估失败不影响跟踪主流程
- 打印详细错误信息便于调试
- 确保跟踪器的鲁棒性

---

## 2. 求解器优化

### 2.1 Cholesky求解修改 (Line 221)

**原始版本**:
```python
L = torch.linalg.cholesky(H, upper=False)
tau_j = torch.cholesky_solve(g, L, upper=False).view(1, -1)
```

**新版本**:
```python
# NEW: Modified Cholesky solve (line 221)
tau_j = torch.cholesky_solve(g, torch.linalg.cholesky(H)).view(1, -1)
```

**说明**:
- 简化了Cholesky分解和求解的代码
- 直接在 `cholesky_solve` 调用中进行分解
- 功能等价，但代码更简洁
- 移除了中间变量 `L`

**技术细节**:
- `torch.linalg.cholesky(H)` 默认返回下三角矩阵
- `torch.cholesky_solve` 默认假设输入是下三角矩阵
- 这种写法减少了一行代码和一个临时变量

---

## 3. 总结

### 3.1 主要新增功能

#### 质量评估系统集成
1. **双模式支持**:
   - 标定模式: 像素+深度残差
   - 非标定模式: 射线距离残差

2. **完整的质量指标**:
   - 重投影误差 (`r_pix`)
   - 置信度 (`Ck`)
   - 匹配质量 (`Qk`)
   - 位姿参数 (`t_norm`, `theta`)

3. **异步处理架构**:
   - 不阻塞主跟踪流程
   - 通过 `quality_service` 异步提交任务
   - 支持批量处理和统计分析

4. **鲁棒性设计**:
   - 异常捕获和日志记录
   - 可选功能（通过 `hasattr` 检查）
   - 不影响核心跟踪功能

### 3.2 代码改进

#### 求解器优化
- 简化了Cholesky求解代码
- 提高了代码可读性
- 保持了数值稳定性

### 3.3 配置依赖

新增功能依赖以下配置参数：

```yaml
tracking:
  sigma_pixel: 1.0      # 像素噪声标准差
  sigma_depth: 0.1      # 深度噪声标准差
  sigma_ray: 0.01       # 射线噪声标准差
  sigma_dist: 0.01      # 距离噪声标准差
  depth_eps: 0.01       # 深度最小值
  pixel_border: 5       # 投影边界
```

### 3.4 与其他模块的交互

#### 输入依赖
- `self.quality_service`: 由 `main.py` 中的 `AsynchronousQualityService` 提供
- `config["use_calib"]`: 全局配置，决定使用哪种模式

#### 输出影响
- 质量数据提交到 `quality_service`
- 后续可用于：
  - 点云质量可视化
  - TSDF精化的权重计算
  - 关键帧选择策略
  - 重建质量评估

### 3.5 性能考虑

#### 计算开销
- 质量计算在 `torch.no_grad()` 下进行，不计算梯度
- 异步提交，不阻塞跟踪主循环
- 主要开销：
  - 点云变换: O(N)
  - 投影/射线距离: O(N)
  - 残差计算: O(N)
  - 其中 N = H × W (图像分辨率)

#### 内存使用
- 临时张量在函数作用域内自动释放
- 提交的数据转换为 NumPy 数组（CPU内存）
- 不会累积GPU内存

### 3.6 调试和监控

#### 调试输出
```python
print(f"DEBUG: Processing quality for keyframe {keyframe.frame_id}")
```
- 可通过此日志确认质量评估是否执行
- 生产环境可考虑使用日志级别控制

#### 错误追踪
```python
print(f"ERROR in quality submission: {e}")
traceback.print_exc()
```
- 完整的异常堆栈便于定位问题
- 不会因质量评估失败而中断跟踪

---

## 附录：质量评估数据流

```
FrameTracker.track()
    ↓
位姿优化成功
    ↓
质量评估 (if quality_service exists)
    ↓
计算残差 (标定/非标定)
    ↓
提取位姿参数
    ↓
构建质量任务
    ↓
quality_service.submit(job)
    ↓
异步处理队列
    ↓
质量分析和存储
    ↓
PLY导出 (带质量属性)
```

---

**文档版本**: 1.0  
**更新日期**: 2025-11-19  
**作者**: MASt3R-SLAM Team
