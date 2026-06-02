# 工业巡检机器人知识教学文档

> 本文档涵盖本项目中运用到的所有核心知识，从数学基础到工程实践，按知识领域组织，
> 每个概念都与项目实际代码关联，帮助读者深入理解"是什么、为什么、怎么用"。

---

## 目录

- [第一部分：数学基础](#第一部分数学基础)
  - [线性代数](#1-线性代数)
  - [概率论与统计](#2-概率论与统计)
  - [最优化方法](#3-最优化方法)
  - [坐标变换与刚体运动](#4-坐标变换与刚体运动)
- [第二部分：图像处理基础](#第二部分图像处理基础)
  - [数字图像基础](#5-数字图像基础)
  - [图像预处理](#6-图像预处理)
  - [边缘检测与特征提取](#7-边缘检测与特征提取)
  - [深度学习目标检测](#8-深度学习目标检测)
- [第三部分：软件工程基础](#第三部分软件工程基础)
  - [ROS2 架构原理](#9-ros2-架构原理)
  - [Gazebo 仿真原理](#10-gazebo-仿真原理)
  - [Docker 容器化](#11-docker-容器化)
  - [Git 版本管理](#12-git-版本管理)
- [第四部分：机器人学基础](#第四部分机器人学基础)
  - [运动学模型](#13-运动学模型)
  - [SLAM 原理](#14-slam-原理)
  - [路径规划](#15-路径规划)
- [第五部分：大模型应用](#第五部分大模型应用)
  - [LLM API 调用原理](#16-llm-api-调用原理)
  - [RAG 检索增强生成](#17-rag-检索增强生成)
  - [多模态模型](#18-多模态模型)
- [第六部分：工程整合实践](#第六部分工程整合实践)

---

## 第一部分：数学基础

### 1. 线性代数

#### 1.1 向量与矩阵

**核心概念**：
- **向量**：有序数组，表示方向+大小。在机器人学中，速度 `(v_x, v_y, ω)` 是一个三维向量。
- **矩阵**：二维数组，表示线性变换。旋转矩阵是 3×3 的正交矩阵。
- **点积 (Dot Product)**：`a·b = |a||b|cosθ`，用于计算向量夹角、投影。
- **矩阵乘法**：`C = AB`，将多个变换组合为一个。

**项目中的应用**：

```python
# 向量相似度计算 — 知识库语义搜索 (industrial_kb.py)
# 两个向量的点积越大，表示语义越相似
query_embedding = model.encode([query])       # 查询向量 [1, 384]
scores = np.dot(self._embeddings, query_embedding.T)  # [N, 1] 相似度矩阵
top_indices = np.argsort(scores.flatten())[::-1][:top_k]  # 相似度排序
```

```python
# 卷积操作 — 图像锐化 (preprocessor.py)
# 卷积核是一个 3×3 矩阵，与图像每个像素邻域做点积
kernel = np.array([[-1, -1, -1],
                   [-1,  9, -1],
                   [-1, -1, -1]]) / 1.0
enhanced = cv2.filter2D(enhanced, -1, kernel * 0.3)
```

**数学直觉**：可以把矩阵看作"空间变换器"—旋转矩阵将点从一个坐标系旋转到另一个，卷积核将图像从"普通"变换到"锐化"。

#### 1.2 特征值与特征向量

**核心概念**：对于方阵 A，若 `Av = λv`，则 λ 是特征值，v 是特征向量。特征值表示变换的"缩放因子"，特征向量表示"不变方向"。

**项目中的应用 — PCA 降维**：
在知识库语义搜索中，sentence-transformers 使用 BERT 模型内部的 PCA 压缩嵌入向量。嵌入维度从 768 降到 384，保留主要语义信息。

#### 1.3 奇异值分解 (SVD)

**核心概念**：任意矩阵 `A = UΣV^T`。SVD 将矩阵分解为旋转(U) × 缩放(Σ) × 旋转(V^T)。

**项目中的应用 — SLAM 优化**：
SLAM Toolbox 使用 Ceres Solver 后端，其内部使用 SVD 求解超定线性方程组：
```
J^T J Δx = -J^T e
```
其中 J 是雅可比矩阵，通过 SVD 分解求解 Δx（状态更新量）。

---

### 2. 概率论与统计

#### 2.1 贝叶斯定理

**核心概念**：`P(A|B) = P(B|A)P(A) / P(B)`

后验概率 ∝ 似然 × 先验概率

**项目中的应用 — AMCL 定位** (nav2_params.yaml)：
```yaml
amcl:
  ros__parameters:
    laser_model_type: "likelihood_field"  # 似然场模型
    max_particles: 2000                    # 粒子数量
    min_particles: 500
```

AMCL (自适应蒙特卡洛定位) 的工作原理：
1. 用 2000 个粒子表示机器人可能的位姿（先验分布）
2. 每个激光扫描更新粒子权重 `P(scan | pose)` （似然函数）
3. 重采样：高权重粒子"繁殖"，低权重粒子"消亡"
4. 粒子云收敛到真实位姿（后验分布）

```python
# 检测置信度本质是后验概率 (defect_detector.py)
confidence = model_output[4]  # YOLO 输出的 class probability
if confidence < self._conf_thresh:  # P(defect | image) < threshold
    continue  # 忽略低置信度检测
```

#### 2.2 高斯分布与卡尔曼滤波

**核心概念**：高斯分布 `N(μ, σ²)` 完全由均值和方差描述。卡尔曼滤波假设状态和噪声都是高斯的。

**项目中的应用 — 传感器融合**：
```xml
<!-- IMU 噪声建模为高斯分布 (sensors.xacro) -->
<angular_velocity>
  <x>
    <noise type="gaussian">
      <mean>0.0</mean>
      <stddev>2e-4</stddev>   <!-- 标准差 σ = 0.0002 rad/s -->
    </noise>
  </x>
</angular_velocity>
```

传感器噪声建模：`measurement = true_value + N(0, σ²)`。噪声标准差 σ 越小，传感器越精确。

#### 2.3 IoU (交并比)

**核心概念**：`IoU = Area(A∩B) / Area(A∪B)`，衡量两个边界框的重叠程度。

**项目中的应用 — 安全检测** (safety_checker.py)：
```python
@staticmethod
def _compute_iou(bbox_a, bbox_b):
    x1 = max(bbox_a[0], bbox_b[0])
    y1 = max(bbox_a[1], bbox_b[1])
    x2 = min(bbox_a[2], bbox_b[2])
    y2 = min(bbox_a[3], bbox_b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)  # 交集面积
    area_a = (bbox_a[2] - bbox_a[0]) * (bbox_a[3] - bbox_a[1])
    area_b = (bbox_b[2] - bbox_b[0]) * (bbox_b[3] - bbox_b[1])
    union = area_a + area_b - inter + 1e-6      # 并集面积
    return inter / union
```

**数学直觉**：IoU = 0 表示完全不重叠，IoU = 1 表示完全重合。判断安全帽是否在人头上：`IoU(helmet_bbox, person_bbox_upper) > 0.1`。

---

### 3. 最优化方法

#### 3.1 梯度下降

**核心概念**：沿负梯度方向迭代更新参数，最小化损失函数。

**项目中的应用 — YOLO 训练**：
```
Loss = λ_coord * BboxLoss + λ_conf * ConfLoss + λ_cls * ClassLoss

梯度下降更新权重：
w_new = w_old - learning_rate * ∂Loss/∂w
```

本项目使用预训练 YOLOv8 ONNX 模型，训练时已通过梯度下降优化了数百万次参数。

#### 3.2 图优化 (Graph Optimization)

**核心概念**：将估计问题建模为图，节点=待优化变量，边=约束/观测。通过最小化所有边的误差和（非线性最小二乘）求解。

**项目中的应用 — SLAM 后端优化**：
```yaml
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver  # Google Ceres 求解器
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY  # 稀疏 Cholesky 分解
    ceres_loss_function: None                    # 损失函数（Huber/Cauchy）
```

SLAM 的图优化问题：
```
min Σ ||observation - prediction(pose, landmark)||²
```

Ceres 使用 Levenberg-Marquardt 算法（介于梯度下降和高斯-牛顿之间）迭代求解。

---

### 4. 坐标变换与刚体运动

#### 4.1 齐次坐标与变换矩阵

**核心概念**：3D 空间中的刚体变换用 4×4 齐次矩阵表示：
```
T = [R  t]    R: 3×3 旋转矩阵（正交）
    [0  1]    t: 3×1 平移向量
```

**项目中的应用 — TF2 变换树**：
机器人各部件通过 TF2 发布坐标变换：
```
map → odom → base_link → chassis → front_camera_link
                            ├→ left_wheel
                            ├→ right_wheel
                            └→ lidar_link
```

```python
# 查询相机在 map 坐标系中的位姿 (waypoint_patrol.py)
self._tf_buffer = Buffer()
self._tf_listener = TransformListener(self._tf_buffer, self)
# 任意时刻可查询: camera_in_map = T_map_base * T_base_camera
```

#### 4.2 旋转表示

| 表示方式 | 参数数量 | 优点 | 缺点 |
|----------|---------|------|------|
| 旋转矩阵 | 9 (3×3) | 直观 | 冗余，需正交约束 |
| 欧拉角 | 3 (roll, pitch, yaw) | 人类可读 | 万向节死锁 |
| **四元数** | 4 (x,y,z,w) | 无死锁，易插值 | 不直观 |

**项目中的应用**：
```python
# 欧拉角 → 四元数 (patrol_cli.py)
yaw = math.pi / 2  # 90度
pose.orientation.z = math.sin(yaw / 2.0)  # qz = sin(θ/2)
pose.orientation.w = math.cos(yaw / 2.0)  # qw = cos(θ/2)
```

#### 4.3 差速驱动运动学

**核心概念**：差速机器人通过两个独立驱动的轮子实现运动。

```
线速度: v = (v_left + v_right) / 2
角速度: ω = (v_right - v_left) / wheel_separation
```

**项目中的应用 — URDF 配置** (inspection_robot.urdf.xacro)：
```xml
<wheel_separation>0.5</wheel_separation>   <!-- 轮距 = 两轮间距 -->
<wheel_diameter>0.16</wheel_diameter>      <!-- 轮径 = 0.08m × 2 -->
```

**逆解**（从期望速度到轮速）：
```
v_left  = v - ω * wheel_separation / 2
v_right = v + ω * wheel_separation / 2
```

例如：期望前进 0.3 m/s 同时左转 0.5 rad/s：
```
v_left  = 0.3 - 0.5 × 0.25 = 0.175 m/s
v_right = 0.3 + 0.5 × 0.25 = 0.425 m/s  （右轮更快 → 左转）
```

---

## 第二部分：图像处理基础

### 5. 数字图像基础

#### 5.1 图像表示

**核心概念**：
- **像素**：图像最小单元，值为 0-255 (uint8) 或 0.0-1.0 (float32)
- **通道**：
  - 灰度图：1 通道，shape = (H, W)
  - RGB/BGR：3 通道，shape = (H, W, 3)
  - OpenCV 使用 BGR 顺序（历史原因），其他库一般用 RGB

**项目中的应用** (preprocessor.py)：
```python
# BGR → RGB 转换
if self._color_order == 'RGB':
    letterbox = cv2.cvtColor(letterbox, cv2.COLOR_BGR2RGB)
```

#### 5.2 色彩空间

| 色彩空间 | 通道含义 | 用途 |
|----------|---------|------|
| BGR | Blue, Green, Red | OpenCV 默认 |
| RGB | Red, Green, Blue | 深度学习模型输入 |
| HSV | Hue, Saturation, Value | 颜色分割 |
| LAB | Luminance, A, B | **工业检测增强** |

**项目中的应用 — CLAHE 增强** (preprocessor.py)：
```python
# 在 LAB 色彩空间的 L 通道上做 CLAHE，不影响颜色
lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
l, a, b = cv2.split(lab)
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
l = clahe.apply(l)           # 仅增强亮度通道
enhanced = cv2.merge([l, a, b])
enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
```

**为什么用 CLAHE？** 工业生产线上光照不均，CLAHE 可以增强暗部缺陷的可见性，同时不引入颜色失真。

---

### 6. 图像预处理

#### 6.1 缩放与 Letterbox

**核心概念**：深度学习模型要求固定输入尺寸（如 640×640），但原始图像比例各异。Letterbox 在保持宽高比的同时填充灰边。

**项目中的应用** (preprocessor.py)：
```python
# 计算缩放比例（取较小的，保证图像不超出目标尺寸）
scale = min(tw / w, th / h)       # 例如：640/800 = 0.8
new_w, new_h = int(w * scale), int(h * scale)  # 640×480 → 512×384

# 填充至目标尺寸
pad_x = (tw - new_w) / 2  # (640 - 512) / 2 = 64
pad_y = (th - new_h) / 2  # (640 - 384) / 2 = 128
letterbox = np.full((th, tw, 3), 114, dtype=np.uint8)  # 灰色背景
letterbox[128:128+384, 64:64+512] = resized
```

```
原始图像 (800×600)          Letterbox 输出 (640×640)
┌──────────────┐            ┌──────────────────┐
│              │            │██████████████████│ ← 灰色填充
│   产品照片    │    →      │██┌──────────┐███│
│              │            │██│  产品照片  │███│
│              │            │██│ (保持比例) │███│
└──────────────┘            │██└──────────┘███│
                            │██████████████████│
                            └──────────────────┘
```

#### 6.2 归一化 (Normalization)

**核心概念**：将像素值从 [0, 255] 映射到 [0, 1]，再标准化为均值 0、标准差 1。

**项目中的应用** (preprocessor.py)：
```python
# ImageNet 标准归一化参数
MEAN  = [0.485, 0.456, 0.406]   # 百万张自然图像的平均像素值
STD   = [0.229, 0.224, 0.225]   # 百万张自然图像的标准差

tensor = image / 255.0                    # [0, 255] → [0, 1]
tensor = (tensor - MEAN) / STD            # 标准化
# 结果：每通道 ≈ N(0, 1) 分布
```

**为什么标准化？** 深度神经网络对输入尺度敏感。标准化后各特征在同一数值范围，梯度下降更稳定、收敛更快。

---

### 7. 边缘检测与特征提取

#### 7.1 Canny 边缘检测

**步骤**：
1. 高斯滤波去噪
2. 计算梯度幅值和方向
3. 非极大值抑制（细化边缘）
4. 双阈值连接（高阈值定边缘，低阈值连边缘）

**项目中的应用 — 仪表指针检测** (meter_reader.py)：
```python
edges = cv2.Canny(gray, 50, 150)  # 低阈值=50, 高阈值=150
lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=min(w, h) // 2)
```

#### 7.2 霍夫变换 (Hough Transform)

**核心概念**：将图像空间的点变换到参数空间。一条直线在极坐标中表示为 `ρ = x·cosθ + y·sinθ`。

**直线检测**：
- 图像空间中每条直线 → 参数空间中的一个点 (ρ, θ)
- 参数空间中投票最多的点 → 图像中最显著的直线

**项目中的应用 — 模拟仪表指针** (meter_reader.py)：
```python
# 检测指针（最长的过中心的直线）
lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=min(w, h)//2)
for line in lines:
    rho, theta = line[0]
    # 计算直线到图像中心的距离
    dist_to_center = abs((a * cx + b * cy) - rho)
    # 选择离中心最近的最长直线 → 指针
```

#### 7.3 模板匹配

**核心概念**：在目标图像中滑动模板图像，计算相似度，找到最佳匹配位置。

```python
# 相关匹配：计算模板与图像每个位置的相似度
result = cv2.matchTemplate(image, template, cv2.TM_CCOEFF_NORMED)
_, max_val, _, max_loc = cv2.minMaxLoc(result)
# max_loc = 最佳匹配位置, max_val = 相似度 (0-1)
```

---

### 8. 深度学习目标检测

#### 8.1 YOLO (You Only Look Once)

**核心思想**：将检测视为回归问题 — 单次前向传播同时预测边界框和类别。

**YOLOv8 网络结构**：
```
Input (640×640×3)
    │
    ▼
Backbone (CSPDarknet)  ← 特征提取
    │
    ▼
Neck (FPN + PAN)       ← 多尺度特征融合
    │
    ▼
Head (Decoupled)       ← 分类 + 边界框回归
    │
    ▼
Output: [N, 6]         ← [x1, y1, x2, y2, conf, class_id]
```

**项目中的应用 — ONNX 推理** (inference_engine.py)：
```python
# ONNX Runtime 加载 YOLOv8 模型
session = ort.InferenceSession("yolov8n_defect.onnx", providers=['CUDAExecutionProvider'])
input_name = session.get_inputs()[0].name       # 'images'
output_names = [o.name for o in session.get_outputs()]  # ['output0']

# 推理
outputs = session.run(output_names, {input_name: tensor})  # [1, 84, 8400]
# 解析输出：84 = 4(bbox) + 1(conf) + 79(COCO classes)
```

#### 8.2 NMS (非极大值抑制)

**核心概念**：去除重叠的冗余检测框，保留置信度最高的。

```
NMS 算法：
1. 按置信度降序排列所有检测框
2. 选置信度最高的框 A
3. 移除所有与 A 的 IoU > threshold 的框
4. 重复步骤 2-3 直到没有框剩余
```

```yaml
# 检测器配置 (detectors.yaml)
defect_detector:
  iou_threshold: 0.45  # IoU > 0.45 的框被认为是同一目标
```

#### 8.3 ONNX Runtime 原理

**核心概念**：ONNX (Open Neural Network Exchange) 是开放的模型互操作格式。

```
PyTorch 模型 → ONNX 导出 → ONNX Runtime 推理

优势：
- 跨框架：PyTorch/TensorFlow/Keras 都可导出
- 跨硬件：CPU/CUDA/TensorRT/RoCm 统一接口
- 优化：图优化、算子融合、量化
```

**项目中的应用** (inference_engine.py)：
```python
# 自动选择最优执行提供器
providers = []
if 'CUDAExecutionProvider' in ort.get_available_providers():
    providers.append(('CUDAExecutionProvider', {'device_id': 0}))
elif 'TensorrtExecutionProvider' in ort.get_available_providers():
    providers.append(('TensorrtExecutionProvider', {'trt_fp16_enable': True}))
else:
    providers.append('CPUExecutionProvider')
```

---

## 第三部分：软件工程基础

### 9. ROS2 架构原理

#### 9.1 ROS2 vs ROS1

| 特性 | ROS1 | ROS2 |
|------|------|------|
| 通信协议 | 自定义 TCP/UDP | **DDS** (Data Distribution Service) |
| 操作系统 | 仅 Linux | Linux / Windows / macOS |
| 实时性 | 弱 | 强实时（RT 支持） |
| 节点发现 | Master 主节点 | **分布式发现**（无中心） |
| Python 版本 | 2.7 | 3.6+ |
| 构建系统 | catkin | **colcon** + ament |

**为什么选 ROS2？**
- DDS 协议提供 QoS（服务质量）配置
- 无中心节点，鲁棒性更好
- 工业级实时支持
- 活跃的社区和长期支持

#### 9.2 ROS2 核心概念

```
┌────────────────────────────────────────────────────┐
│                    ROS2 计算图                      │
│                                                    │
│  Node (节点)          Topic (话题)                 │
│  ┌──────────┐        ┌──────────┐                 │
│  │ Camera   │──pub──→│ /camera  │──sub──→ Detector │
│  └──────────┘        └──────────┘                 │
│                                                    │
│  Service (服务)       Action (动作)                │
│  ┌──────────┐        ┌──────────────────────┐     │
│  │ Client   │──req──→│ /inspection/start     │     │
│  │          │←─res───│ (同步, 请求-响应)      │     │
│  └──────────┘        └──────────────────────┘     │
│                      ┌──────────────────────┐     │
│                      │ /patrol/execute       │     │
│                      │ (异步, 带反馈和取消)    │     │
│                      └──────────────────────┘     │
└────────────────────────────────────────────────────┘
```

**本项目中的使用**：
- **Topic**：相机图像 → `/camera/rgb` → 检测器订阅
- **Service**：`/inspection/start` → 同步启动检测
- **Action**：`/patrol/execute` → 异步巡航（带进度反馈和取消支持）

#### 9.3 DDS 与 QoS

**QoS (Quality of Service) 策略**：

| 策略 | 含义 | 本项目使用 |
|------|------|-----------|
| RELIABILITY | RELIABLE（保证送达）/ BEST_EFFORT | 检测器用 RELIABLE |
| DURABILITY | VOLATILE / TRANSIENT_LOCAL | 默认 VOLATILE |
| HISTORY | KEEP_LAST / KEEP_ALL | 图像用 KEEP_LAST(10) |
| DEPTH | 队列深度 | 相机图像=10 |

**为什么工业检测用 RELIABLE？** 缺陷检测不能丢帧，否则可能漏检关键缺陷。

#### 9.4 colcon 构建系统

```bash
colcon build --symlink-install  # --symlink-install: Python 文件修改后无需重新编译
colcon build --packages-select robot_navigation  # 只编译指定包
colcon build --cmake-clean-first  # 清理后重新编译
colcon test  # 运行所有测试
```

**工作空间层级**：
```
/opt/ros/humble/           ← ROS2 安装空间 (underlay)
    │
simple_detect_robot/       ← 项目工作空间 (overlay)
    ├── src/               ← 源代码
    ├── build/             ← 编译中间文件
    ├── install/           ← 安装目标
    └── log/               ← 编译日志
```

---

### 10. Gazebo 仿真原理

#### 10.1 仿真架构

```
┌─────────────────────────────────────┐
│           Gazebo 仿真引擎            │
│                                     │
│  ┌─────────┐  ┌─────────┐          │
│  │ 物理引擎 │  │ 渲染引擎 │          │
│  │ ODE/     │  │ OGRE    │          │
│  │ Bullet   │  │         │          │
│  └────┬────┘  └────┬────┘          │
│       │            │                │
│  ┌────┴────────────┴────┐          │
│  │    传感器仿真         │          │
│  │  相机/激光/IMU/力矩  │          │
│  └─────────────────────┘          │
└────────────┬────────────────────────┘
             │ ROS 插件
┌────────────┴────────────────────────┐
│        gazebo_ros_pkgs              │
│  ┌──────────┐  ┌────────────────┐  │
│  │ diff_drive│  │ gazebo_ros_    │  │
│  │ plugin    │  │ camera/lidar   │  │
│  └──────────┘  └────────────────┘  │
└─────────────────────────────────────┘
```

#### 10.2 URDF/XACRO 模型描述

**URDF (Unified Robot Description Format)**：XML 格式的机器人模型描述。

```xml
<link name="chassis">           <!-- 连杆：物理实体 -->
  <visual>                      <!-- 可视化几何 -->
    <geometry>
      <box size="0.6 0.4 0.25"/>
    </geometry>
  </visual>
  <collision>                   <!-- 碰撞几何（简化版） -->
    <geometry>
      <box size="0.6 0.4 0.25"/>
    </geometry>
  </collision>
  <inertial>                    <!-- 惯性参数 -->
    <mass value="20.0"/>
    <inertia ixx="0.47" iyy="0.79" izz="0.87" .../>
  </inertial>
</link>

<joint name="left_wheel_joint" type="continuous">  <!-- 关节：连接两个连杆 -->
  <parent link="chassis"/>
  <child link="left_wheel"/>
  <axis xyz="0 1 0"/>           <!-- 旋转轴 -->
</joint>
```

**XACRO 宏**：URDF 的模板语言，避免重复代码。

```xml
<!-- 定义宏 (sensors.xacro) -->
<xacro:macro name="rgbd_camera" params="name parent xyz resolution">
  <joint name="${name}_joint" type="fixed">
    <parent link="${parent}"/>
    <child link="${name}_link"/>
    <origin xyz="${xyz}"/>
  </joint>
  ...
</xacro:macro>

<!-- 使用宏 (inspection_robot.urdf.xacro) -->
<xacro:rgbd_camera name="front_camera" parent="chassis"
  xyz="0.25 0 0.06" resolution="640 480"/>
```

#### 10.3 Gazebo 插件机制

**Gazebo 插件**：动态库（.so），在仿真循环中被调用，实现传感器和控制器仿真。

本项目使用的插件：
| 插件 | 库文件 | 功能 |
|------|--------|------|
| 相机 | `libgazebo_ros_camera.so` | 渲染虚拟相机图像，发布 `/camera/rgb` |
| 激光雷达 | `libgazebo_ros_ray_sensor.so` | 发射虚拟射线，发布 `/lidar/scan` |
| IMU | `libgazebo_ros_imu_sensor.so` | 模拟加速度和角速度数据 |
| 差速驱动 | `libgazebo_ros_diff_drive.so` | 接收 `/cmd_vel`，驱动轮关节，发布 `/odom` |

```xml
<!-- 差速驱动插件配置 (inspection_robot.urdf.xacro) -->
<gazebo>
  <plugin name="gazebo_ros_diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>         <!-- 左轮关节 -->
    <right_joint>right_wheel_joint</right_joint>       <!-- 右轮关节 -->
    <wheel_separation>0.5</wheel_separation>           <!-- 轮距 -->
    <wheel_diameter>0.16</wheel_diameter>              <!-- 轮径 -->
    <publish_odom>true</publish_odom>                  <!-- 发布里程计 -->
    <odometry_frame>odom</odometry_frame>              <!-- 里程计坐标系 -->
  </plugin>
</gazebo>
```

---

### 11. Docker 容器化

#### 11.1 容器 vs 虚拟机

```
虚拟机 (VM):                        容器 (Docker):
┌──────────────────┐                ┌──────────────────┐
│ App A │ App B    │                │ App A │ App B    │
├────────┼─────────┤                ├────────┼─────────┤
│ Guest OS A│Guest B│               │ Docker Engine    │
├────────┼─────────┤                ├──────────────────┤
│ Hypervisor        │                │ Host OS          │
├──────────────────┤                ├──────────────────┤
│ Hardware          │                │ Hardware          │
└──────────────────┘                └──────────────────┘
```

**容器优势**：
- 共享 Host OS 内核，资源开销小
- 镜像分层，快速分发
- 环境一致性（"在我机器上能跑"的终极解决方案）

#### 11.2 Dockerfile 示例（本项目使用）

```dockerfile
FROM osrf/ros:humble-desktop-full
RUN apt update && apt install -y \
    ros-humble-nav2-* ros-humble-slam-toolbox \
    ros-humble-gazebo-ros-pkgs python3-pip
RUN pip3 install opencv-python onnxruntime ultralytics
WORKDIR /workspace
COPY . /workspace/src/simple_detect_robot
RUN colcon build --symlink-install
```

---

### 12. Git 版本管理

#### 12.1 核心工作流

```
Working Directory          Staging Area           Local Repo         Remote Repo
    (工作区)                  (暂存区)              (本地仓库)         (远程仓库)
       │                        │                     │                  │
       │  git add               │  git commit         │  git push        │
       ├───────────────────────→├────────────────────→├─────────────────→│
       │                        │                     │                  │
       │                        │                     │  git fetch/pull  │
       │←───────────────────────┼─────────────────────┼──────────────────┤
```

#### 12.2 常用命令

```bash
# 分支管理
git branch feature/nav2-upgrade       # 创建分支
git checkout -b fix/imu-noise         # 创建并切换
git merge feature/nav2-upgrade        # 合并分支

# 日常操作
git add src/robot_navigation/         # 暂存目录
git commit -m "fix: correct IMU noise parameters"
git push origin main

# 历史查看
git log --oneline --graph             # 图形化日志
git diff HEAD~1                       # 对比上次提交
git show abc1234                      # 查看某次提交

# 撤销
git reset --soft HEAD~1               # 撤销提交（保留修改）
git checkout -- file.txt              # 丢弃工作区修改
```

---

## 第四部分：机器人学基础

### 13. 运动学模型

#### 13.1 差速驱动机器人

**正运动学**（从轮速到位姿变化率）：
```
ẋ = v·cos(θ)      v = (vL + vR) / 2
ẏ = v·sin(θ)      ω = (vR - vL) / L
θ̇ = ω             L = wheel_separation
```

**项目配置**：
```xml
<!-- inspection_robot.urdf.xacro -->
<wheel_separation>0.5</wheel_separation>   <!-- L = 0.5m -->
<wheel_diameter>0.16</wheel_diameter>      <!-- r = 0.08m -->
```

#### 13.2 里程计 (Odometry)

里程计通过累积轮子编码器数据估计机器人位姿：

```
Δt 时间内的位移：
Δs = v · Δt = (vL + vR) · Δt / 2
Δθ = ω · Δt = (vR - vL) · Δt / L

新位姿：
x_new = x + Δs · cos(θ + Δθ/2)
y_new = y + Δs · sin(θ + Δθ/2)
θ_new = θ + Δθ
```

**里程计误差累积**：每步都有微小误差，随时间累积。这就是为什么需要 AMCL 定期用激光数据校正。

---

### 14. SLAM 原理

#### 14.1 SLAM 问题定义

**同时定位与建图** (Simultaneous Localization And Mapping)：

机器人不知道自己在哪（定位问题），也不知道环境长什么样（建图问题），
只能通过自身传感器（激光、里程计）逐步探索。

这是一个"鸡生蛋"问题：
- 要建图，需要知道机器人在哪
- 要知道机器人在哪，需要地图

#### 14.2 SLAM Toolbox 算法

```
传感器数据                    SLAM 处理流程
┌──────────┐                ┌──────────────┐
│ 激光扫描  │───────────────→│ 扫描匹配      │──→ 位姿图添加约束
├──────────┤                │ (Scan Match)  │
│ 里程计    │───────────────→│               │
├──────────┤                ├──────────────┤
│ IMU      │───────────────→│ 回环检测      │──→ 纠正累积误差
│          │                │ (Loop Closure)│
└──────────┘                ├──────────────┤
                            │ 图优化        │──→ 最优位姿 + 地图
                            │ (Pose Graph)  │
                            └──────────────┘
```

**回环检测** (Loop Closure)：当机器人回到之前访问过的位置，SLAM 识别出"这里我见过"，然后调整整个轨迹消除累积误差。

```yaml
# slam_toolbox_params.yaml
loop_search_maximum_distance: 3.0        # 回环搜索半径 (m)
loop_match_minimum_chain_size: 10        # 回环最小激光帧数
do_loop_closing: true                    # 启用回环检测
```

#### 14.3 栅格地图

```python
# Occupancy Grid 占用栅格地图
# 每个栅格存储：0=空闲, 100=占用, -1=未知
resolution: 0.05  # 每个栅格 5cm × 5cm
# 10m × 10m 的地图 = 200 × 200 = 40,000 个栅格
```

**为什么用栅格而不是连续地图？** 栅格化后路径规划变成图搜索问题（A*/Dijkstra），计算高效。

---

### 15. 路径规划

#### 15.1 全局规划 (Global Planner)

**A* 算法**（Nav2 SMAC Planner 使用）：

```
A* 核心思想：
f(n) = g(n) + h(n)

g(n): 从起点到节点 n 的实际代价
h(n): 从节点 n 到终点的启发式估计（直线距离）
f(n): 通过 n 的总估计代价

每次扩展 f(n) 最小的节点 → 保证最优路径
```

```yaml
# Nav2 全局规划器配置
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"  # 混合 A*
      tolerance: 0.5              # 目标容差
      allow_unknown: true         # 允许穿越未知区域
      max_planning_time: 5.0      # 最大规划时间
      minimum_turning_radius: 0.2 # 最小转弯半径
```

#### 15.2 局部规划 (Local Planner)

**DWB (Dynamic Window Approach) + 动态窗口**：

机器人在速度空间中搜索：
1. 考虑所有可能的 (v, ω) 组合
2. 排除会导致碰撞的
3. 对剩余的评分（朝向目标、避障、速度等）
4. 选择最优 (v, ω)

```yaml
# Nav2 控制器配置
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "nav2_dwb_controller/DwbController"
      max_vel_x: 0.5              # 最大线速度
      max_vel_theta: 1.0          # 最大角速度
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle",
                "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```

**Critic 含义**：
| Critic | 功能 | 权重 |
|--------|------|------|
| GoalDist | 到目标距离 | 24 |
| PathDist | 偏离全局路径距离 | 32 |
| GoalAlign | 朝向目标对齐 | 24 |
| PathAlign | 朝向路径对齐 | 32 |
| RotateToGoal | 旋转朝向目标 | 32 |
| BaseObstacle | 障碍物避让 | 0.02 |
| Oscillation | 防止振荡 | 默认 |

#### 15.3 代价地图 (Costmap)

```
代价地图分层：
┌──────────────────────────────┐
│      Master Costmap          │  ← 各层叠加
│  ┌────────────────────────┐  │
│  │   Inflation Layer      │  │  ← 障碍物周围膨胀
│  │  ┌──────────────────┐  │  │
│  │  │ Obstacle Layer   │  │  │  ← 激光检测的障碍物
│  │  │ ┌──────────────┐ │  │  │
│  │  │ │ Static Layer  │ │  │  │  ← 已知地图（墙等）
│  │  │ └──────────────┘ │  │  │
│  │  └──────────────────┘  │  │
│  └────────────────────────┘  │
└──────────────────────────────┘

代价映射：
- 0 (FREE): 可以安全通过
- 1-252: 代价递增
- 253-254: 近致命障碍
- 255 (LETHAL): 致命障碍（碰撞）
```

---

## 第五部分：大模型应用

### 16. LLM API 调用原理

#### 16.1 OpenAI 兼容 API

**标准 HTTP 请求**：
```json
POST /v1/chat/completions
{
  "model": "qwen-plus",
  "messages": [
    {"role": "system", "content": "你是工业巡检专家"},
    {"role": "user", "content": "设备压力异常怎么处理？"}
  ],
  "max_tokens": 4096,
  "temperature": 0.1          // 0=确定性, 1=创造性
}
```

**响应格式**：
```json
{
  "choices": [{
    "message": {"role": "assistant", "content": "根据工业标准..."},
    "finish_reason": "stop"
  }],
  "usage": {"total_tokens": 350}
}
```

**项目中的应用** (qwen_provider.py)：
```python
# OpenAI SDK 封装
self._client = AsyncOpenAI(api_key=api_key, base_url=api_base)
response = await self._client.chat.completions.create(
    model="qwen-plus",
    messages=[{"role": "user", "content": query}],
    max_tokens=4096,
    temperature=0.1,  # 低温度 → 更专业、更确定的回答
)
```

#### 16.2 Temperature 参数

```
Temperature = 0.1 (本项目使用):
  - 每次相同输入 → 几乎相同输出
  - 适合：工业诊断、技术报告、标准回答

Temperature = 1.0:
  - 更多"创造性"
  - 适合：创意写作、头脑风暴

Temperature = 0.0:
  - 完全确定性（贪婪解码）
  - 适合：代码生成、数学计算
```

#### 16.3 Token 与计费

```
Token ≈ 0.75 个英文单词 ≈ 0.5 个中文字

"设备压力异常怎么处理？" = ~10 tokens

计费方式（以千问为例）：
- 输入: ¥0.0008/1K tokens
- 输出: ¥0.002/1K tokens
- 一次典型巡检问答: < ¥0.01
```

---

### 17. RAG 检索增强生成

#### 17.1 RAG 架构

```
┌────────────────────────────────────────────┐
│              RAG 流程                       │
│                                            │
│  用户问题: "安全帽有什么标准？"              │
│       │                                    │
│       ▼                                    │
│  ┌──────────┐     ┌──────────────┐        │
│  │ 向量化    │────→│ 知识库搜索    │        │
│  │ Embedding│     │ Top-K 检索   │        │
│  └──────────┘     └──────┬───────┘        │
│                          │                 │
│                          ▼                 │
│  ┌──────────────────────────────────┐     │
│  │ 检索到的知识:                      │     │
│  │ "安全帽需符合GB 2811标准，         │     │
│  │  帽壳完整无裂纹，下颚带系紧..."    │     │
│  └──────────────┬───────────────────┘     │
│                 │                          │
│                 ▼                          │
│  ┌──────────────────────────────────┐     │
│  │ Prompt 组装 → LLM 生成            │     │
│  │ "请基于以下知识回答用户问题..."    │     │
│  └──────────────────────────────────┘     │
└────────────────────────────────────────────┘
```

**项目中的应用** (industrial_kb.py)：
```python
def build_context(self, query, max_chunks=3):
    results = self.search(query, top_k=max_chunks)
    # 组装 RAG 上下文注入 Prompt
    parts = ['## 相关工业知识 (来自知识库)']
    for r in results:
        parts.append(f"### {r['title']}\n内容: {r['content']}")
    return '\n\n'.join(parts)
```

#### 17.2 文本嵌入 (Embedding)

**为什么用向量搜索？** 关键词匹配 ≠ 语义理解。

```
用户查询: "安全帽佩戴有什么要求？"
关键词匹配: 找到 "安全帽" ✓
语义搜索: 还找到 "头盔规范" ✓ (语义相似但关键词不同)
```

**Sentence-Transformers 模型** (paraphrase-multilingual-MiniLM-L12-v2)：
- 输入：文本
- 输出：384 维向量
- 语义相似的文本 → 向量距离近（余弦相似度 > 0.7）
- 支持中英文混合检索

```python
# 向量化 + 相似度搜索
query_embedding = model.encode(["安全帽佩戴要求"])    # [1, 384]
scores = np.dot(knowledge_embeddings, query_embedding.T)  # [N, 1]
# scores[i] 越大 → 第 i 条知识越相关
```

#### 17.3 ChromaDB

**ChromaDB**：嵌入式向量数据库，无需单独部署服务器。

```python
client = chromadb.PersistentClient(path="~/.inspection_robot/knowledge")
collection = client.get_or_create_collection("industrial_knowledge")

# 添加知识
collection.add(
    ids=["doc_1"],
    documents=["安全帽需符合GB 2811标准..."],
    metadatas=[{"category": "safety_standards", "title": "安全帽规范"}],
)

# 查询
results = collection.query(query_texts=["安全帽标准"], n_results=5)
```

---

### 18. 多模态模型

#### 18.1 多模态架构

```
传统 LLM:                    多模态 LLM (Qwen-VL):
┌─────────┐                 ┌─────────┐  ┌─────────┐
│ Text    │→ LLM → Text    │ Text    │  │ Image   │
└─────────┘                 │         │  │         │
                            └────┬────┘  └────┬────┘
                                 │   Vision    │
                                 │   Encoder   │
                                 └──────┬──────┘
                                        │
                                   Cross-Attention
                                        │
                                        ▼
                                      LLM → Text
```

**项目中的应用** (multimodal_analyzer.py)：
```python
async def analyze_defect(self, image_bytes, detection_result, question):
    prompt = (
        f'{question}\n'
        f'【检测数据】\n'
        f'- 缺陷类型: {detection_result["defect_type"]}\n'
        f'- 严重程度: {detection_result["severity"]}\n'
        f'- 置信度: {detection_result["confidence"]:.1%}'
    )
    return await provider.chat_with_image(
        prompt=prompt, images=[image_bytes])
```

#### 18.2 图像编码方式

```python
# Qwen-VL: Base64 Data URI (qwen_provider.py)
b64 = base64.b64encode(image_bytes).decode('utf-8')
image_uri = f'data:image/jpeg;base64,{b64}'

# API 消息格式
{
    "role": "user",
    "content": [
        {"type": "image_url", "image_url": {"url": image_uri}},
        {"type": "text", "text": "这个缺陷严重吗？"}
    ]
}
```

---

## 第六部分：工程整合实践

### 19. 本项目端到端数据流

```
┌─────────────────────────────────────────────────────────────────┐
│  1. 用户启动仿真                                                 │
│     ros2 launch robot_bringup full_inspection_demo.launch.py    │
└──────────────────────────┬──────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────┐
│  2. Gazebo 仿真引擎启动                                          │
│     - 加载 industrial_factory.world                             │
│     - 启动物理引擎 (ODE) + 渲染引擎                               │
│     - 生成机器人 (spawn_entity.py)                               │
│     - 启动传感器插件 (相机/激光/IMU)                              │
└──────────────────────────┬──────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────┐
│  3. Navigation Stack (Nav2)                                     │
│     - SLAM Toolbox: 异步建图                                     │
│     - AMCL: 粒子滤波定位                                         │
│     - Planner Server: 全局路径规划                               │
│     - Controller Server: 局部速度控制                            │
│     - Lifecycle Manager: 自动激活所有节点                         │
└──────────────────────────┬──────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────┐
│  4. Patrol Mission (巡航任务)                                    │
│     Action Client → /patrol/execute Action Server               │
│     → WaypointPatrol → Nav2 NavigateToPose                      │
│     → 到达 WP1 → 触发 Inspection → 停留 5s → 下一 WP            │
└──────────────────────────┬──────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────┐
│  5. Visual Inspection (视觉检测)                                 │
│     /camera/rgb → DefectDetector / MeterReader / SafetyChecker  │
│     → ONNX Runtime 推理 / Mock 模拟                              │
│     → /inspection/defect/result + /inspection/alert             │
└──────────────────────────┬──────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────┐
│  6. LLM Analysis & Report (智能分析)                             │
│     Inspection Results → MultimodalAnalyzer                     │
│     → LLM 分析 (Qwen/DeepSeek/Ernie)                            │
│     → RAG 知识库增强                                             │
│     → ReportGenerator → Markdown 报告                            │
└─────────────────────────────────────────────────────────────────┘
```

### 20. 项目关键技术决策总结

| 决策 | 选择 | 核心理由 |
|------|------|----------|
| ROS2 版本 | Humble | Ubuntu 22.04 LTS 官方支持 |
| 仿真平台 | Gazebo 11 Classic | 已预装，ROS2 Humble 原生集成 |
| SLAM | SLAM Toolbox | 异步建图，轻量级，支持回环检测 |
| 导航 | Nav2 | ROS2 标准导航栈，Behavior Tree 定制 |
| 检测框架 | YOLOv8 → ONNX | 行业标准，推理快速，跨平台部署 |
| OCR | PaddleOCR | 中文识别最优版，支持多语言 |
| LLM 协议 | OpenAI 兼容格式 | 千问/DeepSeek 均支持，减少适配代码 |
| RAG 数据库 | ChromaDB | 嵌入式，零运维，向量+过滤双检索 |
| 嵌入模型 | MiniLM-L12-v2 | 轻量(118M参数)，支持中英，本地运行 |
| 构建系统 | colcon | ROS2 官方，支持 ament_cmake/ament_python |

### 21. 调试技巧

```bash
# 1. 查看话题数据
ros2 topic echo /camera/rgb --once          # 查看一次相机数据
ros2 topic hz /lidar/scan                   # 查看话题发布频率
ros2 topic info /inspection/result -v       # 查看话题详细信息（含订阅者）

# 2. 查看节点图
ros2 run rqt_graph rqt_graph                # 图形化节点图
rqt                                        # 综合调试工具

# 3. 查看 TF 变换树
ros2 run tf2_tools view_frames              # 生成 PDF 变换树
ros2 run tf2_ros tf2_echo base_link camera_link  # 查询实时变换

# 4. 调试单个节点
ros2 run robot_inspection defect_detector --ros-args \
  -p enable_mock:=true \
  --log-level debug                          # 开启 DEBUG 日志

# 5. 录制和回放数据
ros2 bag record /camera/rgb /lidar/scan     # 录制话题数据
ros2 bag play rosbag2_2026_06_02/           # 回放
```

---

## 推荐学习路径

### 初学者 (0 → 1)

1. Python 基础 → OpenCV 图像处理
2. ROS2 基础 → Topic/Service/Action
3. Gazebo 基础 → 添加模型/插件
4. Git 基础 → 克隆/提交/推送

### 进阶者 (1 → 10)

1. 线性代数 + 坐标变换
2. SLAM 原理 → 参数调优
3. 深度学习 → YOLO 训练/ONNX 部署
4. LLM API → RAG 架构设计

### 深入者 (10 → 100)

1. 多传感器融合（EKF/PF）
2. 导航栈源码分析
3. 自定义 Gazebo 插件开发
4. 模型量化/剪枝/知识蒸馏
5. 数字孪生 Web 可视化

---

> **文档版本**: v1.0 | **作者**: brandon407 | **项目**: simple_detect_robot
>
> 本教学文档随项目持续更新。如有疑问或建议，欢迎提交 Issue。
