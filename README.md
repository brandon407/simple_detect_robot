# 工业机器人智能巡检系统 Industrial Inspection Robot

[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python)](https://www.python.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-11 Classic-FF6600)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

基于 ROS2 的工业机器人智能巡检系统，集成**自主导航巡航**、**工业视觉检测**、**大模型智能问答**三大核心能力，全部功能可在 Gazebo 仿真环境中验证。

---

## 目录

- [系统架构](#系统架构)
- [项目结构](#项目结构)
- [环境要求](#环境要求)
- [快速开始](#快速开始)
- [使用指南](#使用指南)
- [配置说明](#配置说明)
- [API 参考](#api-参考)
- [开发阶段](#开发阶段)
- [常见问题](#常见问题)

---

## 系统架构

```
┌────────────────────────────────────────────────────────────────┐
│                    仿真验证层 (Gazebo)                          │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────┐  │
│  │ 工业场景  │  │ 机器人模型│  │ 传感器   │  │ 检测目标模型  │  │
│  └──────────┘  └──────────┘  └──────────┘  └──────────────┘  │
└────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌────────────────────────────────────────────────────────────────┐
│                      ROS2 功能层                                │
│                                                                │
│  ┌────────────┐  ┌────────────┐  ┌──────────────────────┐    │
│  │ 导航巡航    │  │ 视觉检测    │  │ 智能问答 (LLM)       │    │
│  │ SLAM+Nav2  │  │ 缺陷/仪表/  │  │ 千问/文心/DeepSeek  │    │
│  │ 路径巡航    │  │ 安全合规    │  │ RAG + 报告生成      │    │
│  └────────────┘  └────────────┘  └──────────────────────┘    │
│                                                                │
│  ┌────────────┐  ┌────────────┐  ┌──────────────────────┐    │
│  │ 机器人控制  │  │ 传感器融合  │  │ 数据管理              │    │
│  │ 差速驱动    │  │ 激光/相机/  │  │ 巡检记录/检测结果/   │    │
│  │ 状态估计    │  │ IMU        │  │ 日志回放              │    │
│  └────────────┘  └────────────┘  └──────────────────────┘    │
└────────────────────────────────────────────────────────────────┘
```

### ROS2 节点图

```
                     ┌──────────────────┐
                     │   /patrol_server  │  ← 巡航任务 (Action)
                     └──────┬───────────┘
                            │
        ┌───────────────────┼───────────────────┐
        ▼                   ▼                   ▼
┌───────────────┐  ┌───────────────┐  ┌─────────────────┐
│  Nav2 导航栈   │  │ /inspection   │  │ /llm_agent      │
│  SLAM Toolbox │  │  _orchestrator│  │ 智能问答+报告    │
│  Planner      │  │ 检测编排       │  │                 │
│  Controller   │  └───┬───┬───┬──┘  └─────────────────┘
└───────────────┘      │   │   │
                       ▼   ▼   ▼
              ┌────────────────────────┐
              │     检测器插件          │
              ├────────┬───────┬───────┤
              │ 缺陷   │ 仪表  │ 安全  │
              │ 检测   │ 读取  │ 检查  │
              └────────┴───────┴───────┘
```

---

## 项目结构

```
simple_detect_robot/
├── src/
│   ├── inspection_msgs/           # 自定义 ROS2 消息接口
│   │   ├── msg/                   # InspectionResult, DefectDetection,
│   │   │                          # MeterReading, SafetyCheck, Alert
│   │   ├── srv/                   # StartInspection, LLMQuery
│   │   └── action/                # PatrolMission
│   │
│   ├── robot_description/         # 机器人 URDF/XACRO 模型
│   │   └── urdf/                  # 差分驱动底盘 + 相机/激光/IMU
│   │
│   ├── robot_bringup/             # 启动配置与参数
│   │   ├── launch/                # simulation_bringup, full_inspection_demo
│   │   └── config/                # Nav2, SLAM Toolbox, Controller, Sensors
│   │
│   ├── robot_control/             # 机器人控制 (C++)
│   │   └── src/                   # diff_drive_controller
│   │
│   ├── robot_navigation/          # 导航与巡航 (Python)
│   │   └── robot_navigation/      # patrol_server, waypoint_patrol,
│   │                              # slam_manager, navigation_manager,
│   │                              # patrol_cli, inspection_demo
│   │
│   ├── robot_inspection/          # 视觉检测管线 (Python)
│   │   └── robot_inspection/
│   │       ├── detectors/         # defect_detector, meter_reader,
│   │       │                      # safety_checker, base_detector
│   │       ├── models/            # inference_engine, model_manager
│   │       └── utils/             # preprocessor, visualization
│   │
│   ├── llm_agent/                 # 大模型智能问答 (Python)
│   │   └── llm_agent/
│   │       ├── providers/         # qwen, ernie, deepseek providers
│   │       ├── industrial_kb.py   # RAG 工业知识库
│   │       ├── multimodal_analyzer.py
│   │       └── report_generator.py
│   │
│   └── inspection_simulation/     # Gazebo 仿真场景
│       ├── worlds/                # industrial_factory.world
│       └── models/                # 自定义检测目标模型
│
├── DESIGN_PLAN.md                 # 完整架构设计文档
├── KNOWLEDGE_GUIDE.md             # 知识教学文档
└── README.md                      # 本文件
```

---

## 环境要求

| 组件 | 版本 | 说明 |
|------|------|------|
| **操作系统** | Ubuntu 22.04 (WSL2) | 推荐使用 WSL2 |
| **ROS2** | Humble Hawksbill | LTS，支持至 2027 |
| **Gazebo** | 11 Classic | 经典版，已预装 |
| **Python** | 3.10+ | 系统自带 |
| **CUDA** | 12.x | NVIDIA RTX 5060 8GB |
| **Git** | 2.x | 版本管理 |

### 依赖安装

```bash
# ROS2 Humble (Ubuntu 22.04)
sudo apt update
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs

# 导航与 SLAM
sudo apt install ros-humble-nav2-* ros-humble-slam-toolbox

# 控制
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# Python 依赖
pip3 install opencv-python numpy onnxruntime paddleocr ultralytics chromadb

# LLM SDK (按需)
pip3 install openai qianfan sentence-transformers
```

---

## 快速开始

### 1. 克隆项目

```bash
git clone git@github.com:brandon407/simple_detect_robot.git
cd simple_detect_robot
```

### 2. 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. 启动基础仿真

```bash
# 启动 Gazebo + 机器人 + Nav2 + SLAM + 巡逻
ros2 launch robot_bringup simulation_bringup.launch.py
```

### 4. 启动全系统演示

```bash
# 启动所有组件：仿真 + 导航 + 检测 + LLM
ros2 launch robot_bringup full_inspection_demo.launch.py
```

### 5. 运行自动化巡检演示

```bash
# 在另一个终端中，启动演示脚本
source install/setup.bash
ros2 run robot_navigation inspection_demo
```

演示脚本将：
1. 等待系统就绪
2. 发送 3 个巡检点（产线缺陷检测 → 设备仪表读取 → 安全合规检查）
3. 实时显示巡检进度
4. 收集检测结果
5. 生成 Markdown 巡检报告 → `/tmp/inspection_reports/`

---

## 使用指南

### 手动控制

```bash
# 键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 发送巡航任务
ros2 run robot_navigation patrol_cli --ros-args \
  -p "waypoints:=[[4.0,-2.5,0],[-3.5,0,0],[4.0,2.5,3.14]]"
```

### 视觉检测

```bash
# 启动单个检测器
ros2 run robot_inspection defect_detector --ros-args -p enable_mock:=true
ros2 run robot_inspection meter_reader --ros-args -p enable_mock:=true
ros2 run robot_inspection safety_checker --ros-args -p enable_mock:=true

# 启动检测编排器
ros2 run robot_inspection inspection_orchestrator

# 手动触发检测
ros2 service call /inspection/start std_srvs/srv/Trigger

# 查看检测结果
ros2 topic echo /inspection/defect/result
ros2 topic echo /inspection/alert
```

### LLM 智能问答

```bash
# 启动 LLM 节点
ros2 run llm_agent llm_node

# 发送问答请求
ros2 service call /llm/query inspection_msgs/srv/LLMQuery \
  "{query: '设备压力异常怎么处理？', context: '', images: []}"
```

### SLAM 建图

```bash
# 保存当前地图
ros2 service call /slam/save_map std_srvs/srv/Trigger

# 地图保存在 ~/.inspection_robot/maps/
```

---

## 配置说明

### LLM API 密钥

```bash
# 通义千问 (推荐，OpenAI 兼容)
export QWEN_API_KEY="your-dashscope-api-key"

# DeepSeek (OpenAI 兼容)
export DEEPSEEK_API_KEY="your-deepseek-api-key"

# 文心一言
export ERNIE_API_KEY="your-qianfan-ak"
export ERNIE_SECRET_KEY="your-qianfan-sk"
```

未配置 API 密钥时，系统自动使用 **mock 模式**，返回模拟的专业回答。

### 检测器参数

编辑 `src/robot_inspection/config/detectors.yaml`：

```yaml
defect_detector:
  confidence_threshold: 0.5    # 检测置信度阈值
  device: "cuda"               # cuda | cpu | auto
  image_size: [640, 640]       # 模型输入尺寸

safety_checker:
  zone_intrusion:
    enabled: true
    zone_polygon: []            # 受限区域多边形坐标
```

### Nav2 导航参数

编辑 `src/robot_bringup/config/nav2_params.yaml`：

关键参数：
- `max_vel_x: 0.5` — 最大线速度 (m/s)
- `max_vel_theta: 1.0` — 最大角速度 (rad/s)
- `xy_goal_tolerance: 0.25` — 目标点到达容差 (m)
- `inflation_radius: 0.55` — 障碍物膨胀半径 (m)

---

## API 参考

### ROS2 Topics

| Topic | 类型 | 说明 |
|-------|------|------|
| `/camera/rgb` | `sensor_msgs/Image` | RGB 相机图像 |
| `/lidar/scan` | `sensor_msgs/LaserScan` | 激光雷达扫描 |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度控制指令 |
| `/odom` | `nav_msgs/Odometry` | 里程计 |
| `/inspection/result` | `InspectionResult` | 综合检测结果 |
| `/inspection/defect/result` | `DefectDetection` | 缺陷检测结果 |
| `/inspection/meter/result` | `MeterReading` | 仪表读数结果 |
| `/inspection/safety/result` | `SafetyCheck` | 安全检测结果 |
| `/inspection/alert` | `Alert` | 告警信息 |

### ROS2 Services

| Service | 类型 | 说明 |
|---------|------|------|
| `/inspection/start` | `std_srvs/Trigger` | 启动检测 |
| `/inspection/stop` | `std_srvs/Trigger` | 停止检测 |
| `/inspection/status` | `std_srvs/Trigger` | 查询检测状态 |
| `/llm/query` | `LLMQuery` | LLM 问答 |
| `/slam/save_map` | `std_srvs/Trigger` | 保存地图 |

### ROS2 Actions

| Action | 类型 | 说明 |
|--------|------|------|
| `/patrol/execute` | `PatrolMission` | 巡航任务 |

### PatrolMission 格式

```json
{
  "waypoints": [
    {"position": {"x": 4.0, "y": -2.5, "z": 0.0}, "orientation": {"w": 1.0}},
    {"position": {"x": -3.5, "y": 0.0, "z": 0.0}, "orientation": {"w": 1.0}}
  ],
  "loop_mode": false,
  "stay_duration": 5.0,
  "inspection_modes": ["defect", "meter"]
}
```

---

## 开发阶段

| Phase | 内容 | 代码量 | 状态 |
|-------|------|--------|------|
| 1 | 项目骨架 + 机器人模型 + 仿真环境 | ~2300 行 | ✅ |
| 2 | 导航与巡航 (SLAM + Nav2 + Patrol) | ~900 行 | ✅ |
| 3 | 视觉检测管线 (3 检测器 + ONNX) | ~1800 行 | ✅ |
| 4 | 大模型智能问答 (3 LLM + RAG) | ~1500 行 | ✅ |
| 5 | 全系统集成 + 自动化演示 | ~500 行 | ✅ |
| **总计** | **8 Packages, ~7000 行代码** | | **✅** |

---

## 常见问题

### Q: Gazebo 无法启动？

```bash
# 检查 Gazebo 安装
gazebo --version

# 杀死残留进程
killall gzserver gzclient

# 设置环境变量
echo "export SVGA_VGPU10=0" >> ~/.bashrc
```

### Q: 检测器无法加载 ONNX 模型？

系统默认使用 **mock 模式**，无需模型即可运行。将 ONNX 模型放入 `~/.inspection_robot/models/` 目录后自动启用真实推理。

### Q: LLM 问答返回 Mock 回答？

配置 API 密钥后自动切换到真实 API。详见 [配置说明](#配置说明)。

### Q: 如何添加自定义检测器？

继承 `BaseDetector` 基类：

```python
from robot_inspection.detectors.base_detector import BaseDetector

class MyDetector(BaseDetector):
    def initialize(self, config): ...
    def detect(self, image, depth=None): ...
    def get_type(self): return 'my_type'
    def shutdown(self): ...
```

在 `orchestrator.py` 的 `DETECTOR_TOPICS` 中注册即可。

---

## 许可证

Apache-2.0 License

## 贡献

欢迎提交 Issue 和 Pull Request。

## 联系

- **GitHub**: [brandon407/simple_detect_robot](https://github.com/brandon407/simple_detect_robot)
- **ROS2**: Humble Hawksbill
- **仿真**: Gazebo 11 Classic
