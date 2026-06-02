# 工业机器人巡检系统 - 项目规划与架构设计

## Context

构建一个基于 ROS2 的工业机器人智能巡检系统，使用轮式移动机器人平台，集成工业视觉检测、自主导航巡航、大模型智能问答等功能，全部功能可在 Gazebo 仿真环境中验证。

### 当前环境
- **OS**: Ubuntu 22.04 (WSL)
- **ROS2**: Humble Hawksbill
- **Gazebo**: 11.10.2 (Classic)
- **Python**: 3.10.12
- **GPU**: NVIDIA RTX 5060 8GB, Driver 581.29
- **LLM**: 国内大模型（通义千问/文心一言/DeepSeek）

---

## 一、总体架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        仿真验证层 (Gazebo)                        │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌────────────────┐  │
│  │ 工业场景  │  │ 机器人模型│  │ 传感器   │  │ 检测目标模型    │  │
│  │ (world)  │  │ (URDF)   │  │ 插件     │  │ (缺陷/仪表/安全)│  │
│  └──────────┘  └──────────┘  └──────────┘  └────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                       ROS2 功能层                                 │
│                                                                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │ 导航巡航模块  │  │ 视觉检测模块  │  │ 智能问答模块(LLM)    │  │
│  │ - SLAM建图   │  │ - 缺陷检测   │  │ - 多模型API抽象      │  │
│  │ - Nav2导航   │  │ - 仪表读数   │  │ - 工业知识问答      │  │
│  │ - 路径巡航   │  │ - 安全检测   │  │ - 巡检报告生成      │  │
│  │ - 避障       │  │ - 检测编排   │  │ - 多模态分析        │  │
│  └──────────────┘  └──────────────┘  └──────────────────────┘  │
│                                                                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │ 机器人控制    │  │ 传感器融合    │  │ 数据管理模块         │  │
│  │ - 差速驱动   │  │ - 激光雷达   │  │ - 巡检记录存储      │  │
│  │ - 云台控制   │  │ - RGB-D相机  │  │ - 检测结果数据库    │  │
│  │ - 状态估计   │  │ - IMU融合    │  │ - 日志与回放        │  │
│  └──────────────┘  └──────────────┘  └──────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 二、ROS2 节点图与数据流

```
                      ┌──────────────────┐
                      │   /patrol_server  │  ← 巡航任务管理 (Action)
                      │  (巡航编排节点)    │
                      └──────┬───────────┘
                             │
         ┌───────────────────┼───────────────────┐
         │                   │                   │
         ▼                   ▼                   ▼
┌────────────────┐  ┌───────────────┐  ┌─────────────────┐
│ /nav2_planner  │  │ /inspection   │  │ /llm_agent      │
│ (Nav2导航栈)    │  │  _orchestrator│  │ (大模型问答节点) │
│                │  │ (检测编排节点) │  │                  │
│ ┌────────────┐ │  │               │  │ - 问答服务       │
│ │slam_toolbox│ │  │ 启动/停止检测 │  │ - 报告生成       │
│ │/map_server │ │  │ 收集检测结果  │  │ - 多模态分析     │
│ │/planner    │ │  │ 异常告警      │  │                  │
│ │/controller │ │  └───┬───┬───┬──┘  └─────────────────┘
│ │/behavior   │ │      │   │   │
│ └────────────┘ │      ▼   ▼   ▼
└────────────────┘  ┌──────────────────────────────┐
                    │      检测器插件 (Plugin)       │
                    ├──────────┬──────────┬─────────┤
                    │/defect   │/meter    │/safety  │
                    │_detector │_reader   │_checker │
                    │(缺陷检测) │(仪表读取) │(安全检测) │
                    └──────────┴──────────┴─────────┘
                             │
                             ▼
                    ┌─────────────────┐
                    │  /sensor_fusion │  ← 传感器数据融合
                    │  /camera_raw    │
                    │  /lidar_scan    │
                    │  /imu_data      │
                    └─────────────────┘
```

### 核心 Topic 定义

| Topic/Service/Action | 类型 | 说明 |
|---------------------|------|------|
| `/camera/rgb` | `sensor_msgs/Image` | RGB 图像流 |
| `/camera/depth` | `sensor_msgs/Image` | 深度图像流 |
| `/lidar/scan` | `sensor_msgs/LaserScan` | 激光雷达扫描 |
| `/imu/data` | `sensor_msgs/Imu` | IMU 数据 |
| `/cmd_vel` | `geometry_msgs/Twist` | 机器人速度指令 |
| `/odom` | `nav_msgs/Odometry` | 里程计 |
| `/inspection/result` | `inspection_msgs/InspectionResult` | 检测结果 |
| `/inspection/start` | `std_srvs/Trigger` | 启动检测 Service |
| `/patrol/execute` | `patrol_msgs/PatrolMission` (Action) | 巡航任务 Action |
| `/llm/query` | `llm_msgs/LLMQuery` | LLM 问答请求 |
| `/alert` | `inspection_msgs/Alert` | 异常告警 |

---

## 三、ROS2 Package 结构

```
simple_detect_robot/
├── src/
│   ├── robot_description/           # 机器人URDF/XACRO模型
│   │   ├── urdf/
│   │   │   ├── inspection_robot.urdf.xacro
│   │   │   ├── sensors.xacro
│   │   │   └── macros.xacro
│   │   ├── launch/
│   │   │   ├── robot_state_publisher.launch.py
│   │   │   └── description.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── robot_bringup/               # 启动配置
│   │   ├── launch/
│   │   │   ├── robot_bringup.launch.py      # 真实机器人
│   │   │   ├── simulation_bringup.launch.py  # 仿真环境
│   │   │   └── nav2_bringup.launch.py
│   │   ├── config/
│   │   │   ├── nav2_params.yaml
│   │   │   ├── slam_toolbox_params.yaml
│   │   │   ├── controller_params.yaml
│   │   │   └── sensors.yaml
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── robot_control/               # 机器人控制
│   │   ├── src/
│   │   │   ├── diff_drive_controller.cpp/hpp
│   │   │   ├── sensor_fusion.cpp/hpp
│   │   │   └── state_estimator.cpp/hpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── robot_navigation/            # 导航与巡航 (Python)
│   │   ├── robot_navigation/
│   │   │   ├── __init__.py
│   │   │   ├── navigation_manager.py    # Nav2 封装
│   │   │   ├── waypoint_patrol.py       # 路径巡航
│   │   │   ├── patrol_server.py         # 巡航任务Action服务
│   │   │   └── slam_manager.py          # SLAM管理
│   │   ├── launch/
│   │   │   ├── slam.launch.py
│   │   │   ├── navigation.launch.py
│   │   │   └── patrol.launch.py
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── robot_inspection/            # 视觉检测 (Python)
│   │   ├── robot_inspection/
│   │   │   ├── __init__.py
│   │   │   ├── orchestrator.py          # 检测编排主节点
│   │   │   ├── detectors/
│   │   │   │   ├── __init__.py
│   │   │   │   ├── base_detector.py     # 检测器基类 (插件接口)
│   │   │   │   ├── defect_detector.py   # 产品缺陷检测
│   │   │   │   ├── meter_reader.py      # 仪表读数识别
│   │   │   │   └── safety_checker.py    # 安全合规检测
│   │   │   ├── models/                  # 推理模型管理
│   │   │   │   ├── model_manager.py     # 模型加载/卸载
│   │   │   │   └── inference_engine.py  # ONNX Runtime 推理
│   │   │   └── utils/
│   │   │       ├── visualization.py     # 检测结果可视化
│   │   │       └── preprocessor.py      # 图像预处理
│   │   ├── config/
│   │   │   ├── detectors.yaml           # 检测器配置
│   │   │   └── inspection_params.yaml   # 检测参数
│   │   ├── models/                      # 预训练模型存放 (git-ignored)
│   │   ├── launch/
│   │   │   └── inspection.launch.py
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── llm_agent/                    # 大模型智能问答 (Python)
│   │   ├── llm_agent/
│   │   │   ├── __init__.py
│   │   │   ├── llm_node.py              # LLM交互ROS节点
│   │   │   ├── providers/
│   │   │   │   ├── __init__.py
│   │   │   │   ├── base_provider.py     # LLM Provider 抽象基类
│   │   │   │   ├── qwen_provider.py     # 通义千问
│   │   │   │   ├── ernie_provider.py    # 文心一言
│   │   │   │   └── deepseek_provider.py # DeepSeek
│   │   │   ├── report_generator.py      # 巡检报告生成
│   │   │   ├── industrial_kb.py         # 工业知识库管理
│   │   │   └── multimodal_analyzer.py   # 多模态分析(图像+文本)
│   │   ├── config/
│   │   │   ├── llm_config.yaml          # API密钥与模型配置
│   │   │   └── knowledge_base.json      # 工业知识库
│   │   ├── launch/
│   │   │   └── llm_agent.launch.py
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── inspection_msgs/             # 自定义消息接口
│   │   ├── msg/
│   │   │   ├── InspectionResult.msg
│   │   │   ├── DefectDetection.msg
│   │   │   ├── MeterReading.msg
│   │   │   ├── SafetyCheck.msg
│   │   │   └── Alert.msg
│   │   ├── srv/
│   │   │   ├── StartInspection.srv
│   │   │   └── LLMQuery.srv
│   │   ├── action/
│   │   │   └── PatrolMission.action
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── simulation/                   # 仿真场景与模型
│       ├── worlds/
│       │   ├── industrial_factory.world    # 工厂车间场景
│       │   ├── inspection_targets.world    # 检测目标场景
│       │   └── patrol_route.world          # 巡航路线场景
│       ├── models/
│       │   ├── inspection_robot/           # 机器人SDF模型
│       │   ├── conveyor_belt/              # 传送带
│       │   ├── instrument_panel/           # 仪表盘
│       │   ├── safety_zone/                # 安全区域标识
│       │   └── defect_samples/             # 缺陷样本模型
│       ├── launch/
│       │   ├── factory_sim.launch.py       # 工厂仿真启动
│       │   └── spawn_robot.launch.py       # 生成机器人
│       ├── CMakeLists.txt
│       └── package.xml
│
├── docker/                           # Docker 部署 (可选)
│   ├── Dockerfile
│   └── docker-compose.yaml
│
└── docs/                             # 文档
    ├── architecture.md
    ├── api_design.md
    └── user_guide.md
```

---

## 四、各模块详细设计

### 4.1 机器人模型 (robot_description)

**技术选型**: XACRO + URDF, Gazebo 11 兼容
- 差分驱动底盘 (轮距 ~0.5m, 轮径 ~0.15m)
- 传感器配置: RGB-D 相机(前向, 640×480@30Hz) + 2D 激光雷达(360°, 10m范围) + IMU
- 可选云台 (Pan-Tilt) 用于相机姿态调整

**关键文件**:
- `robot_description/urdf/inspection_robot.urdf.xacro` — 主模型
- `robot_description/urdf/sensors.xacro` — 传感器宏定义
- Gazebo 插件: `libgazebo_ros_diff_drive`, `libgazebo_ros_camera`, `libgazebo_ros_ray_sensor`

### 4.2 导航巡航模块 (robot_navigation)

**技术栈**: Nav2 + SLAM Toolbox

| 功能 | 组件 | 说明 |
|------|------|------|
| SLAM 建图 | SLAM Toolbox | 在线异步建图, 支持保存/加载地图 |
| 全局规划 | Nav2 Planner (Smac Hybrid) | 混合 A*/Dijkstra |
| 局部规划 | Nav2 Controller (DWB) | 动态窗口方法 |
| 行为树 | Nav2 Behavior Tree | 巡航逻辑编排 |
| 定位 | AMCL | 自适应蒙特卡洛定位 |

**巡航模式**:
1. **全自主巡航**: 给定巡检点序列，自主规划路径逐一到达
2. **固定路线**: 预录轨迹精准复现
3. **点检模式**: 远程指定目标点，到达后启动检测

**关键接口**:
- Action: `PatrolMission.action` — 巡航任务定义
  ```
  # Goal
  geometry_msgs/Pose[] waypoints
  bool loop_mode            # 是否循环
  float32 stay_duration     # 每点停留时间
  ---
  # Result
  bool success
  ---
  # Feedback
  int32 current_waypoint
  float32 distance_remaining
  ```

### 4.3 视觉检测模块 (robot_inspection)

**插件化架构** — 每种检测能力作为独立插件，可动态加载:

```python
class BaseDetector(ABC):
    """检测器基类 — 所有检测器必须实现此接口"""
    
    @abstractmethod
    def initialize(self, config: dict) -> bool: ...
    
    @abstractmethod
    def detect(self, image: np.ndarray, depth: np.ndarray = None) -> InspectionResult: ...
    
    @abstractmethod
    def get_type(self) -> str: ...  # 'defect' | 'meter' | 'safety'
    
    @abstractmethod
    def shutdown(self) -> None: ...
```

**三合一检测器**:

| 检测器 | 方法 | 模型 | 仿真验证 |
|--------|------|------|----------|
| **DefectDetector** | YOLOv8 + 传统CV | ONNX Runtime | Gazebo中放置缺陷样本模型, 传送带模拟产线 |
| **MeterReader** | OCR(PaddleOCR) + 模板匹配 | ONNX Runtime | 仪表盘模型, 随机生成读数 |
| **SafetyChecker** | YOLOv8 + 区域分析 | ONNX Runtime | 人物模型+安全帽, 划定安全区域 |

**推理引擎** (`inference_engine.py`):
- ONNX Runtime 推理, CPU/CUDA 自动切换 (8GB显存足够跑多个轻量模型)
- 模型预热与批处理
- 支持 TensorRT 加速 (可选)

**检测编排** (`orchestrator.py`):
- 订阅 `/camera/rgb`, 按需启动/停止各检测器
- 检测结果发布到 `/inspection/result`
- 异常时发布 `/alert`
- Service: `/inspection/start` (std_srvs/Trigger)
- Service: `/inspection/set_mode` (切换检测模式)

### 4.4 大模型智能问答模块 (llm_agent)

**Provider 抽象层** — 统一接口，支持多模型:

```python
class BaseLLMProvider(ABC):
    @abstractmethod
    async def chat(self, messages: list[dict], **kwargs) -> str: ...
    
    @abstractmethod
    async def chat_with_image(self, messages: list[dict], images: list[bytes]) -> str: ...
    
    @abstractmethod
    def get_model_info(self) -> dict: ...
```

**三个 Provider 实现**:
- `QwenProvider` — 通义千问 (DashScope API / OpenAI 兼容)
- `ErnieProvider` — 文心一言 (千帆 API)
- `DeepSeekProvider` — DeepSeek (OpenAI 兼容 API)

**核心功能**:

1. **工业知识问答** (`industrial_kb.py`)
   - 嵌入式工业知识库 (JSON → 向量检索)
   - RAG 增强: 检索相关工业标准/操作规范 → 注入 Prompt
   - 支持: 设备参数查询, 故障诊断建议, SOP 问答

2. **多模态分析** (`multimodal_analyzer.py`)
   - 输入: 检测图像 + 检测结果 + 用户问题
   - 利用多模态 LLM 进行图像理解:
     - "这个区域的缺陷严重程度如何？"
     - "仪表读数是否在正常范围内？"
     - "这个安全隐患的优先级是什么？"

3. **巡检报告生成** (`report_generator.py`)
   - 收集一次巡检的全部检测结果
   - 自动生成结构化巡检报告 (Markdown):
     - 巡检概要 (时间/路线/时长)
     - 检测结果汇总
     - 异常项详细描述 (含图片)
     - AI 分析建议
     - 整改建议优先级

**ROS 接口**:
- Service: `/llm/query` — 同步问答
- Service: `/llm/generate_report` — 报告生成
- Topic: `/llm/response` — 流式响应 (异步)

### 4.5 自定义消息接口 (inspection_msgs)

```ros2
# InspectionResult.msg
std_msgs/Header header
string detector_type          # 'defect' | 'meter' | 'safety'
string status                 # 'ok' | 'warning' | 'error'
string description            # 检测结果描述
float32 confidence            # 置信度
sensor_msgs/Image annotated_image  # 标注图像
geometry_msgs/Pose detection_pose  # 检测位置

# Alert.msg
std_msgs/Header header
uint8 severity                # 0=info, 1=warning, 2=critical, 3=emergency
string message
string[] suggested_actions
sensor_msgs/Image evidence_image
```

### 4.6 仿真场景 (simulation)

**工厂车间场景** (`industrial_factory.world`):
- 车间地面、墙壁、柱子的基础结构
- 传送带区域 (产线质检)
- 设备区域 (仪表盘、管路、阀门)  — 设备巡检
- 安全区域 (围栏、标识线、人员通道)  — 安全巡检

**检测目标模型** (放在 `simulation/models/`):
- `defect_samples/` — 含人为缺陷的工件模型 (裂纹、划痕、形变)
- `instrument_panel/` — 带可旋转指针的仪表盘 (生成随机读数)
- `safety_zone/` — 安全区域标识, 可放置人物模型测试安全检测

**生成模型来源**:
- Gazebo Model Database 下载基础模型
- 用 Blender 制作自定义工业部件模型
- 缺陷纹理处理增强真实感

---

## 五、开发阶段

### Phase 1: 项目骨架 + 机器人模型 + 基础仿真环境 (3-4天)

**目标**: ROS2 workspace 可编译运行, 机器人在 Gazebo 里动起来

```
□ 创建 ROS2 workspace
□ robot_description: 差分驱动底盘 URDF/XACRO
□ robot_bringup: 仿真启动 launch 文件
□ simulation: 基础工厂场景 world 文件
□ 验证: spawn 机器人 → 键盘控制移动 → 相机/雷达数据正常
```

### Phase 2: 导航与巡航 (4-5天)

**目标**: 机器人能自主建图、导航、按路径巡航

```
□ robot_navigation:
  □ SLAM Toolbox 配置与调参
  □ Nav2 参数配置 (planner + controller + behavior tree)
  □ waypoint_patrol.py — 路径巡航实现
  □ patrol_server.py — 巡航 Action 服务
□ inspection_msgs: PatrolMission action 定义
□ 验证: 建图 → 保存地图 → 设置巡检点 → 自动巡航
```

### Phase 3: 视觉检测管线 (5-6天)

**目标**: 三种检测器可用，在仿真中检出异常

```
□ robot_inspection:
  □ base_detector.py — 插件接口
  □ model_manager.py + inference_engine.py — ONNX Runtime 推理
  □ defect_detector.py — YOLOv8 产品缺陷检测
  □ meter_reader.py — PaddleOCR 仪表读数
  □ safety_checker.py — 安全穿戴 + 区域入侵检测
  □ orchestrator.py — 检测编排 (与巡航联动)
□ inspection_msgs: 所有 msg/srv 定义
□ simulation: 检测目标模型 (缺陷样本/仪表/人物)
□ 验证: 巡航到检测点 → 自动启动检测 → 检出模拟异常
```

### Phase 4: 大模型智能问答 (3-4天)

**目标**: LLM 接入, 工业问答, 巡检报告自动生成

```
□ llm_agent:
  □ base_provider.py — API 抽象层
  □ qwen_provider.py + ernie_provider.py + deepseek_provider.py
  □ industrial_kb.py — 工业知识库 + RAG
  □ multimodal_analyzer.py — 多模态分析
  □ report_generator.py — 报告生成
  □ llm_node.py — ROS2 服务节点
□ llm_config.yaml — API 配置模板
□ 验证: 问设备参数 → RAG 检索 + LLM 回答 → 巡检后生成 Markdown 报告
```

### Phase 5: 全系统集成与仿真验证 (3-4天)

```
□ 端到端巡检流程:
  □ 启动仿真 → 加载地图 → 设定巡检路线 → 开始巡航
  □ 到达检测点 → 触发对应检测器 → 收集结果
  □ 异常告警 → LLM 分析 → 生成巡检报告
□ 三种场景覆盖:
  □ 产线缺陷检测场景
  □ 设备仪表巡检场景
  □ 安全合规检查场景
□ 系统稳定性测试
□ 文档编写
```

---

## 六、关键技术决策

| 决策项 | 选择 | 理由 |
|--------|------|------|
| ROS2 版本 | Humble | Ubuntu 22.04 官方支持, LTS 到 2027 |
| SLAM | SLAM Toolbox | Humble 原生支持, 异步建图, 比 Cartographer 轻量 |
| 导航 | Nav2 | 标准方案, 生态完善, Behavior Tree 可定制巡航逻辑 |
| 深度学习框架 | ONNX Runtime | 跨平台, CPU/CUDA/TensorRT 通用, 适合边缘部署 |
| 检测模型 | YOLOv8-n/s | 轻量级, mAP 足够工业场景, 8GB 显存可并行跑多个 |
| OCR | PaddleOCR | 中文识别最优, 支持仪表数字/刻度识别 |
| LLM 协议 | OpenAI 兼容格式 | 通义千问/DeepSeek 均支持, 减少适配代码 |
| RAG | ChromaDB (嵌入式) | 无需额外部署, 适合嵌入式知识库 |
| Gazebo 版本 | Gazebo 11 Classic | 已安装, ros-humble-gazebo-* 包兼容 |
| 差速控制 | ros2_control + gazebo_ros2_control | Humble 标准控制框架 |

---

## 七、依赖清单 (ROS2 Packages)

```bash
# 导航
ros-humble-nav2-* ros-humble-slam-toolbox ros-humble-navigation2
# 控制
ros-humble-ros2-control ros-humble-gazebo-ros2-control ros-humble-ros2-controllers
# 传感器
ros-humble-gazebo-ros-pkgs ros-humble-cv-bridge ros-humble-image-transport
# Python
python3-pip python3-opencv python3-numpy
pip: onnxruntime-gpu paddleocr ultralytics chromadb qwen-agent dashscope openai
```

---

## 八、验证方案

### 每个 Phase 的验收标准

**Phase 1**: `ros2 launch robot_bringup simulation_bringup.launch.py` → Gazebo 中出现机器人 → `ros2 topic echo /cmd_vel` 可控制移动 → `ros2 topic echo /camera/rgb` 有图像输出

**Phase 2**: 键盘控制建图 → `ros2 run nav2_map_server map_saver_cli` 保存地图 → 设定 3 个 waypoint → `ros2 action send_goal /patrol/execute` → 机器人依次到达

**Phase 3**: 加载缺陷样本到 Gazebo → 机器人到达检测位 → `ros2 service call /inspection/start` → `/inspection/result` 输出检出结果 → `/alert` 有异常告警

**Phase 4**: `ros2 service call /llm/query` 发送问询 → 返回专业回答 → 模拟一次巡检后调用 `/llm/generate_report` 生成完整报告

**Phase 5**: 一条命令启动全流程 (`ros2 launch robot_bringup full_inspection_demo.launch.py`) → 机器人自动完成巡检测试 → 终端输出巡检报告

---

## 九、后续扩展方向 (本阶段不实现, 架构预留)

- 多机器人协同巡检
- 边缘-云端 LLM 混合推理
- 数字孪生 Web 可视化面板
- 真实机器人硬件适配层 (Jetson Orin)
- 5G 远程操控与直播推流
- 预测性维护 (时序数据分析)
