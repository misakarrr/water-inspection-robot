# 水利巡检机器狗 · Isaac Sim 上手路线图（零基础版）

> 目标：在 Isaac Sim 中搭建"巡检道路 + 混凝土墙 + 转弯 + 障碍物 + 禁入区"的水利巡检场景，
> 用 ROS 2 Nav2 实现自主巡检，并用合成数据训练"裂缝 / 渗水"病害识别模型，最终形成可运行闭环。
> 基准版本：Isaac Sim 6.1（2026-09 GA）+ ROS 2 Jazzy（以官方 install_ros 兼容矩阵为准）。

---

## 0. 一页版结论（先记住这 5 条）

1. **把机器狗先降维成"差速底盘"**：Nav2 只认 `/cmd_vel`，先让一个能执行速度指令的底盘跑通导航，
   四足步态（RL locomotion policy）留到第 9 周做可选升级。否则"导航 + 强化学习"两个难题耦合，入门必卡。
2. **裂缝/渗水在仿真里是"贴图问题"，不是"建模问题"**：真实裂缝是毫米级，几何建模不现实；
   用 albedo + normal + roughness 贴图 + 域随机化生成数据，再训练分割/检测模型。
3. **地图用 Isaac Sim 的 Occupancy Map 扩展烘焙**（快），跑通后再换成仿真内 SLAM（贴近真机）。
4. **禁入区用 Nav2 的 KeepoutFilter**（mask 图），同时在仿真里画成可见的红色区域做双重验证。
5. **版本必须成套**：Isaac Sim 6.1 / ROS 2 Jazzy / 驱动 580.x / Nav2 对应发行版。
   网上 90% 的 Isaac Sim 教程是 4.x 的，直接抄代码会报错——只看与你版本号一致的文档。

---

## 1. 目标拆解：你说的"训练"其实是 4 件不同的事

| 编号 | 任务 | 工具 | 难度 | 是否必须 |
|---|---|---|---|---|
| T1 | 场景与数字孪生搭建 | Isaac Sim GUI + Python(USD) | ★★ | 必须（第 1 步） |
| T2 | 自主导航（定位/建图/避障/巡线） | ROS 2 Nav2 + Isaac Sim ROS 2 Bridge | ★★★ | 必须 |
| T3 | 病害识别模型训练 | Replicator 合成数据 + PyTorch(YOLO-seg/U-Net) | ★★★★ | 必须（项目核心价值） |
| T4 | 四足运动控制策略训练 | Isaac Lab（RL，velocity tracking） | ★★★★★ | 可选升级 |

> 建议顺序：T1 → T2 → T3 → （有余力再）T4。
> T4 是"让狗真的用四条腿走路"，T2 是"让底盘按导航指令移动"。先做 T2，项目才能出成果。

---

## 2. 关键设计决策（踩坑前先定）

| 决策点 | 推荐做法 | 理由 |
|---|---|---|
| 机器狗形态 | 第 1 阶段用 USD 四足外观 + 隐藏关节驱动（差速底盘行为） | 保留"像机器狗"，但控制简单 |
| 导航定位来源 | 先用 ground-truth odom（仿真真值） | 排除定位误差干扰，先验证规划 |
| 地图来源 | Occupancy Map 扩展烘焙静态图 → 后期切 SLAM | 快速闭环 |
| 障碍感知 | RTX Lidar（3D）→ `pointcloud_to_laserscan` 或 Nav2 voxel layer | 复用真机同款传感器 |
| 病害检测输入 | 前视 RGB 相机（1080p，固定俯仰角） | 与真机部署形态一致 |
| 裂缝尺度 | 墙面 2 m 高，相机 1.5~2.5 m 距离，1080p 下 1 cm 裂缝 ≈ 15~25 px | 像素级可见，模型可学 |
| 渗水表征 | 深色湿斑 + 低粗糙度/高光 + 可选"热成像"灰度渲染 | 无需流体仿真 |
| 天气/光照 | Replicator 随机化（正午/阴天/黄昏/湿面） | 缩小 sim-to-real 差距 |

---

## 3. 环境准备清单

### 3.1 硬件（官方 6.x 要求）

| 组件 | 最低 | 推荐 |
|---|---|---|
| GPU | GeForce RTX 4080 | RTX 5080 |
| 显存 | 16 GB | 16 GB+ |
| CPU | 4 核 | 8 核 |
| 内存 | 32 GB | 64 GB |
| 存储 | 50 GB SSD | 500 GB SSD |
| 系统 | Ubuntu 22.04/24.04、Windows 10/11 | Ubuntu 24.04 |
| 驱动 | Linux 580.65.06 / Windows 580.88 | 同左 |

> 若本机不达标：用云端 RTX 实例（NVIDIA Brev / AWS g6 / 阿里云·腾讯云 GPU 实例），
> 走 WebRTC Livestream 或容器无头模式；本机只留 ROS 2 与 RViz。
> 注意：Isaac Sim 必须有 NVIDIA RTX 显卡，AMD/核显无法运行。

**关于 RTX 3090（常见问题）**

- 结论：**可以胜任本项目**。它有 RT Cores（满足硬性前提）、**24 GB 显存**（比官方最低 16 GB 更宽裕）、
  驱动 580.x 也支持 Ampere。属于"低于官方最低标称（4080）、实际可用"的档位。
- 短板在**光追吞吐**：第 2 代 RT Core、无 Ada 的 SER（Shader Execution Reordering），
  RTX Lidar/Radar 与路径追踪渲染明显更慢，且官方不为其做验证。
- 使用建议：
  1. RTX Lidar 降到 32 线 / 10 Hz 起步，再逐级上调；
  2. 日常用 **RTX Real-Time** 渲染模式，只有最终出图才开 Path Tracing；
  3. Replicator 批处理时控制并发相机数与分辨率；
  4. Isaac Lab 训练从 1024~2048 个环境起试，别照抄 4096/8192 的配置。
- 二手 3090（矿卡）必做检查：`nvidia-smi -q -d TEMPERATURE` 看显存结温、
  用 `gpu-burn` / `cuda_memtest` 压测 30~60 分钟、必要时更换显存导热垫；电源 ≥750 W。
- 官方提供 **Isaac Sim Compatibility Checker**，装之前先跑一遍确认环境是否放行。

**关于 RTX 3060 Ti（8 GB）**

- 结论：**不能作为本项目的主力卡**。瓶颈不是算力，而是 **8 GB 显存**（6.x 官方最低 16 GB）。
- 能做的事：跑通教程、搭 USD 场景、ROS 2 + Nav2 联调（导航基本不吃显存）、本地 bag 回放。
- 做不了 / 很痛苦：Replicator 批量出图（项目核心）、Isaac Lab 并行训练、RTX Lidar 仿真、中大型场景渲染。
- 只有这张卡时的最低配置：用 4.x 世代版本 + 720p + 单相机 + 关闭 Path Tracing +
  Lidar 降到 16/32 线 @5-10 Hz；**SDG 与训练放云端 4090，本机只做交互与验证**。
- 采购提醒：**RTX 3060 12 GB 比 3060 Ti 8 GB 更适合本项目**——显存比那点算力差距重要得多。

### 3.2 软件清单（按安装顺序）

1. Ubuntu 24.04 LTS（或 Windows 11；本文以 Ubuntu 为例）
2. NVIDIA 驱动 580.65.06
3. Isaac Sim 6.1（Workstation 安装包或容器）
4. Isaac Sim Assets 资产包（单独下载，最容易漏）
5. ROS 2 Jazzy（`ros-jazzy-desktop`）
6. `nav2`、`slam_toolbox`、`robot_localization`、`pointcloud_to_laserscan`、`rviz2`
7. Isaac Sim ROS 2 Bridge 工作空间（`isaac-sim/IsaacSim-ros_workspaces`）
8. Python 侧：`ultralytics`（或 mmsegmentation）、`opencv-python`

### 3.3 一次性验证（第 1 周必须全绿）

```bash
# ① Isaac Sim 能启动
./python.sh -c "from isaacsim import SimulationApp; app=SimulationApp({'headless':True}); app.close()"

# ② ROS 2 能跑
ros2 run demo_nodes_cpp talker

# ③ Bridge 打通：Isaac Sim 里打开 ROS 2 Bridge 扩展后
ros2 topic list          # 期望看到 /clock /tf
ros2 topic echo /clock --once
```

### 3.4 硬件角色分工：Jetson AGX Orin 能做什么、不能做什么

**结论：Orin 不能跑 Isaac Sim（也不该试），但它是本项目的"机载大脑"，必须要有。**

| 任务 | 推荐硬件 | AGX Orin 可以吗 |
|---|---|---|
| Isaac Sim 仿真 / GUI | RTX 工作站（≥4080 16GB）、DGX Spark、云 RTX | ❌ 无 RT Cores，且为 aarch64，官方明确不支持 |
| Replicator 合成数据 | 同上 | ❌ |
| 检测模型训练 | RTX 工作站 / 云 GPU | ⚠️ 能跑但极慢，不划算 |
| 四足 RL 策略训练（Isaac Lab） | RTX 工作站 / 云 | ❌ |
| 病害实时推理（TensorRT） | AGX Orin | ✅ 强项（275 TOPS） |
| ROS 2 + Nav2 导航栈 | AGX Orin | ✅ |
| Isaac ROS（cuVSLAM / nvblox / AprilTag） | AGX Orin | ✅ 设计用途 |
| 硬件在环 HIL 验证 | Isaac Sim(PC) + Orin(机载栈) | ✅ 推荐的 sim2real 路径 |

- 官方 Jetson HIL 课程 FAQ 原文：**"Can Jetson run Isaac SIM? Short answer: No."**
- 原因：① 无光追单元（RT Cores），而 Isaac Sim 的渲染管线（**包括无头模式**）依赖 RTX；
  ② 平台为 aarch64，官方仿真主机要求 x86_64 + RTX 显卡（A5000/A6000/L40S 或 GeForce RTX 4080+，显存 ≥16 GB）。
- 顺带纠正一个常见误解：**Jetson AGX Thor 同样没有 RT Cores，也跑不了 Isaac Sim**；
  Thor/Orin 的价值是"更强的边缘推理 + 部署"，不是"便宜的仿真机"。

**只有 Orin 一台设备时的替代方案**

1. 云上 RTX 实例跑 Isaac Sim（无头 + WebRTC 串流），本机只跑 ROS 2 / RViz / 模型训练（或也用云）。
2. 本地先用 Gazebo/MuJoCo 在 Orin 上做 Nav2 联调（能跑），但场景/传感器与 Isaac Sim 不通用，
   最终的场景、SDG 与验证仍要回到 Isaac Sim。
3. 训练检测模型可先用云 GPU 或任何一台 RTX 笔记本，比在 Orin 上训快一个数量级。

---

## 4. 12 周路线图（每天 2~3 小时）

| 周 | 阶段 | 主要任务 | 验收标准 |
|---|---|---|---|
| W1 | 地基 | 装驱动/Isaac Sim/资产/ROS 2；跑 Hello World、Hello Robot、Core API 前 3 课 | 能打开 GUI、能加载机器人、`/clock` 有数据 |
| W2 | 场景 | 搭巡检道路（L 形）、混凝土墙、障碍物、禁入标记；整理 prim 命名 | 场景 USD 保存成功，FPS 稳定，比例正确（1 unit = 1 m） |
| W3 | 底盘 | 加差速底盘行为、/odom、/tf、/cmd_vel；RViz 可视化 | RViz 里 base_link 随 cmd_vel 移动，TF 树完整 |
| W4 | 导航 | 烘焙 Occupancy Map → map_server → Nav2 静态层 → 障碍层 → 控制器 | 下发 2D Goal 能自主到达；禁入区目标被拒绝 |
| W5 | 巡检逻辑 | Nav2 Waypoint Follower 跑完 6~10 个巡检点；到点停车拍照 | 一条命令跑完整条巡检路线 |
| W6 | 病害资产 | 墙面裂缝/渗水材质；相机标定（高度/俯仰/FOV） | 视图中裂缝清晰可见，可截图存盘 |
| W7 | 数据合成 | Replicator 脚本：随机裂缝参数、光照、视角、天气 → 输出 RGB+标注 | 生成 ≥5000 张带标注图像，Train/Val 划分完成 |
| W8 | 模型训练 | YOLO-seg 或 U-Net 训练 + 评估（mAP/IoU，按距离分桶统计） | Val 集 IoU ≥ 0.5，输出模型权重 |
| W9 | 闭环推理 | ROS 2 推理节点：相机 → 推理 → `/defect_event`；仿真中验证 | 巡检时能自动框出病害并发布事件 |
| W10 | 任务编排 | 状态机（巡检→停车→拍照→推理→上报→继续）+ 事件记录文件 | 一键跑完整流程并生成巡检报告 JSON/CSV |
| W11 | 真实感升级（可选） | heightmap 导入坝坡；天气随机化；GNSS+EKF | 机器人能爬坡巡检，定位链路完整 |
| W12 | 收尾/Sim2Real | 四足步态策略接入（可选）；整理 sim-to-real 差异清单与真机部署文档 | 形成可交付的演示 + 文档 |

### 两周 MVP（赶时间版）

- D1–D2：环境 + Hello World
- D3–D4：官方 warehouse/simple 场景 + 差速底盘 + Nav2 直道导航
- D5–D6：加墙 + 转弯 + 禁入区 Keepout，跑通 3 个巡检点
- D7–D9：墙面裂缝贴图 + Replicator 生成 2000 张
- D10–D12：训练小模型（YOLOv8n-seg）
- D13–D14：接回仿真闭环 + 演示录屏

---

## 5. 场景规格书（把你描述的元素变成可施工的参数）

| 元素 | 建议规格 | 实现要点 |
|---|---|---|
| 巡检道路 | L 形：直道 60 m + 90° 转弯 + 直道 40 m，宽 3 m；可选 3° 缓坡 | `Cube` 拉平或用 heightmap；路面材质给一定摩擦 |
| 混凝土墙 | 高 2.5 m、长 40 m、厚 0.3 m，沿路单侧布置 | 检测 ROI；表面用裂缝贴图，法线贴图增强立体感 |
| 转弯 | 90° 弯道，内角放一个障碍物逼出真实避障行为 | 用于验证 Nav2 的 costmap 膨胀半径 |
| 障碍物 | 锥桶 ×5、石块 ×3、树枝/落石堆 ×1、窄通道 1 处（宽 1.2 m） | 用官方 Props 或简单几何 + 语义标签 |
| 禁入区域 | 道路外侧靠"水面/坝坡"一侧，宽 3 m 的条带 | ①Nav2 KeepoutFilter mask 涂黑；②仿真中铺红色警戒材质 |
| 水面（可选） | 静态平面 + 半透明材质（不做流体） | 仅作视觉参考与边界 |

### 命名规范（现在就定，后期省一周）

```
/World/WaterInspection/
├── Environment/Road, Wall_Crackable, Water_Surface, Keepout_Marker
├── Obstacles/Cone_01..., Rock_01..., NarrowPass
├── Robots/Go2_01 (base_link = /World/.../base_link)
├── Sensors/Camera_Front, Lidar_Mid360, IMU
└── Zones/InspectionPoint_01..N, Keepout_Polygon
```

### 语义/标注预埋

- 墙面 prim 打语义标签 `class: wall_surface`，裂缝子 prim 用 `class: crack` / `class: seepage`。
- 标注输出格式一开始就选 COCO（检测/分割通用），不要中途换。

---

## 6. Nav2 集成要点

### 6.1 必须发布的 5 个话题

| 话题 | 类型 | 来源 |
|---|---|---|
| `/clock` | rosgraph_msgs/Clock | Isaac Sim ROS 2 Bridge（全链 `use_sim_time:=true`） |
| `/tf`, `/tf_static` | tf2_msgs/TFMessage | Bridge TF 节点（map→odom→base_link→sensors） |
| `/odom` | nav_msgs/Odometry | 底盘/真值 |
| `/scan` 或 `/points` | LaserScan / PointCloud2 | RTX Lidar（3D 用 `pointcloud_to_laserscan`） |
| `/cmd_vel` | geometry_msgs/Twist | Nav2 → 底盘控制器 |

### 6.2 地图两条路线

- **路线 A（推荐先做）**：Isaac Sim *Occupancy Map* 扩展 → 从 USD 场景烘焙 2D 占据栅格 → 导出 PGM/YAML → `map_server`。
  改了场景要重新烘焙（容易忘）。
- **路线 B（更贴真机）**：仿真里跑 `slam_toolbox` 在线建图，地图与真机流程一致，但引入建图误差。

### 6.3 禁入区（KeepoutFilter）

1. 在场景俯视图上画一张 mask 图（PGM）：可通行=254，禁入=0，未知=205。
2. `nav2_params.yaml` 里 costmap 的 `filters` 层加入 `KeepoutFilter`，指定 `filter_mask_server` 与 `filter_info_topic`（或静态 YAML）。
3. 二次保险：任务层在下发目标前做一次"目标点是否落在禁入多边形内"的校验。
4. 参考：`docs.nav2.org` → Configuration → Costmap Plugins → Keepout Filter。

### 6.4 巡检任务编排

```
Nav2 Waypoint Follower（或自写 Python 节点）
   → 到达 InspectionPoint_XX
   → 停车 + 稳定 1 s
   → 触发相机拍照（存图 + 记录位姿）
   → 推理节点出病害结果
   → 发布 /defect_event + 写入巡检记录
   → 前往下一个点
```

- 进阶：用行为树（6.1 支持 USD 编写的 Omniverse Behavior Tree，或 `py_trees`）替换脚本。
- 6.1 起支持 `ros2_control`：可把 Controller Manager 跑在仿真里，控制器 YAML 与真机共用（W11 再上）。

---

## 7. 病害识别数据管线（项目核心）

### 步骤

1. **素材**：收集真实裂缝/渗水图片（开源数据集如 SDNET2018、Concrete Crack Images、CrackSeg9k），
   转成可平铺贴图；渗水用"深色湿斑 + 粗糙度/高光变化"制作。
2. **材质**：墙面 MDL/USD 材质 = 基础混凝土 albedo + 裂缝 albedo + normal + roughness；
   裂缝子 prim 单独存在，便于打标注。
3. **尺度校验**：先渲染一张图，确认裂缝像素宽度（1 cm ≈ 15~25 px @1080p/2 m）。
   太小则模型学不到，太大则失真——这是最关键的一次标定。
4. **Replicator 随机化**：
   - 裂缝：数量 0~6、长度、走向、分叉、宽度、深浅
   - 渗水：位置、面积、浓度、边界模糊度
   - 环境：太阳角度、云量、湿面反光、阴影
   - 相机：高度 0.6~1.2 m、俯仰 ±10°、距离 1.5~3 m、轻微抖动
5. **输出**：RGB + 语义/实例分割 + 2D 框（COCO），至少 5000 张（可无头批处理跑一夜）。
6. **训练**：`ultralytics` YOLOv8/YOLO11-seg 起步；数据量到 2 万张再考虑更大模型。
7. **评测分桶**：按"距离 / 光照 / 裂缝宽度"分桶统计召回率，直接决定真机的巡检速度与拍摄距离。
8. **闭环**：推理节点订阅 `/camera/image_raw` → 发布 `/defect_event`（含类别、置信度、位姿、时间）→ 写入报告。

### 与真机的差距（心里有数）

- 真实裂缝可能是"渗水痕迹/水垢/苔藓/阴影"等混淆项 → 在随机化里主动加入这些负样本。
- 真机抖动、运动模糊、曝光变化 → 仿真里加运动模糊与噪声（官方有相机噪声教程）。
- 热成像渗水检测需要真机数据校准，仿真里只能做"流程验证"。

---

## 8. 建议工程目录

```
water-inspection/
├── isaac/
│   ├── scenes/            # 场景 USD（road_wall_L.usd ...）
│   ├── robots/            # 机器狗封装 USD + 底盘配置
│   ├── materials/         # 裂缝/渗水材质
│   ├── scripts/           # standalone 启动、Replicator 数据生成
│   └── replicator/
├── ros2_ws/src/
│   ├── inspection_bringup/    # launch + nav2_params.yaml + 地图
│   ├── inspection_robot/      # 底盘/传感器接口
│   ├── inspection_mission/    # 巡检任务节点（waypoint + 状态机）
│   └── defect_detector/       # 推理节点 + 模型权重
├── datasets/              # 合成数据（或软链到大盘）
├── models/                # 训练产物
└── docs/                  # 场景规格、巡检点清单、sim2real 差异表
```

---

## 9. 常见坑清单（按出现频率排序）

1. **版本错配**：Isaac Sim 6.x 的 Core API 大量迁移到 `isaacsim.core.experimental`，
   4.x 教程代码直接报 ImportError → 只对照当前版本文档。
2. **忘了装资产包**：场景加载失败、机器人是白色方块。
3. **单位不统一**：USD `metersPerUnit` 混用 → 场景比例错乱，Nav2 参数全部失效。
4. **`use_sim_time` 不一致**：TF 报 extrapolation into the future / past。
5. **TF 树不完整**：Nav2 起不来，先 `ros2 run tf2_tools view_frames`。
6. **改了场景没重烘焙地图**：机器人在 RViz 里"穿墙"或原地打转。
7. **RTX 传感器开太多**：Lidar + 多相机 + 高 tick rate → 实时因子掉到 0.1 以下。
   先降分辨率与 tick rate，再谈性能优化。
8. **禁入区只做在 Nav2 层**：mask 画错/加载失败时毫无保护 → 加任务层校验。
9. **标注格式中途更换**：越早定 COCO 越好。
10. **一上来就搞 RL 步态**：这是最容易劝退的路径，务必放到最后。

---

## 10. 官方与社区资源

**必读官方文档（注意选你的版本）**
- ROS 2 Navigation 教程：`docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/robot_control/tutorial_ros2_navigation.html`
- 带高程图的导航：`.../tutorial_ros2_navigation_heightmap.html`
- ros2_control：`.../tutorial_ros2_control.html`
- ROS 2 安装与兼容矩阵：`.../installation/install_ros.html`
- Occupancy Map 扩展：`.../digital_twin/ext_isaacsim_asset_generator_occupancy_map.html`
- 四足机器人资产：`.../assets/usd_assets_robots_quadruped.html`
- Replicator 总览：`.../replicator_tutorials/tutorial_replicator_overview.html`
- 相机噪声：`.../ros2_tutorials/advanced_sensors/tutorial_ros2_camera_noise.html`

**代码仓库**
- Isaac Sim ROS 2 工作空间：`github.com/isaac-sim/IsaacSim-ros_workspaces`
- Go2 + Nav2 参考项目：`github.com/ChefChun/isaac-go2-ros2`
- Go2/G1 for Isaac Lab：`github.com/danmartinez78/go2_omniverse`
- Isaac Lab：`isaac-sim.github.io/IsaacLab`
- 人形 loco-manipulation 工作流（含策略部署范式）：`github.com/nvidia-isaac/WBC-AGILE`

**学习顺序建议**
1. Isaac Sim 基础用法 → 2. Core API 教程 → 3. ROS 2 TurtleBot 教程 →
4. ROS 2 Navigation 教程 → 5. Replicator 入门 → 6. 回到本项目按 W1~W12 推进

---

## 11. 附录 A：把一台机器做成多人共用的 Isaac Sim 服务器

**核心结论**：不能"多人共享一个 Isaac Sim 进程"，正确模式是
**一台服务器跑 N 个独立实例（每用户一个容器 + 一块 GPU），各自用 WebRTC 串流到自己的电脑**
（这是 NVIDIA 论坛给出的推荐做法）。而且**必须用 Isaac Sim 6.0 及以上**——
6.0 才支持自定义 livestream 端口（5.0 明确不支持），端口可改是多实例并存的先决条件。

**两种典型模式**

| 模式 | 做法 | 适合 |
|---|---|---|
| 交互式：一用户一容器一 GPU + WebRTC | `docker run --gpus '"device=N"' --network=host`，每实例独立 signalPort/streamPort | 教学、团队日常开发 |
| 批处理：无头作业队列 | 共享实例，用户提交 SDG/训练脚本排队 | 数据生成、RL 训练（最省显卡） |

**双实例参考命令（6.x）**

```bash
# 用户 1：GPU0，signalPort 49100 / streamPort 47998
docker run --name isaac-user1 --rm --runtime=nvidia --gpus '"device=0"' \
  --network=host --ipc=host -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v ~/isaac/user1/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/isaac/user1/documents:/isaac-sim/Documents:rw \
  nvcr.io/nvidia/isaac-sim:6.1.0 ./runheadless.sh -v \
    --/exts/omni.kit.livestream.app/primaryStream/publicIp=$PUBLIC_IP \
    --/exts/omni.kit.livestream.app/primaryStream/signalPort=49100 \
    --/exts/omni.kit.livestream.app/primaryStream/streamPort=47998

# 用户 2：GPU1，signalPort 49101 / streamPort 47999（其余相同）
```

- 需放行：TCP/UDP `47995-48012`、TCP/UDP `49000-49007`、TCP `49100+`；默认 HTTP 8011 / WS 8040,8098。
- 每个实例使用**独立的 cache 与 documents 目录**，否则 Kit 缓存冲突。
- 客户端：WebRTC Streaming Client（原生 App）或浏览器；跨网段需正确设置 `publicIp` 并放行 UDP。

**容量规划（每用户）**

| 资源 | 建议 |
|---|---|
| GPU | 1 块（GeForce 消费卡**不支持 MIG/vGPU**，不能切分） |
| 显存 | ≥16 GB；轻量场景实测单实例约 2.4~6.3 GB |
| CPU | 4~8 核 |
| 内存 | 16~32 GB（主机 64 GB 只够 2 人，4 人建议 128~256 GB） |
| 磁盘 | Isaac Sim + 资产约 100 GB 起，另加每人数据集；NVMe 必备 |
| 网络 | 局域网千兆起，建议 ≥50 Mbps/用户的稳定上行 |

**只有一台 3090 时的现实做法**

1. **1 个交互用户 + N 个无头作业**（最划算）：GUI 串流给一人，其余人提交脚本排队；
2. 时间片轮转；
3. 本地只做 USD 组装与 ROS 2 联调，重活（SDG / 训练）上云按人开实例；
4. 不要指望 3090 + 远程桌面让 4 人同时流畅交互。

**其他注意**

- Omniverse Launcher 已于 2025-10-01 退役，Nucleus Workstation 同步退役；
  多用户资产共享现在走 Enterprise Nucleus Server（NGC 上有非生产版）或共享文件系统 / Git。
- 不要多人同时写同一个 USD 文件：按 layer 拆分文件，用 Git/Perforce 管理。
- Isaac Sim 本体 Apache 2.0 免费；Nucleus / Omniverse Enterprise 属商业产品，另行授权。

---

## 12. 第 1 周就能做的三件事（今天开始）

- [ ] 确认显卡型号与显存（`nvidia-smi`），不达标就先申请云 GPU 或换机
- [ ] 下载 Isaac Sim 6.1 + 资产包，跑通 Quick Install 与 GUI Hello World
- [ ] 装 ROS 2 与 Nav2，跑通 TurtleBot 官方示例，理解 `/cmd_vel` 与 TF 树
