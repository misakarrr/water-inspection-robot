# 水利巡检机器人 Nav2 导航原型实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Complete each task's checks before starting the next task.

**目标：** 在 Ubuntu 22.04 + ROS 2 Humble + Isaac Sim 4.x + RTX 3090 上，以简化差速底盘完成固定巡检路线导航，随后切换到激光雷达 + AMCL，并把 SLAM 留作独立可选阶段。

**架构：** 先使用 Isaac Sim 官方 ROS 2 导航示例中的差速机器人验证桥接和 `/cmd_vel`，再放入仓库已有的 L 形水利巡检场景。Nav2 使用固定 OccupancyGrid 地图和 KeepoutFilter；真值模式用于隔离定位问题，AMCL 模式由激光雷达和运动里程计提供定位。

**技术栈：** Ubuntu 22.04、ROS 2 Humble、Nav2、Isaac Sim 4.x ROS 2 Bridge、Python `ament_python` 包、RViz2、pytest/ament tests。

---

## 当前代码与计划文件

仓库已有 `isaac/scripts/build_scene.py`，它能生成 L 形道路、混凝土墙、锥桶、落石、窄通道、红色禁入标记和巡检点，但没有机器人，也没有 ROS 2 控制/传感器接口。场景图像输出和纯 USD 自检已有雏形，可继续使用。`docs/isaac-sim-water-inspection-roadmap.md` 与当前选择的 Isaac Sim 4.x / Humble 有冲突，执行前要在文档首页标注当前导航基线。

| 路径 | 责任 |
|---|---|
| `isaac/scripts/build_scene.py` | 参数化生成可复现的巡检 USD 场景；增加资产机器人入口和场景自检 |
| `isaac/robots/README.md` | 记录占位机器人资产、传感器位置、尺寸和后续替换方式 |
| `isaac/robots/inspection_carter.usda` | 对 Isaac Sim 官方 Carter 资产的项目级场景包装和 ROS 2 Action Graph |
| `isaac/scenes/` | 保存可再生成的场景输出；体积较大的 USD 不默认提交 Git |
| `ros2_ws/src/inspection_bringup/` | 地图、mask、Nav2 参数、启动文件和 RViz 配置 |
| `ros2_ws/src/inspection_mission/` | 巡检点 YAML、Waypoint Follower 调用和任务记录 |
| `ros2_ws/src/inspection_robot/` | 仅在官方 Isaac 图无法直接满足接口时，放 ROS 2 机器人适配节点 |
| `ros2_ws/src/*/test/` | 路线配置、禁入区校验和 launch smoke tests |
| `docs/environment-baseline.md` | 实际安装的 Ubuntu、驱动、Isaac Sim、ROS 2、Nav2 和 RMW 版本记录 |
| `README.md` | 项目文档索引和当前软件基线 |
| `docs/isaac-sim-water-inspection-roadmap.md` | 标注当前导航决策的优先级，避免旧版版本说明误导执行 |

## 分阶段验收门

| 阶段 | 验收门 |
|---|---|
| P0 环境 | Isaac Sim headless 冒烟测试、ROS 2 demo 和 Isaac ROS Bridge `/clock` 检查全部通过 |
| P1 场景 | USD 可重建；单位/坐标轴正确；俯视图显示道路、墙、障碍物、弯道和禁入区 |
| P2 控制 | `/cmd_vel` 驱动机器人；`/odom`、`/tf` 连续；ROS 2 时间一致 |
| P3 真值导航 | 固定地图中的 3 至 5 个巡检点全部可达；KeepoutFilter 生效；全程 10 次无碰撞 |
| P4 AMCL | 停止真值定位发布后，AMCL 单独提供 `map -> odom`；同一任务可完成 |
| P5 可选 SLAM | 在线建图并保存，重启后能在保存地图上定位并完成路线 |

阶段 P0 未通过前不调 Nav2；P2 未通过前不调路径规划；P3 未通过前不接 AMCL。每次只让一个节点发布某一条 TF。

---

### Task 1: 固定软件与硬件基线

**Files:**
- Create: `docs/robot-asset-inventory.md`
- Create: `docs/environment-baseline.md`
- Modify: `README.md`
- Modify: `docs/isaac-sim-water-inspection-roadmap.md`

- [ ] **Step 1: 盘点实验室机器人资产**

在 Ubuntu 中搜索常见机器人描述、网格和 ROS 包文件：

```bash
find ~/ -type f \( \
  -iname '*.urdf' -o -iname '*.xacro' -o \
  -iname '*.usd' -o -iname '*.usda' -o -iname '*.usdc' -o \
  -iname '*.stl' -o -iname '*.obj' -o -iname '*.fbx' -o \
  -iname '*.step' -o -iname '*.stp' -o -name 'package.xml' \
\) -print 2>/dev/null
```

将结果归类写到 `docs/robot-asset-inventory.md`，不搬动或改写实验室原始文件。按结果标记后续路径：有 URDF/Xacro 就核查关节、碰撞、惯量和 ROS 控制接口；只有 USD 就确认 Articulation Root、关节 drive 和碰撞体；只有 CAD/网格则标为需补齐 link/joint/惯量后再导入；未找到模型时继续使用官方 Carter 资产。所有分支都先完成同一 Nav2 MVP，真实资产接入不阻塞导航链路。

- [ ] **Step 2: 记录本机信息**

在 Ubuntu 终端运行：

```bash
lsb_release -a
nvidia-smi
uname -r
df -h /
free -h
```

记录 Ubuntu 必须为 22.04、GPU 必须显示 RTX 3090 和约 24 GB 显存；确保系统盘至少有 100 GB 空闲空间，内存建议至少 32 GB。若空间或内存不足，先清理/扩容或决定把 Isaac Sim 安装到独立 NVMe，再继续。

- [ ] **Step 3: 核对 Isaac Sim 4.5 与 Humble 官方兼容表**

打开 Isaac Sim 4.5.0 Requirements 与 ROS 2 Installation 文档：

- `https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/requirements.html`
- `https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_ros.html`

确认 Ubuntu 22.04、当前 NVIDIA 驱动和 ROS 2 Humble Bridge 都在该发行版支持范围内。若 4.5.0 页面不支持当前驱动或 Humble，使用官方 4.x 发布矩阵中同时支持这三项的最新补丁版本，不跨大版本抄扩展配置。

- [ ] **Step 4: 写入版本记录并标记旧路线图**

创建 `docs/environment-baseline.md`，写入已知目标（Ubuntu 22.04、RTX 3090、ROS 2 Humble、Isaac Sim 4.5.0 候选）和实际驱动/Isaac 补丁版本。将 README 软件行改为“Isaac Sim 4.x（按 Humble 兼容矩阵固定补丁版）+ ROS 2 Humble + Nav2”，并把 RTX 3090 Ubuntu 工作站列为本地仿真主机；在旧路线图开头说明本导航计划优先，旧版 6.1/Jazzy 建议不适用于当前基线。

- [ ] **Step 5: 检查并提交基线记录**

```bash
git diff --check
git status --short
git add README.md docs/environment-baseline.md docs/robot-asset-inventory.md docs/isaac-sim-water-inspection-roadmap.md
git commit -m "docs: pin water inspection simulation baseline"
```

Expected: commit 只包含新增的两份基线文档、README 和旧路线图的基线提示，无 whitespace error。

### Task 2: 安装 ROS 2 Humble 与 Nav2

**Files:**
- Modify: Ubuntu 系统软件包和 `~/.bashrc`（不纳入仓库）
- Create: `ros2_ws/`

- [ ] **Step 1: 按 ROS 官方 Humble 教程配置 apt 源**

使用 `https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html` 的 Jammy 二进制安装步骤。设置 `en_US.UTF-8` locale、启用 Ubuntu Universe、安装官方 ROS apt source，再执行 `sudo apt update`。不要混用 Foxy、Iron 或 Jazzy 软件源。

- [ ] **Step 2: 安装 ROS、Nav2 和开发工具**

```bash
sudo apt install ros-humble-desktop ros-dev-tools \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-tf2-tools \
  ros-humble-pointcloud-to-laserscan ros-humble-teleop-twist-keyboard \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  python3-pytest python3-yaml python3-pil
```

Expected: apt 完成且无未满足依赖。

- [ ] **Step 3: 配置并验证 ROS 环境**

```bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
ros2 pkg list | grep -E '^(nav2_bringup|nav2_map_server|nav2_amcl|slam_toolbox)$'
ros2 run demo_nodes_cpp talker
```

Expected: `ROS_DISTRO` 为 `humble`，包列表包含 Nav2/AMCL/SLAM Toolbox，talker 持续输出 `Publishing`。另开终端运行 `ros2 run demo_nodes_cpp listener`，确认收到消息；用 `Ctrl+C` 停止两端。

- [ ] **Step 4: 将项目放到 Ubuntu 并配置工作区**

```bash
git clone https://github.com/misakarrr/water-inspection-robot.git ~/water-inspection-robot
mkdir -p ~/water-inspection-robot/ros2_ws/src
cd ~/water-inspection-robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
```

如果 Ubuntu 已有本仓库 checkout，不要重复 clone，直接在该 checkout 根目录执行后续命令。将 ROS 环境 source 命令加入 `~/.bashrc`，新开终端确认 `printenv ROS_DISTRO` 输出 `humble`。ROS 工作区无源包时 `colcon build` 可以提示无包但不能报 Python/ROS 环境错误。将仓库内创建的空目录由 `.gitkeep` 保持可跟踪，不提交系统安装产物。

### Task 3: 安装 Isaac Sim 并验证 ROS 2 Bridge

**Files:**
- Create: `~/isaacsim-4.5.0/`（本机软件，不纳入仓库）
- Create: `docs/environment-baseline.md` 中的实际安装记录

- [ ] **Step 1: 下载并安装兼容版本**

在 Task 1 确定的 Isaac Sim 4.x 补丁版官方安装文档中选择 Ubuntu Linux x86_64 workstation 安装方式，默认候选为 4.5.0。按官方流程安装到稳定路径 `~/isaacsim-4.5.0` 并完成 post-install；若兼容表要求其他 4.x 补丁版，仍使用这个稳定目录名，并在 `docs/environment-baseline.md` 记录实际补丁版本。

- [ ] **Step 2: 做无头启动检查**

```bash
~/isaacsim-4.5.0/python.sh -c 'from isaacsim import SimulationApp; app=SimulationApp({"headless": True}); print("ISAAC_SIM_OK"); app.close()'
```

Expected: 输出 `ISAAC_SIM_OK` 并正常退出。若安装包要求以其 `isaac-sim.sh` 初始化，先运行官方 post-install，再重复这条 Python 冒烟测试。

- [ ] **Step 3: 启动 Isaac Sim ROS 2 Bridge 示例**

按 4.5 官方 ROS 2 Bridge 教程启动 Isaac Sim 示例场景和 ROS 2 Bridge；Isaac Sim 和 ROS 2 进程都设置 `ROS_DOMAIN_ID=42`，所有 ROS 节点使用 `use_sim_time:=true`。另开 Humble 终端执行：

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /clock --once
```

Expected: 发现 `/clock` 且能读到非零仿真时钟。将 Isaac Sim 版本、驱动版本、安装方式、Bridge 示例名写入 `docs/environment-baseline.md` 并提交。

### Task 4: 生成并检查水利巡检场景

**Files:**
- Modify: `isaac/scripts/build_scene.py`
- Modify: `.gitignore`
- Create: `isaac/robots/README.md`
- Create: `isaac/robots/inspection_carter.usda`
- Create: 本机生成的 `isaac/scenes/water_inspection.usd` 和验收图（大文件按 `.gitignore` 规则处理）

- [ ] **Step 1: 运行现有脚本并保存俯视图**

```bash
cd ~/water-inspection-robot
~/isaacsim-4.5.0/python.sh isaac/scripts/build_scene.py \
  --out isaac/scenes/water_inspection.usd --usda --shots \
  --shotdir isaac/scenes/shots
```

Expected: 终端显示 `metersPerUnit = 1.0`、`upAxis = Z` 和 USD 保存路径，`isaac/scenes/shots/` 中有 top、along_a、wall、corner 视图。先打开 top 和 corner 图确认 L 形道路方向与场景尺度。

- [ ] **Step 2: 按批准设计修正道路外区域与禁入区**

在 `CFG` 中保留现有道路、墙和障碍物参数。先修正已有的道路断开错误：A 段从 `x=0` 延伸到 `x=leg_a`，所以 B 段中心改为 `(leg_a, leg_b/2, road_thickness/2)`，使它从转角 `(leg_a, 0)` 沿 `+Y` 延伸。窄通道挡板的 x 坐标也加上 `leg_a`。红色禁入区域沿 A 段 `+Y` 危险侧和 B 段 `+X` 危险侧延伸到转弯；ROS 地图中道路外区域不得作为 free space。加入机器人起点周围的净空，不把巡检点放在墙体、障碍物或禁入区内。每个固定障碍 prim 都必须有 collision API。

对应几何中心应按下面方式计算：

```python
road_b_center = (CFG["leg_a"], CFG["leg_b"] / 2, T / 2)
plate_center_x = CFG["leg_a"] + sign * (half_gap + plate_w / 2)
inspection_points = [(5, 0), (20, -0.8), (40, -0.8), (55, -0.8),
                     (CFG["leg_a"], 20), (CFG["leg_a"], 38)]
```

将对应的 B 段 `box(...)`、窄道挡板中心和 inspection point 循环改为使用这些值。B 段危险侧条带中心 x 为 `leg_a + road_width/2 + keepout_width/2`，长度沿 `+Y`。

`.gitignore` 增加 `runs/` 和 `isaac/scenes/shots/`，不忽略 PGM/YAML 地图或 `isaac/robots/README.md`。

增加几何自检：A 段终点中心 `(leg_a, 0)` 到 B 段起点中心 `(leg_a, 0)` 的距离为 0；最终俯视图显示连续转角，不能仅依赖 prim 数量验收。

- [ ] **Step 3: 添加可切换的官方差速机器人资产**

在 Isaac Sim 4.5 官方 ROS 2 Navigation 示例中先运行其差速机器人（优先使用 Carter 示例）。将项目包装层和控制 Action Graph 保存为 `isaac/robots/inspection_carter.usda`；场景脚本引用包装层，不把某台机器的绝对路径写进仓库。将机器人尺寸、footprint、传感器 frame、官方资产 URI 和资产版本记录在 `isaac/robots/README.md`。若官方资产库在目标安装中不可用，暂用带碰撞体的简化差速底盘验证图形和 Nav2 接口，真实四足模型仍延后。

- [ ] **Step 4: 用物理运行检查机器人和场景**

在 GUI 中打开生成 USD，按 Play 运行至少 60 秒；观察机器人是否沉降、穿地、卡住或撞墙。确认机器人能在 3 m 道路上通过弯道，1.2 m 窄道只在实际 footprint 和安全边距验证通过后加入导航路线。重新运行 Task 4 Step 1 保存最终验收视图。

### Task 5: 创建 ROS 2 bringup 与巡检任务包

**Files:**
- Create: `ros2_ws/src/inspection_bringup/package.xml`
- Create: `ros2_ws/src/inspection_bringup/setup.py`
- Create: `ros2_ws/src/inspection_bringup/setup.cfg`
- Create: `ros2_ws/src/inspection_bringup/resource/inspection_bringup`
- Create: `ros2_ws/src/inspection_bringup/inspection_bringup/`
- Create: `ros2_ws/src/inspection_bringup/config/nav2_params.yaml`
- Create: `ros2_ws/src/inspection_bringup/launch/navigation.launch.py`
- Create: `ros2_ws/src/inspection_mission/package.xml`
- Create: `ros2_ws/src/inspection_mission/setup.py`
- Create: `ros2_ws/src/inspection_mission/setup.cfg`
- Create: `ros2_ws/src/inspection_mission/resource/inspection_mission`
- Create: `ros2_ws/src/inspection_mission/inspection_mission/__init__.py`
- Create: `ros2_ws/src/inspection_mission/inspection_mission/config.py`
- Create: `ros2_ws/src/inspection_mission/config/inspection_points.yaml`
- Create: `ros2_ws/src/inspection_mission/test/test_inspection_points.py`

- [ ] **Step 1: 建立两个 ament_python 包**

在 `ros2_ws/src` 创建 `inspection_bringup` 和 `inspection_mission`，分别放启动/导航配置与巡检任务逻辑。使用以下命令创建包骨架：

```bash
cd ~/water-inspection-robot/ros2_ws/src
source /opt/ros/humble/setup.bash
ros2 pkg create --build-type ament_python inspection_bringup \
  --dependencies launch launch_ros nav2_bringup nav2_map_server
ros2 pkg create --build-type ament_python inspection_mission \
  --dependencies rclpy nav2_msgs geometry_msgs ament_index_python
```

不把仿真安装目录、地图生成代码和任务逻辑塞进同一个包。

`setup.py` 必须用 `data_files` 安装 `launch/`, `config/`, `maps/` 和 `rviz/` 内容到各自 ROS package 的 `share/<package>` 目录；`setup.cfg` 将 console scripts 安装到 `lib/<package>`，保证 `ros2 run` 能找到任务节点。`inspection_bringup` 的地图目录稍后由 Task 6 创建。

- [ ] **Step 2: 先写巡检点配置测试**

在 `inspection_mission/inspection_mission/config.py` 约定 `load_waypoints(path)`，返回包含 `id`, `x`, `y`, `yaw` 的 dict 列表。然后在 `inspection_mission/test/test_inspection_points.py` 测试唯一 `id`、有限 `x/y/yaw` 和非空列表。地图边界及 keepout 区通过 Task 6 的 mask/RViz 集成检查。先写下面两个失败测试：

```python
from pathlib import Path

import pytest

from inspection_mission.config import load_waypoints


def test_load_waypoints_rejects_duplicate_ids(tmp_path: Path):
    path = tmp_path / "points.yaml"
    path.write_text(
        "waypoints:\n"
        "  - {id: wall_a, x: 2.0, y: 1.0, yaw: 0.0}\n"
        "  - {id: wall_a, x: 3.0, y: 1.0, yaw: 0.0}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="duplicate waypoint id"):
        load_waypoints(path)


def test_load_waypoints_rejects_non_finite_pose(tmp_path: Path):
    path = tmp_path / "points.yaml"
    path.write_text(
        "waypoints:\n"
        "  - {id: wall_a, x: .nan, y: 1.0, yaw: 0.0}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="finite"):
        load_waypoints(path)
```

在 `setup.py` 注册 `pytest` 测试依赖后运行：

```bash
cd ~/water-inspection-robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select inspection_mission --event-handlers console_direct+
```

Expected: 初始测试因配置/解析函数缺失而失败；错误是预期的导入或断言失败，不是测试环境错误。

- [ ] **Step 3: 创建第一版巡检点 YAML 和解析函数**

`inspection_points.yaml` 使用 `id`, `x`, `y`, `yaw` 四个字段，至少包含修正道路上的 3 至 5 个有效导航点。具体格式见下方 YAML。用 `yaml.safe_load` 读文件，拒绝空列表/重复 ID，并检查每个 pose 数值 finite，使 Step 2 测试通过。`setup.py` 的 `install_requires` 加入 `PyYAML`；不要在该阶段加入相机或缺陷识别字段。

`config.py` 的最小解析实现：

```python
import math
from pathlib import Path

import yaml


def load_waypoints(path: str | Path) -> list[dict[str, float | str]]:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    points = data.get("waypoints") if isinstance(data, dict) else None
    if not points:
        raise ValueError("waypoints must be a non-empty list")

    ids = [point.get("id") for point in points]
    if any(not item for item in ids) or len(ids) != len(set(ids)):
        raise ValueError("duplicate waypoint id or missing id")

    for point in points:
        for key in ("x", "y", "yaw"):
            value = point.get(key)
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"waypoint {point['id']} pose must be finite")
    return points
```

文件采用以下明确格式，第一条路线先运行 5 个点；wall 点朝向墙面，转角后的点沿 B 段前进：

```yaml
waypoints:
  - {id: start, x: 5.0, y: 0.0, yaw: 0.0}
  - {id: wall_01, x: 20.0, y: -0.8, yaw: -1.57079632679}
  - {id: wall_02, x: 40.0, y: -0.8, yaw: -1.57079632679}
  - {id: turn_exit, x: 60.0, y: 5.0, yaw: 1.57079632679}
  - {id: north_end, x: 60.0, y: 38.0, yaw: 1.57079632679}
```

运行前按 RViz 中实际地图验证点位；若固定障碍物或 footprint 侵入目标容差范围，只调整 waypoint 坐标，不移动真实墙体/禁入区来迁就错误点位。

- [ ] **Step 4: 重跑测试并构建工作区**

```bash
cd ~/water-inspection-robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test --packages-select inspection_mission --event-handlers console_direct+
colcon test-result --verbose
```

Expected: build 成功，测试结果为 100% pass。

### Task 6: 导出静态地图与 KeepoutFilter mask

**Files:**
- Create: `ros2_ws/src/inspection_bringup/maps/inspection_map.yaml`
- Create: `ros2_ws/src/inspection_bringup/maps/inspection_map.pgm`
- Create: `ros2_ws/src/inspection_bringup/maps/keepout_mask.yaml`
- Create: `ros2_ws/src/inspection_bringup/maps/keepout_mask.pgm`
- Create: `ros2_ws/src/inspection_bringup/test/test_map_metadata.py`

- [ ] **Step 1: 从当前 USD 导出 occupancy grid**

在 Isaac Sim 4.5 中使用该版本自带的 Occupancy Map 工作流，以场景俯视 XY 平面生成 ROS map_server 可读的 PGM + YAML。设置分辨率 `0.10 m/cell`，地图覆盖 `x=-5..65 m, y=-5..45 m`。导出后记录 YAML 的 `resolution`、`origin`、`negate`、`occupied_thresh` 和 `free_thresh`。

- [ ] **Step 2: 修正可通行范围**

在地图编辑器中只将 3 m 道路设为 free；墙、固定障碍物和道路外区域设为 lethal/occupied 或 unknown，不得将整个地面 prim 烘焙成可通行区域。用 RViz `Map` 显示检查地图方向、原点和道路轮廓。

- [ ] **Step 3: 创建同分辨率 Keepout mask**

基于同一地图原点和分辨率创建 mask，禁入区沿危险侧及转角连续覆盖；mask 外形对禁入区留出 robot footprint 的安全缓冲。用 RViz 同时显示 map 与 mask，逐像素确认二者尺寸和原点匹配。根据 KeepoutFilter 示例约定确认“禁入值”后再启动导航，不从颜色外观推断数值语义。

- [ ] **Step 4: 写地图元数据测试**

在 `test_map_metadata.py` 中用 PyYAML 读取两份 YAML，并用 Pillow 打开图片；断言地图与 mask 的分辨率、宽高和原点完全一致，图片存在且为同尺寸灰度图：

```python
from pathlib import Path

import yaml
from PIL import Image


def test_map_and_mask_metadata_match():
    root = Path(__file__).parents[1] / "maps"
    map_meta = yaml.safe_load((root / "inspection_map.yaml").read_text())
    mask_meta = yaml.safe_load((root / "keepout_mask.yaml").read_text())
    assert map_meta["resolution"] == mask_meta["resolution"]
    assert map_meta["origin"] == mask_meta["origin"]
    map_image = Image.open(root / map_meta["image"]).convert("L")
    mask_image = Image.open(root / mask_meta["image"]).convert("L")
    assert map_image.size == mask_image.size
```

运行：

```bash
cd ~/water-inspection-robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select inspection_bringup --event-handlers console_direct+
colcon test-result --verbose
```

Expected: metadata tests pass；若图片尺寸/原点不一致，测试应在集成 Nav2 前失败。

### Task 7: 接通机器人、仿真时钟和真值定位

**Files:**
- Modify: `isaac/robots/inspection_carter.usda`
- Modify: `docs/environment-baseline.md`

- [ ] **Step 1: 从 Isaac 官方 Navigation 示例复制并简化控制图**

在 Isaac Sim 中从官方 Carter/Nav2 示例复制 ROS 2 Subscribe Twist、差速控制器、Odometry Publisher 和 TF Publisher 所需节点，命令输入使用 `/cmd_vel`，输出 `/odom` 和 `odom -> base_link`。禁用示例世界/机器人专有的话题重映射。保存机器人文件时使用相对资产引用。

- [ ] **Step 2: 明确真值模式 TF 所有权**

真值模式下由 Isaac ROS 2 Bridge 发布 `/odom` 和 `odom -> base_link`；bringup 只发布恒等 `map -> odom`；`robot_state_publisher` 或固定 TF 发布器发布 `base_link -> sensor_link`。AMCL 必须关闭。不要再启动会重复发布 `odom -> base_link` 的控制节点。

- [ ] **Step 3: 验证时钟、控制和 TF**

在仿真 Play 状态下运行：

```bash
source /opt/ros/humble/setup.bash
timeout 5s ros2 topic hz /clock
timeout 5s ros2 topic hz /odom
timeout 5s ros2 run tf2_ros tf2_echo map base_link
```

另开终端测试运动，运行 3 秒后按 `Ctrl+C`：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

Expected: `/clock` 和 `/odom` 持续更新，`map` 到 `base_link` transform 可查询，机器人缓慢向前移动并在停止命令后停住。通过 RViz 的 TF display 核对 frame 名称，不接受重复 transform warning。

### Task 8: 配置 Nav2 静态地图导航与禁入区

**Files:**
- Modify: `ros2_ws/src/inspection_bringup/config/nav2_params.yaml`
- Modify: `ros2_ws/src/inspection_bringup/launch/navigation.launch.py`
- Create: `ros2_ws/src/inspection_bringup/rviz/inspection.rviz`

- [ ] **Step 1: 以 ROS 2 Humble TurtleBot/Nav2 默认配置为底稿**

复制 Humble `nav2_bringup` 中适用于差速机器人的参数配置到包内，再只修改 `use_sim_time`, `map`, `odom`, `base_link`, `scan` frame、robot footprint、控制速度上限和地图文件路径。初始最高线速度设为 `0.25 m/s`、最高角速度设为 `0.5 rad/s`；robot radius/footprint 必须从所选 USD 实际尺寸量取后填写。

- [ ] **Step 2: 配置全局静态层和 KeepoutFilter**

全局 costmap 启用 static layer 和 KeepoutFilter，过滤 mask 及 filter info 使用 Task 6 的文件和 Nav2 Humble 示例参数。阶段 A 不启用依赖 `/scan` 的 obstacle layer。局部 costmap 使用 static layer 和 robot footprint；AMCL 阶段再加入 scan obstacle layer。

Humble 参数结构按下面骨架配置（完整默认 planner/controller 参数从本机 `nav2_bringup` 差速配置复制，只覆盖明确列出的 frame、footprint 和速度字段）：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      plugins: [static_layer, inflation_layer]
      filters: [keepout_filter]
      keepout_filter:
        plugin: nav2_costmap_2d::KeepoutFilter
        enabled: true
        filter_info_topic: /costmap_filter_info
```

同一 launch 必须启动 `map_server`、filter mask map server 和 costmap filter info server；三者都纳入 lifecycle manager。`filter_info_topic`、mask topic、filter type/base/multiplier 按 Humble KeepoutFilter 官方教程的参数名配置。

- [ ] **Step 3: 启动 map_server 和 Nav2**

```bash
cd ~/water-inspection-robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch inspection_bringup navigation.launch.py localization_mode:=truth
```

Expected: lifecycle nodes 激活；RViz 同时显示 map、costmap、robot TF；Nav2 不报 transform extrapolation、missing frame 或 map load error。

- [ ] **Step 4: 用 RViz 单点验证路径与 KeepoutFilter**

先在道路中部下发一个 2D Goal 并确认到达，再把目标放进 keepout 区。道路目标必须成功，禁入目标必须被全局规划器拒绝或显示不可达；机器人任何时刻不可驶入红色标记区。保留两个测试位置截图供后续回归。

### Task 9: 运行多巡检点路线

**Files:**
- Create: `ros2_ws/src/inspection_mission/inspection_mission/waypoint_runner.py`
- Modify: `ros2_ws/src/inspection_mission/setup.py`
- Modify: `ros2_ws/src/inspection_mission/test/test_inspection_points.py`
- Create: `ros2_ws/src/inspection_mission/launch/inspection.launch.py`

- [ ] **Step 1: 测试 waypoint 转 pose 的行为**

扩展单元测试：配置中的 `yaw` 转换为单位四元数；missed index 能映射回 waypoint ID。先运行测试确认新增函数行为失败，再实现下面两个无 ROS 依赖的纯函数：

测试断言写为：`yaw_to_quaternion(0.0) == (0.0, 1.0)`；对 `pi/2` 检查 `z` 和 `w` 分别近似 `sin(pi/4)`、`cos(pi/4)`；`missed_waypoint_ids(["a", "b", "c"], [2, 0]) == ["c", "a"]`。

```python
import math


def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def missed_waypoint_ids(waypoint_ids: list[str], missed_indices: list[int]) -> list[str]:
    return [waypoint_ids[index] for index in missed_indices]
```

- [ ] **Step 2: 实现最小 FollowWaypoints 客户端**

实现一个 ROS 2 Python 节点，从 `config/inspection_points.yaml` 顺序构造 `nav2_msgs/action/FollowWaypoints` goal；使用 `ActionClient` 等待 `follow_waypoints` server、发送一次完整目标列表、等待 action result。纯函数 `yaw_to_quaternion(yaw)` 用 `z=sin(yaw/2)`, `w=cos(yaw/2)`；结果解析把每个 `missed_waypoints[].index` 映射回 waypoint ID。日志目录用 `datetime.now().strftime("%Y%m%d-%H%M%S")` 创建在 `~/water-inspection-robot/runs/` 下，每行 JSON 包含 waypoint ID、结果状态、仿真时间和目标 pose；有 missed waypoint 时任务整体返回非零状态。将 `runs/` 和 `isaac/scenes/shots/` 加入 `.gitignore`，日志和渲染截图不提交。

action 客户端按此顺序实现，避免在节点构造函数里阻塞 executor：初始化节点与 `ActionClient` → `wait_for_server(timeout_sec=30.0)` → 从 YAML 创建 `PoseStamped(frame_id="map")` → `send_goal_async` → `spin_until_future_complete` 等待接受状态 → 等待 `get_result_async()` → 将 `missed_waypoints` 转成失败 ID 并写 JSONL → 销毁节点。服务器未启动、goal 被拒绝或 action 失败时以非零退出码结束，并写明失败原因。

`inspection.launch.py` include `inspection_bringup/navigation.launch.py` 并原样传递 `localization_mode`，然后启动 FollowWaypoints 客户端，因此单条任务命令只启动一个 Nav2 实例。Task 8 单点检查继续直接运行 `navigation.launch.py`。

- [ ] **Step 3: 运行任务测试**

```bash
cd ~/water-inspection-robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test --packages-select inspection_mission --event-handlers console_direct+
colcon test-result --verbose
```

Expected: unit tests pass；配置缺字段、重复 ID 或非法 yaw 时有具体错误消息。

- [ ] **Step 4: 从起点跑完路线**

先在 RViz 设置初始位姿，然后执行：

```bash
ros2 launch inspection_mission inspection.launch.py localization_mode:=truth
```

Expected: 3 至 5 个巡检点依序完成并写入 JSONL；目标之间允许 Nav2 自主规划绕开地图内静态障碍物。到达点只记录位置，不触发拍照或缺陷推理。

### Task 10: 加入激光雷达与 AMCL

**Files:**
- Modify: `isaac/robots/inspection_carter.usda`
- Modify: `ros2_ws/src/inspection_bringup/config/nav2_params.yaml`
- Modify: `ros2_ws/src/inspection_bringup/launch/navigation.launch.py`
- Create: `ros2_ws/src/inspection_bringup/test/test_localization_modes.py`

- [ ] **Step 1: 给机器人加入模拟 2D 激光扫描**

在机器人前部安装 16 线 RTX Lidar 并按 Isaac Sim 4.x 官方 ROS 2 Bridge 示例发布 PointCloud2，再用 Humble `pointcloud_to_laserscan` 转成 `/scan`。设置 10 Hz，固定 frame 为 `laser_frame`，并通过固定 TF 增加 `base_link -> laser_frame`。

- [ ] **Step 2: 加入 scan 观测检查**

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo base_link laser_frame
```

Expected: scan 有非空 ranges、频率接近设置值，LaserScan frame 能通过 TF 变换，RViz 中点云/扫描线落在墙和障碍物上。

- [ ] **Step 3: 建立互斥定位模式**

`navigation.launch.py` 提供 `localization_mode` 的 `truth` 和 `amcl` 两个合法值。truth 模式启动恒等 `map -> odom`，关闭 AMCL；amcl 模式启动 `nav2_amcl` 并关闭恒等 `map -> odom` 发布器。测试两种模式不会同时启动两个 `map -> odom` 发布者；未知模式启动前报错退出。

amcl 模式还必须关闭 Isaac 的真值 odometry publisher，改由差速控制器的运动/wheel odometry 发布 `/odom` 和 `odom -> base_link`；AMCL 只负责发布 `map -> odom`。真值模式和 amcl 模式可以共用 `/cmd_vel`、`/odom`、`/scan` 话题名，但各自的定位数据来源必须在 launch 中互斥。

- [ ] **Step 4: 先在静止和低速条件下调 AMCL**

以手动设置初始位姿启动 amcl 模式；在 RViz 检查 map、laser scan 和 robot footprint 对齐。先原地旋转，再以 `0.1 m/s` 行走 5 m；确认定位不会持续漂移、跳变或丢失。根据激光角分辨率、地图分辨率和机器人 footprint 调整 AMCL 与 costmap 参数，不用真值 TF 掩盖定位问题。

- [ ] **Step 5: 用同一路线回归 AMCL**

```bash
ros2 launch inspection_mission inspection.launch.py localization_mode:=amcl
```

Expected: 所有巡检点在 AMCL 模式下成功；mission log 记录完成状态；RViz 显示地图和扫描匹配一致。若 AMCL 不稳定，先回到 Task 8 truth 模式排除场景/控制回归，再单独检查 `/scan`、TF、时钟和地图方向。

### Task 11: SLAM Toolbox 在线建图（可选）

**Files:**
- Create: `ros2_ws/src/inspection_bringup/launch/mapping.launch.py`
- Create: `ros2_ws/src/inspection_bringup/config/slam_params.yaml`
- Create: `ros2_ws/src/inspection_bringup/README.md`

- [ ] **Step 1: 以 AMCL 已通过为前置条件启动同步建图**

按 ROS 2 Humble SLAM Toolbox `online_sync` 示例配置 2D 激光模式，使用 `/scan`、`odom`、`base_link` 和仿真时钟。另开终端运行 `ros2 run teleop_twist_keyboard teleop_twist_keyboard`，手动遥控机器人沿路线完整走一圈，避免首次就让 Nav2 在未知地图中自主探索。

- [ ] **Step 2: 保存并复载地图**

```bash
mkdir -p ~/water-inspection-robot/runs/maps
ros2 run nav2_map_server map_saver_cli -f ~/water-inspection-robot/runs/maps/slam_map
```

重启 Nav2 localization 模式，指定新地图，设置初始位姿并跑完同一路线。用 `sha256sum ~/water-inspection-robot/runs/maps/slam_map.pgm ~/water-inspection-robot/runs/maps/slam_map.yaml` 记录地图摘要。Expected: 地图可重复载入；道路轮廓闭合；定位成功。

- [ ] **Step 3: 记录是否采用 SLAM**

在 `docs/isaac-sim-water-inspection-roadmap.md` 记录静态地图和 SLAM 的差异、地图质量和路线结果。SLAM 有明确现场需求且重复建图质量满足导航时才保留为后续路径；它不是 MVP 的完成门槛。

### Task 12: 回归验收与操作手册

**Files:**
- Create: `docs/run-navigation-demo.md`
- Modify: `README.md`
- Create: `runs/`（运行结果，不提交 Git）

- [ ] **Step 1: 冻结一次可复现运行配置**

记录场景 USD 生成命令、Isaac Sim 版本、ROS 2/Nav2 包版本、机器人资产版本、地图 SHA256、`ROS_DOMAIN_ID`、启动终端顺序和 RViz 配置。将项目自己的 `ROS_DOMAIN_ID` 固定为 `42`，确认所有进程一致。

- [ ] **Step 2: 连续执行 10 次真值巡检**

每次重置仿真到相同起点，运行完整 waypoint 任务，保存 JSONL。通过条件：10 次全部到达配置中的每个点；无碰撞；无 keepout 侵入；单点定位误差在 0.35 m 和 15 度以内；每次都有完成/失败状态记录。

- [ ] **Step 3: 连续执行 10 次 AMCL 巡检**

使用相同场景、地图和 waypoint 再运行 10 次。通过条件同真值模式；允许单次初始化后由 AMCL 定位，不允许真值 `map -> odom` 同时运行。保存成功率、失败点和 RViz 截图。

- [ ] **Step 4: 编写从干净终端复现的操作手册**

`docs/run-navigation-demo.md` 按顺序写明启动 Isaac Sim、source ROS 环境、build workspace、启动 truth 或 amcl、打开 RViz、设置 initial pose、启动 waypoints、检查日志和安全停止。README 增加该手册、当前设计和执行计划的链接。

- [ ] **Step 5: 最终检查并提交**

```bash
cd ~/water-inspection-robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
cd ~/water-inspection-robot
git diff --check
git status --short
```

Expected: build/tests 全部通过；版本记录完整；仓库不包含生成的 USD 大文件、bag、截图或 mission 运行日志，除非项目明确选择纳入版本控制。

## 计划自检

- 设计覆盖：Ubuntu/Humble/Isaac Sim 4.x/RTX 3090、差速底盘抽象、现有 L 形场景、静态地图、禁入区、Waypoint Follower、真值定位、AMCL 和可选 SLAM 均有任务及验收门。
- TF 所有权：truth 模式由仿真提供 `odom -> base_link`、bringup 提供恒等 `map -> odom`；AMCL 模式由仿真提供运动里程计 TF、AMCL 提供 `map -> odom`，启动模式互斥。
- 地图安全：道路外区域不可通行，静态障碍在地图中标记，KeepoutFilter mask 与主地图共用分辨率和原点。
- 场景拓扑：验证 `Road_B` 与 `Road_A` 在 `(leg_a, 0)` 连通，B 段巡检点和窄通道随 `leg_a` 平移。
- 占位项扫描：没有待补内容、占位路径或未定义的测试命令；Isaac Sim 补丁版由 Task 1 的官方兼容核对确定，后续安装和命令使用记录的版本。
- 明确不在此计划中的工作：缺陷识别、合成数据训练、四足步态、坡面行走和真机部署。
