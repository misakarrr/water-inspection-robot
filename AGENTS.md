# AGENTS.md

给在此仓库工作的编码代理的项目说明。

## 项目

水利巡检机器狗：Isaac Sim 仿真 + ROS 2 Nav2，让机器狗沿坝顶 / 堤顶道路自主巡检，到巡检点采集坝面影像；病害识别（裂缝 / 渗水）在后续阶段接入。

当前基线（2026-10-02，权威文档见 `docs/superpowers/specs/2026-10-02-water-inspection-nav-design.md`）：

- Ubuntu 22.04 + ROS 2 Humble + Nav2
- Isaac Sim 4.x，按 Humble 官方兼容矩阵固定补丁版（候选 4.5.0）
- RTX 3090（24 GB）Ubuntu 工作站做本地仿真
- 分阶段定位：truth（仿真真值）→ 激光雷达 + AMCL，SLAM 为可选独立阶段

**旧基线 Isaac Sim 6.1 + ROS 2 Jazzy 已弃用**，不要套用 6.x / Jazzy 的 API 与教程；`docs/` 下带"已弃用"标注的文档仅作参考。

## 仓库结构

- `isaac/scripts/build_scene.py`：GUI / 无头生成 L 形巡检场景（pure USD + Replicator）
- `isaac/scenes/`、`isaac/robots/`、`isaac/materials/`、`isaac/replicator/`：生成物与资产（大部分尚未开始）
- `ros2_ws/src/`：ROS 2 包 `inspection_bringup`、`inspection_mission`、`inspection_robot`（计划中，尚未创建）
- `docs/superpowers/specs|plans/`：当前导航设计与实施计划，冲突时以此为准
- `datasets/`、`models/`：大文件，软链到 NAS，不进 Git

## 运行环境

本 checkout 的日常编辑在 Windows 上；**Isaac Sim、ROS 2、RViz 只能在 Ubuntu 3090 工作站运行**（仓库路径 `~/water-inspection-robot`）。Windows 上只做编辑与语法检查：

```powershell
python -m py_compile isaac/scripts/build_scene.py
```

## 常用命令（Ubuntu 3090）

场景生成（有显示器用默认 GUI，远程 / 无显示器加 `--headless`）：

```bash
$ISAACSIM_PATH/python.sh isaac/scripts/build_scene.py \
    --out isaac/scenes/water_inspection.usd --usda --shots \
    --shotdir isaac/scenes/shots
```

ROS 2 包创建后：

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test --packages-select inspection_mission --event-handlers console_direct+
colcon test-result --verbose
```

## 约定

- **按阶段提交**：每完成一个逻辑单元就提交，不攒大提交
- **推送须授权**：`git push` 必须先得到用户明确同意，不得自行推送
- 文档与提交说明可用中文；提交标题用英文 conventional commit 前缀：`docs:` / `feat:` / `fix:` / `chore:`
- 不提交生成物与大数据：USD/USDA/USDZ、bag、`ros2_ws/build|install|log`、`runs/`、验收截图、数据集、模型权重
- `build_scene.py` 只用 pure USD (pxr) + Replicator 接口，新场景代码沿用该风格以保持跨版本稳定
- 机器人资产不写死绝对路径；来源、尺寸与版本记录到 `isaac/robots/README.md`
- 结论性参数（点云、地图、点位）以 RViz / 实际数据验证为准，不接受仅凭 prim 数量或外观的验收

## 关键约束

- **版本锁定**：安装、示例与文档按 Isaac Sim 4.x 官方兼容矩阵选择，不与 6.x 混用
- **仿真时钟**：全链 `use_sim_time:=true`
- **TF 单一发布者**：truth 模式由仿真发布 `odom -> base_link`、bringup 发布恒等 `map -> odom`；AMCL 模式与 truth 模式互斥，禁止两个节点发布同一条动态 TF
- **地图安全**：道路外不可通行；keepout mask 与主地图同分辨率、同原点；场景改动必须同步重导地图
- **计划执行**：按 `docs/superpowers/plans/2026-10-02-water-inspection-nav.md` 的 P0–P5 验收门推进，P0 不过不调 Nav2，P2 不过不做路径规划，P3 不过不接 AMCL

## 测试

当前只有 `build_scene.py`，在 Isaac Sim 环境手动验收（数值自检 + 4 张验收图）。ROS 2 包创建后，单元测试放各包 `test/`（pytest / ament），改动 `inspection_mission` 或 `inspection_bringup` 必须跑对应 `colcon test`。
