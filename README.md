# water-inspection-robot · 水利巡检机器狗

基于 **NVIDIA Isaac Sim + ROS 2 Nav2** 的水利设施自主巡检机器狗项目。

- **场景**：大坝、堤防
- **任务**：沿坝顶 / 堤顶道路自主巡检，到巡检点采集坝面影像
- **病害识别**：混凝土裂缝、渗水（湿斑）——后续阶段

## 当前基线

| 项目 | 选型 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble + Nav2（后续 AMCL、SLAM Toolbox） |
| 仿真 | Isaac Sim 4.x，按 Humble 官方兼容矩阵固定补丁版（候选 4.5.0） |
| 本地仿真主机 | RTX 3090（24 GB 显存）Ubuntu 工作站 |

导航路线：先把四足机器狗简化为差速底盘，在固定地图上跑通真值定位的 Nav2 巡检点导航，再接入激光雷达 + AMCL，SLAM 作为独立可选阶段。四足步态与病害识别（Replicator 合成数据 + 训练）留在后续。

> 仓库早期文档曾以 Isaac Sim 6.1 + ROS 2 Jazzy 为基线，该路线已弃用；导航相关决策以 `docs/superpowers/` 下的设计与计划为准。

## 目录结构

```
water-inspection-robot/
├── docs/                     # 参考文档
│   └── superpowers/
│       ├── specs/            # 当前导航设计
│       └── plans/            # 实施计划
├── isaac/
│   ├── scripts/              # 场景生成、Replicator 数据生成脚本
│   ├── scenes/               # 生成的场景 USD（可由脚本重建，不入库）
│   ├── robots/               # 机器狗封装 USD / 底盘配置
│   ├── materials/            # 裂缝、渗水材质
│   └── replicator/           # SDG 配置与产出
├── ros2_ws/src/              # ROS 2 功能包（inspection_bringup / inspection_mission / inspection_robot）
├── datasets/                 # 合成数据集（体积大，建议软链到大盘）
└── models/                   # 训练产物与权重
```

## 存储方案（本地 NVMe + NAS）

| 位置 | 内容 | 建议容量 |
| --- | --- | --- |
| 本地 NVMe | 系统、ROS 2、Isaac Sim 本体、资产包、Kit/着色器缓存、当前工作数据 | 250–300 GB（500 GB SSD 起步，1 TB 非必需） |
| NAS | Replicator 数据集、模型 checkpoint、bag、地图归档、仓库备份 | 2–4 TB |

- 不放 NAS：Isaac Sim 本体、资产根目录、Python 环境——USD/纹理是大量小文件随机读，SMB/NFS 延迟会拖慢启动和渲染
- Replicator 出图先写本地再 `rsync` 到 NAS；直接写几千个小文件到 NAS 会拖慢生成（除非 10GbE）
- 网络至少 2.5GbE；1GbE 只适合同步归档，不适合在线跑 SDG
- `datasets/`、`models/` 软链到 NAS 挂载点，保持仓库结构不变：

```bash
ln -s /mnt/nas/water-inspection/datasets datasets
ln -s /mnt/nas/water-inspection/models models
```

## 文档

**当前导航原型**

| 文档 | 内容 |
| --- | --- |
| [导航原型设计](docs/superpowers/specs/2026-10-02-water-inspection-nav-design.md) | 基线、场景范围、分阶段定位（真值 / AMCL / SLAM）、验收目标 |
| [导航实施计划](docs/superpowers/plans/2026-10-02-water-inspection-nav.md) | 12 个 Task：环境 → 场景 → 控制 → 真值导航 → AMCL → 可选 SLAM |

**参考文档**（首页带弃用标注的以标注为准）

| 文档 | 状态 | 内容 |
| --- | --- | --- |
| [docs/headless-scene-guide.md](docs/headless-scene-guide.md) | 有效（批量/远程） | 无头（云端 / 无显示器 / 批处理）搭场景的方式、验收方法、坑 |
| [docs/isaac-sim-water-inspection-roadmap.md](docs/isaac-sim-water-inspection-roadmap.md) | 已弃用 | 旧路线图：硬件选型、12 周计划、SDG 与训练参考 |
| [docs/isaac-sim-autodl-4090-plan.md](docs/isaac-sim-autodl-4090-plan.md) | 已弃用 | 旧基线的 AutoDL 云 GPU 部署方案 |

## 场景脚本

`isaac/scripts/build_scene.py` —— 无头生成巡检场景（L 形道路 + 混凝土墙 + 障碍 + 禁入区 + 巡检点），
同时输出 4 张验收视角图（俯视 / 沿路 / 贴墙 / 转弯）。

```bash
$ISAACSIM_PATH/python.sh isaac/scripts/build_scene.py \
    --out isaac/scenes/water_inspection.usd --usda --shots \
    --shotdir isaac/scenes/shots
```
