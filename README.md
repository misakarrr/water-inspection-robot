# water-inspection-robot · 水利巡检机器狗

基于 **NVIDIA Isaac Sim + ROS 2 Nav2** 的水利设施自主巡检机器狗项目。

- **场景**：大坝、堤防
- **任务**：沿坝顶 / 堤顶道路自主巡检，到巡检点采集坝面影像
- **病害识别**：混凝土裂缝、渗水（湿斑）

## 目录结构

```
water-inspection-robot/
├── docs/                     # 路线图、部署方案、操作规程
├── isaac/
│   ├── scripts/              # 场景生成、Replicator 数据生成脚本
│   ├── scenes/               # 生成的场景 USD（可由脚本重建，不入库）
│   ├── robots/               # 机器狗封装 USD / 底盘配置
│   ├── materials/            # 裂缝、渗水材质
│   └── replicator/           # SDG 配置与产出
├── ros2_ws/src/              # ROS 2 功能包（bringup / mission / perception）
├── datasets/                 # 合成数据集（体积大，建议软链到大盘）
└── models/                   # 训练产物与权重
```

## 文档

| 文档 | 内容 |
| --- | --- |
| [docs/isaac-sim-water-inspection-roadmap.md](docs/isaac-sim-water-inspection-roadmap.md) | 总路线图：目标拆解、硬件选型、12 周计划、场景规格、Nav2 接入、坑清单 |
| [docs/isaac-sim-autodl-4090-plan.md](docs/isaac-sim-autodl-4090-plan.md) | AutoDL 私有云 4090 部署方案（无头运行、数据盘、网络与端口） |
| [docs/headless-scene-guide.md](docs/headless-scene-guide.md) | 无头模式下搭场景的三种方式、验收方法、坑 |

## 场景脚本

`isaac/scripts/build_scene.py` —— 无头生成巡检场景（L 形道路 + 混凝土墙 + 障碍 + 禁入区 + 巡检点），
同时输出 4 张验收视角图（俯视 / 沿路 / 贴墙 / 转弯）。

```bash
$ISAACSIM_PATH/python.sh isaac/scripts/build_scene.py \
    --out isaac/scenes/water_inspection.usd --usda --shots
```

## 环境角色

| 环节 | 机器 |
| --- | --- |
| 场景搭建 / 仿真 / SDG / 训练 | RTX 工作站或 AutoDL 4090 私有云（显存 ≥ 16 GB） |
| 部署 / 推理 | Jetson AGX Orin（Isaac Sim 本身不能跑在 Orin 上） |
| 本地轻量验证 | RTX 3060 Ti（仅够起场景，不用于训练） |

软件：Isaac Sim 6.1 + ROS 2（Humble / Jazzy）+ Nav2。
