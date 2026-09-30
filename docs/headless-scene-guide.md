# 无头（headless）模式下在 Isaac Sim 里搭场景

配套脚本：[build_scene.py](build_scene.py)

---

## 1. 核心思路

**没有 GUI，场景就不是"拖出来"的，而是"写出来"的。**

```
场景 = 一个 Python 脚本 + 一份参数表  →  生成一个 .usd 文件
         （可版本管理、可复现、可批量出变体）
```

这条路不仅可行，而且比手工拖拽更适合你的项目：
- SDG 需要成百上千个场景变体 → 参数化脚本天生适合；
- 场景要进 git → `.usda` 是文本，能 diff；
- 云端和本地跑同一份脚本 → 场景完全一致。

---

## 2. 三种搭建方式

| 方式 | 做法 | 适用 |
|---|---|---|
| **A. 脚本生成**（推荐） | Python + USD API 在 headless 下建场景并保存 | 本项目主路线 |
| **B. 本地 GUI 搭好上传** | 本地有显卡的机器用 GUI 搭，存 USD，上传到云端跑 | 快速起步、复杂造型 |
| **C. 直接写 .usda 文本** | `.usda` 是纯文本，编辑器手写/改 | 微调坐标、加语义标签 |

实际操作里三者混用最舒服：**用 A 打底，用 B 做复杂部分，用 C 打补丁**。

---

## 3. 跑起来

```bash
# 1) 上传脚本到数据盘
scp build_scene.py root@<host>:/root/autodl-tmp/work/

# 2) 执行（用 Isaac Sim 自带的 python.sh，不要用系统 python）
cd /root/autodl-tmp/work
$ISAACSIM_PATH/python.sh build_scene.py \
    --out /root/autodl-tmp/work/water_inspection.usd \
    --usda --shots

# 3) 取回验收图和 USD
#    图片：/root/autodl-tmp/work/shots/{top,along_a,wall,corner}/*.png
#    场景：/root/autodl-tmp/work/water_inspection.usd
```

脚本会输出：

```
[check] metersPerUnit = 1.0  (应为 1.0)
[check] upAxis        = Z    (应为 Z)
[check] prim 数量      = 27
[check] 场景包围盒     = min (...) max (...)
[ok] 场景已保存: ...
[ok] 验收图已输出到: ./shots/
```

---

## 4. 没有 GUI，怎么验收场景对不对

三层验证，从便宜到贵：

**第一层：数值自检（秒级）**
- `metersPerUnit == 1.0`、`upAxis == Z`（错了后面全乱）
- 包围盒尺寸：L 形道路应该是 60 × 40 m 量级
- prim 数量、命名前缀是否符合规范

**第二层：渲几张图下载看（分钟级）**
脚本里已经内置 4 个机位：俯视全局 / 沿路视角 / 贴墙视角（看裂缝 ROI）/ 转弯处。
**这是无头环境下最重要的一招**——渲染不需要显示器，Replicator 自己创建渲染产物。

**第三层：加载进物理世界跑几步**
```python
# 加一个重力下落的小方块，确认没穿模、地面有碰撞
```
物理能跑通，说明碰撞体、物理场景、单位都没问题。

---

## 5. 搭完场景之后

| 下一步 | 用场景做什么 |
|---|---|
| 传感器仿真 | 在墙上/狗上挂相机、RTX Lidar |
| Replicator SDG | 以这个 USD 为底座，随机化裂缝材质 + 光照 + 视角 |
| Nav2 地图 | 从 USD 烘焙 2D 占据栅格（Occupancy Map 扩展），禁入区单独做 mask |
| ROS 2 联调 | 无头跑 Isaac Sim + Nav2，录 bag 回本地看 |

---

## 6. 坑

1. **`SimulationApp` 必须在 `import omni` / `import pxr` 之前创建**，顺序错了会崩。
2. **单位**：默认造出来的 stage 要显式设成米 + Z 轴向上；引用外部 USD 时也要确认对方的 `metersPerUnit`。
3. **静态物体要碰撞**：需要 `UsdPhysics.CollisionAPI`，并且 stage 里要有 `PhysicsScene`。
4. **headless 下没有视口**：别用依赖 Viewport 的 API，统一改用 Replicator 渲染产物。
5. **保存路径要在数据盘**：`/root/autodl-tmp/...`，系统盘会满。
6. **语义标签可能失败**：`Semantics` 模块需要对应扩展启用，脚本里已用 try/except 包住，不影响建场景。
7. **不同版本 API 有差异**：6.0 起 Core API 有迁移，本脚本刻意只用 pure USD + Replicator，遇到报错请对照你的版本文档。
8. **脚本结尾别忘了 `sim_app.close()`**，否则进程不退出，云端一直在计费。
