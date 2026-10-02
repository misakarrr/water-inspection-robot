# 在 AutoDL 私有云（RTX 4090）上跑 Isaac Sim — 详细方案

> **已弃用（2026-10-02）**：本文档基于旧基线 Isaac Sim 6.1 编写，当前导航原型已改为 Ubuntu 22.04 + ROS 2 Humble + Isaac Sim 4.x + RTX 3090 本地仿真，本文仅作云 GPU 部署的历史参考。
> 后续如需用 AutoDL 跑 Replicator / 训练，请按当前 Isaac Sim 补丁版本的官方兼容矩阵重新核对驱动与安装方式。当前设计见 [superpowers/specs/2026-10-02-water-inspection-nav-design.md](superpowers/specs/2026-10-02-water-inspection-nav-design.md)。

> 适用场景：实验室自建（或租用）的 AutoDL 风格 GPU 云，提供 RTX 4090 容器实例。
> 目标：把 Isaac Sim 跑起来，用于水利巡检机器狗项目的**合成数据生成（Replicator）、模型训练、Nav2 无头联调**。
> 基准：Isaac Sim 6.1（若驱动不达标则降级，见第 1 节）。

---

## 0. 一句话结论（先看这个）

**4090 完全满足 Isaac Sim 的硬件要求（官方最低是 4080，4090 是 24 GB 显存 + Ada 第 3 代 RT Core），
但 AutoDL 类平台的三个特性决定了你的用法：**

| AutoDL 特性 | 后果 | 应对 |
|---|---|---|
| 实例**没有独立公网 IP**，通常只映射 6006/6008 两个端口 | **WebRTC 串流 GUI 基本不可用**（需要 TCP 49100 + 大段 UDP） | 走**无头（headless）作业模式**，这是主路线 |
| 系统盘通常只有 30 GB | Isaac Sim 装不下（磁盘爆满） | 一切装到 `/root/autodl-tmp` 数据盘 |
| 实例本身是容器，**一般不能再跑 Docker** | 不能用官方 `nvcr.io/nvidia/isaac-sim` 镜像 | 用**预编译二进制包**安装 |

**所以：AutoDL 上正确的定位是"无头算力节点"——跑数据生成、训练、无 GUI 的 ROS 2 联调；
需要看三维视图的调试放在本地或实验室工作站做。**

---

## 1. 第 0 步：先确认三件事（决定后面所有选择）

### ① 宿主机驱动版本 → 决定装哪个 Isaac Sim 版本

```bash
nvidia-smi          # 看右上角 Driver Version
```

| 驱动版本 | 可选 Isaac Sim | 说明 |
|---|---|---|
| ≥ 580.65.06 | **6.x**（推荐 6.1） | 官方 6.x 要求 |
| 535 ~ 580 | 4.5 等较早版本 | 4.5 要求 ≥ 535.129.03 |
| < 535 | 都跑不了 | 只能请管理员升级宿主机驱动 |

> ⚠️ **容器内改不了驱动**（驱动属于宿主机内核模块）。驱动不达标时，**降 Isaac Sim 版本**是唯一自助方案。
> 以官方 [Requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html) 页面对应版本为准。

### ② 网络：有没有独立公网 IP / 能不能开放 UDP 端口段

- 若**有独立公网 IP 且可开放端口**：可以尝试 WebRTC 串流 GUI（第 5 节方案 C）。
- 若**只有 6006/6008 映射、其余靠 SSH 隧道**（AutoDL 公网版就是这样）：**放弃串流**，走无头模式。
- 隧道只转发 TCP，而 WebRTC 需要 UDP 47995-48012 / 49000-49007 → TCP 隧道救不了串流。

### ③ 权限：是否 root、能否嵌套 Docker、数据盘多大

```bash
whoami; id -u
df -h                       # 重点看 / 和 /root/autodl-tmp
docker info 2>/dev/null || echo "no docker"
```

- 能跑 Docker → 直接用 `nvcr.io/nvidia/isaac-sim` 镜像，省去手工装依赖。
- 不能跑 Docker（多数情况）→ 用二进制包（第 2 节方案 A）。

---

## 2. 安装：两条路线

### 路线 A（推荐）：预编译二进制包

**为什么推荐**：pip 安装在这类平台上极易撑爆系统盘（实测 `~/.cache/pip` 能到 11 GB 且
`TMPDIR`/`PIP_CACHE_DIR` 改了也未必够），二进制包解压即用、可控。

```bash
# 1) 网络加速（AutoDL 提供）
source /etc/network_turbo

# 2) 准备目录（全部放在数据盘！）
mkdir -p /root/autodl-tmp/isaac && cd /root/autodl-tmp/isaac

# 3) 获取二进制包（~8-10 GB）
#    建议：本地/实验室网络下载 → 通过平台网盘、公开数据盘或 scp 上传
#    下载页：https://docs.isaacsim.omniverse.nvidia.com/latest/installation/download.html

# 4) 解压
unzip "isaac-sim-*-linux-x86_64.zip" -d /root/autodl-tmp/isaac/sim
cd /root/autodl-tmp/isaac/sim
ls post_install.sh && ./post_install.sh      # 若包内有此脚本，先跑一次

# 5) 冒烟测试（root 必须加 --allow-root）
export ISAACSIM_PATH=/root/autodl-tmp/isaac/sim
$ISAACSIM_PATH/isaac-sim.sh --allow-root --no-window --/app/quitAfter=50
```

看到加载完成且无致命报错即可。等价的 Python 方式：

```bash
$ISAACSIM_PATH/python.sh -c "from isaacsim import SimulationApp; app=SimulationApp({'headless':True}); print('OK'); app.close()"
```

### 路线 B：pip 安装（仅在能跑 Docker/磁盘充足时）

```bash
source /etc/network_turbo
export PIP_CACHE_DIR=/root/autodl-tmp/pipcache
export TMPDIR=/root/autodl-tmp/tmp
mkdir -p $PIP_CACHE_DIR $TMPDIR

# 关键：venv 建在数据盘，否则 site-packages 会写爆系统盘
python3 -m venv /root/autodl-tmp/venv-isaac
source /root/autodl-tmp/venv-isaac/bin/activate

pip install --upgrade pip
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
```

> pip 路线要预留 **≥40 GB** 空间（下载 + 解包 + 安装 + 扩展缓存）。
> 装完先跑第 2 节的冒烟测试。

### 资产包（Isaac Sim Assets）的处理

Isaac Sim 自带资产很小，**官方资产库是另一个几十 GB ~ 100 GB 的包**，必须单独处理：

```bash
mkdir -p /root/autodl-tmp/isaac_assets && cd /root/autodl-tmp/isaac_assets
# 同样：本地下载 → 网盘/公开数据 → 解压

# 启动时或写进 kit 配置，指向资产根目录：
$ISAACSIM_PATH/isaac-sim.sh --allow-root --no-window \
  --/persistent/isaac/asset_root/default="/root/autodl-tmp/isaac_assets/Assets/Isaac/6.1"
```

> 只做裂缝 SDG 的话，其实**不需要整包**——用自建 USD + 少量官方 props 就够，
> 资产包可以按需下载，省几十 GB 磁盘和上传时间。

---

## 3. 无头作业模式（主用法）

### 3.1 启动方式

```bash
# GUI 版（无显示器时用 --no-window）
$ISAACSIM_PATH/isaac-sim.sh --allow-root --no-window

# 纯脚本（推荐）：SimulationApp(headless=True)
$ISAACSIM_PATH/python.sh /root/autodl-tmp/work/my_sdg.py
```

`my_sdg.py` 骨架：

```python
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})      # 必须在最前面

import omni.usd, omni.replicator.core as rep
# ... 打开场景、配相机、随机化、写数据 ...

app.close()                                  # 别忘了，否则进程不退出
```

### 3.2 后台跑长任务

```bash
tmux new -s sdg
$ISAACSIM_PATH/python.sh /root/autodl-tmp/work/sdg_crack.py 2>&1 | tee /root/autodl-tmp/logs/sdg_$(date +%m%d).log
# Ctrl+B 然后 D 脱离；tmux attach -t sdg 回来看
```

```bash
# 或者 nohup
nohup $ISAACSIM_PATH/python.sh /root/autodl-tmp/work/sdg_crack.py \
  > /root/autodl-tmp/logs/sdg.log 2>&1 &
```

### 3.3 计费策略（省钱关键）

```
无卡模式/低配开机 → 上传二进制包与资产、装依赖、调试脚本（不烧 GPU 钱）
        ↓
切换 4090 开机 → 跑 SDG / 训练 → 产出写数据盘
        ↓
下载结果（网盘 / scp / 公开数据盘）→ 立刻关机
```

- SDG 是"跑一夜出几万张"的批处理，**最适合这种按小时计费的实例**。
- 训练裂缝检测模型（YOLO-seg）在 4090 上很快，2 万张图几小时量级。
- 别让实例空转挂着——这是实验室经费最常见的浪费。

### 3.4 "看不到场景"怎么办

不需要 GUI 也能验收：**用相机传感器渲染出图**，导出 PNG 到数据盘再看。

```python
# Replicator 出图就是"可视化"：先渲 1 张，确认裂缝可见、比例正确
rep.orchestrator.step()
# 图片落在 writer 指定的 output_dir，下载后本地查看即可
```

---

## 4. 与实验室本地机器协同（ROS 2 + Nav2）

### 4.1 方案一（推荐，最简单）：全部跑在实例内

Isaac Sim + Nav2 + 推理节点都在同一个 4090 实例里，本地只做两件事：

- 用 **SSH 隧道**看 RViz？——不行，RViz 也要 GUI。

所以落地做法是：

| 在实例内（无头） | 在本地 |
|---|---|
| Isaac Sim + 传感器桥 + Nav2 + 巡检任务节点 | 无（或只跑轻量 RViz 做离线回放） |
| 用 `ros2 bag record` 录制 TF / scan / odom / 相机 | 下载 bag → 本地 `rviz2 -d xxx.rviz` 回放检查 |
| 出图、出日志、出 JSON 报告 | 看结果、调参、写代码 |

> **这个"录制 bag → 本地回放"的技巧非常实用**：无头实例负责跑，本地负责看，
> 既绕开 GUI 限制，又不牺牲调试能力。

### 4.2 方案二：跨机组网（Tailscale / EasyTier / ZeroTier）

把实例和实验室工作站放进同一个虚拟内网，然后：

```bash
export ROS_DOMAIN_ID=42           # 两边一致，且与别人隔离
export ROS_LOCALHOST_ONLY=0
# 跨公网组播不可用 → 用静态 peer 或 discovery server
export ROS_STATIC_PEERS=<对端虚拟IP>
```

注意：跨公网传图像会吃掉带宽，**相机流用压缩传输（`image_transport`）或干脆只传事件**。

### 4.3 DDS 选择

云端无组播，建议 **CycloneDDS** 或 **FastDDS discovery server** 模式，
否则 `ros2 topic list` 会互相看不到。

---

## 5. 图形界面的三条路线（按可行性排序）

### 方案 A：不要 GUI（推荐）
见 3.4 与 4.1，用出图 + bag 回放替代。

### 方案 B：VNC over SSH 隧道（可行性中等，仅轻量场景）

原理：VNC 走 **TCP 5900**，而 SSH 隧道只支持 TCP → 这条路能通。

```bash
# 实例内（一次性）
apt update && apt install -y xvfb x11vnc virtualgl mesa-utils

# 启动虚拟显示 + VNC
Xvfb :1 -screen 0 1920x1080x24 &
export DISPLAY=:1
x11vnc -display :1 -forever -shared -rfbport 5900 -nopw &

# 用 VirtualGL 把 OpenGL 渲染交给 4090
vglrun $ISAACSIM_PATH/isaac-sim.sh --allow-root
```

```bash
# 本地（Windows 图形工具或命令行）
ssh -CNg -L 5900:127.0.0.1:5900 root@<实例host> -p <实例端口>
# 然后 VNC Viewer 连 127.0.0.1:5900
```

> ⚠️ 注意：Isaac Sim 依赖 Vulkan + RTX 渲染，VirtualGL 对它的支持并不完美，
> 这一步是"能试但不保证"，且帧率会很低。**只用于偶尔看一眼场景，不要作为日常开发方式。**

### 方案 C：WebRTC 串流（只在私有云开放了端口时才考虑）

需要同时满足：

1. 实例有**独立公网 IP 或可开放自定义端口**；
2. 能放行 **TCP 49100 + UDP 47995-48012 + UDP 49000-49007**；
3. Isaac Sim ≥ 6.0（5.0 不支持改串流端口）。

```bash
$ISAACSIM_PATH/isaac-sim.sh --allow-root --no-window \
  --/exts/omni.kit.livestream.app/primaryStream/publicIp=<公网IP> \
  --/exts/omni.kit.livestream.app/primaryStream/signalPort=49100 \
  --/exts/omni.kit.livestream.app/primaryStream/streamPort=47998
```

若端口受限但能和本地组网（Tailscale），可把 `publicIp` 设为虚拟网卡 IP 再试——
**属于需要实测的偏方，不要写进实验室的正式流程**。

---

## 6. 团队化使用（对应"多用户"需求）

AutoDL 类平台的最大好处：**每个成员一个实例 = 天然的多用户方案**，不需要在一台机器上挤。

| 事项 | 做法 |
|---|---|
| 环境一致性 | 配好一个实例后**保存为自定义镜像**，新成员直接用镜像开实例（省 1-2 小时安装） |
| 资产共享 | 把 Isaac Sim 资产包放到平台的**文件存储 / 公共数据盘**，各实例只读挂载，避免每人一份 100 GB |
| 代码共享 | Git（GitHub / 实验室 GitLab）；不要用共享目录改同一份 USD |
| 数据交换 | 平台网盘 / 对象存储；SDG 输出直接写共享存储，省得来回下载 |
| 版本矩阵 | 在实验室群里固定一张表：驱动版本 / Isaac Sim 版本 / ROS 2 版本 / Isaac Lab 版本 |
| 端口冲突 | 同一实例内多服务用不同端口；不同实例互不影响 |
| 计费 | 约定"跑完即关机"；用 `tmux` 跑长任务避免 SSH 断线中断 |

---

## 7. 部署检查清单

**环境**
- [ ] `nvidia-smi` 记录驱动版本，确定 Isaac Sim 版本
- [ ] `df -h` 确认数据盘容量（建议 ≥200 GB 可用）
- [ ] `source /etc/network_turbo` 能访问外网源
- [ ] 二进制包已上传并解压到数据盘
- [ ] 冒烟测试通过：`isaac-sim.sh --allow-root --no-window --/app/quitAfter=50`
- [ ] 资产根目录已指向数据盘

**作业**
- [ ] SDG 脚本能在 headless 下跑通并出图
- [ ] 输出目录在数据盘，磁盘水位有监控
- [ ] 用 tmux/nohup 跑，日志落盘
- [ ] 跑完自动关机（或人工确认关机）

**协同**
- [ ] ROS 2 版本、`ROS_DOMAIN_ID`、DDS 配置与本地一致
- [ ] bag 录制/回放流程验证通过
- [ ] 环境已保存为自定义镜像
- [ ] 资产与数据集放在共享存储

---

## 8. 常见坑（按踩坑频率）

1. **系统盘爆满**：pip 缓存 + 解压 + 安装全在系统盘 → 所有路径指到 `/root/autodl-tmp`。
   （官方 FAQ 也有"[系统盘空间不足](https://api.autodl.com/docs/qa1/)"条目。）
2. **驱动版本不够**：报 `CUDA driver version is insufficient` → 降 Isaac Sim 版本，别折腾驱动。
3. **忘了 `--allow-root`**：容器内是 root，Isaac Sim 会拒绝启动。
4. **想用 WebRTC 却发现连不上**：实例无独立公网 IP、UDP 段没开 → 回到无头模式。
5. **以为能用官方 Docker 镜像**：实例本身是容器，一般不能嵌套 Docker。
6. **资产包没装/路径没配**：场景加载失败或机器人变白块。
7. **进程不退出**：脚本结尾忘了 `app.close()`，实例一直占着 GPU 计费。
8. **SSH 断线任务中断**：长任务必须 `tmux` 或 `nohup`。
9. **跨机 ROS 2 看不到话题**：无组播环境没配 DDS 静态发现。
10. **多人各存一份资产包**：浪费几百 GB 存储，改成共享存储只读挂载。

---

## 9. 落地节奏建议（与水利巡检项目对齐）

| 阶段 | 在 AutoDL 上做什么 | 在本地做什么 |
|---|---|---|
| W1 环境 | 装二进制包、跑通 headless 冒烟测试、保存镜像 | 装 ROS 2 / Nav2，跑通基础例子 |
| W2 场景 | 用脚本生成 L 形道路 + 墙 + 障碍 USD（无头） | 下载 USD 到本地查看、调整 |
| W3-W4 导航 | 实例内无头跑 Isaac Sim + Nav2，录 bag | 本地回放 bag 调 Nav2 参数 |
| W6-W7 SDG | **主力用途**：Replicator 批量出裂缝/渗水数据 | 抽查图片质量 |
| W8 训练 | 4090 上训 YOLO-seg / U-Net | 本地部署推理节点 |
| W9-W10 闭环 | 无头跑完整巡检流程 + 出报告 | 本地可视化验收 |

---

## 10. 参考链接

- Isaac Sim 系统要求：https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html
- 下载页：https://docs.isaacsim.omniverse.nvidia.com/latest/installation/download.html
- pip 安装（Python Environment）：https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html
- 容器安装（若允许 Docker）：https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html
- AutoDL 开放端口说明：https://api.autodl.com/docs/port/
- AutoDL SSH 隧道：https://api.autodl.com/docs/ssh_proxy/
- AutoDL 学术资源加速：https://api.autodl.com/docs/network_turbo/
- AutoDL 系统盘空间不足 FAQ：https://api.autodl.com/docs/qa1/
- 社区实践（AutoDL 上配置 Isaac Sim，二进制包 + headless）：https://www.cnblogs.com/LLLLLL123456/p/19239172
