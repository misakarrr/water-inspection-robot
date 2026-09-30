"""
headless 场景生成脚本 —— 水利巡检场景
L 形巡检道路 + 混凝土墙 + 障碍物 + 禁入区

在无 GUI（无头）环境下把场景"写"出来并存成 USD，之后可以：
  - 直接用于 Isaac Sim 物理/传感器仿真
  - 作为 Replicator 合成数据的场景
  - 烘焙 2D 占据栅格给 ROS 2 Nav2 用

用法（Isaac Sim 自带的 python.sh）：
  $ISAACSIM_PATH/python.sh build_scene.py --out /root/autodl-tmp/work/water_inspection.usd --shots

注意：API 在不同 Isaac Sim 版本间有差异（6.0 起 Core API 有迁移），
      本脚本只用「pure USD (pxr) + Replicator」这些跨版本稳定的接口，
      首次运行请对照你的版本文档核对报错信息。
"""

import argparse

# ---------------------------------------------------------------- 场景参数
CFG = dict(
    # 道路：L 形，A 段沿 +X，B 段沿 +Y，宽 3 m、厚 10 cm
    road_width=3.0,
    road_thickness=0.1,
    leg_a=60.0,               # 直道 A 长度
    leg_b=40.0,               # 直道 B 长度
    # 混凝土墙（检测目标），沿 A 段单侧
    wall_x0=10.0,             # 墙起点（沿 X）
    wall_length=40.0,
    wall_height=2.5,
    wall_thickness=0.3,
    wall_gap=0.15,            # 墙与路边的间隙
    # 禁入区（靠水一侧的条带）
    keepout_width=3.0,
    # 障碍物
    cone_count=5,
    cone_h=0.7, cone_r=0.15,
    narrow_pass_width=1.2,    # 窄通道净宽
)

ap = argparse.ArgumentParser()
ap.add_argument("--out", default="water_inspection.usd", help="输出 USD 路径")
ap.add_argument("--shots", action="store_true", help="渲染验收图（无 GUI 下用来看场景）")
ap.add_argument("--shotdir", default="./shots", help="验收图输出目录")
ap.add_argument("--usda", action="store_true", help="同时导出 .usda 文本版（便于 git diff）")
args = ap.parse_args()

# ---------------------------------------------------------------- 1. 启动 Isaac Sim
# headless=True 表示不创建窗口；必须在 import omni / pxr 之前执行
try:
    from isaacsim import SimulationApp            # Isaac Sim 4.5+
except ImportError:
    from omni.isaac.kit import SimulationApp      # Isaac Sim 4.0 ~ 4.2

sim_app = SimulationApp({"headless": True})

import omni.usd                                    # noqa: E402
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Gf, Sdf   # noqa: E402

# ---------------------------------------------------------------- 2. 工具函数
def new_stage():
    """新建空 stage，并统一单位与朝向（Isaac Sim 约定：米、Z 轴向上）"""
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    return stage


def make_material(stage, path, color, roughness=0.7, metallic=0.0):
    """用 UsdPreviewSurface 建材质（纯 USD，跨版本稳定）"""
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def box(stage, path, size, center, material=None, collide=False, semantic=None):
    """建一个长方体（size/center 都是米）"""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)                       # 单位立方体，-0.5 ~ +0.5
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))           # 缩放 = 实际尺寸

    if material is not None:
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if semantic:
        _tag_semantics(cube.GetPrim(), semantic)
    return cube


def _tag_semantics(prim, label):
    """打语义标签（供 Replicator 分割标注 / 后续查询使用）。失败不影响建场景。"""
    try:
        from pxr import Semantics
        api = Semantics.SemanticsAPI.Apply(prim, "Semantics")
        api.CreateSemanticTypeAttr().Set("class")
        api.CreateSemanticDataAttr().Set(label)
    except Exception as e:                          # 扩展未启用时跳过
        print(f"[warn] 语义标签跳过 {prim.GetPath()}: {e}")


def ref_asset(stage, path, asset_usd, position):
    """引用官方/外部 USD 资产（无需 GUI）"""
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(asset_usd)
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*position))
    return prim


# ---------------------------------------------------------------- 3. 建场景
stage = new_stage()
stage.DefinePrim("/World", "Xform")
stage.DefinePrim("/World/physicsScene", "PhysicsScene")   # 静态碰撞体需要有物理场景

mat_ground = make_material(stage, "/World/Looks/Ground", (0.35, 0.33, 0.30))
mat_road   = make_material(stage, "/World/Looks/Road",   (0.45, 0.45, 0.47), roughness=0.9)
mat_wall   = make_material(stage, "/World/Looks/Wall",   (0.72, 0.71, 0.68), roughness=0.85)
mat_cone   = make_material(stage, "/World/Looks/Cone",   (0.90, 0.35, 0.05))
mat_rock   = make_material(stage, "/World/Looks/Rock",   (0.40, 0.38, 0.36))
mat_keepout = make_material(stage, "/World/Looks/Keepout", (0.85, 0.10, 0.10), roughness=0.5)

W = CFG["road_width"]
T = CFG["road_thickness"]

# 3.1 地面（禁入区之外的可通行大地面）
box(stage, "/World/Environment/Ground", (200, 200, 0.1), (0, 0, -0.05),
    mat_ground, collide=True, semantic="ground")

# 3.2 L 形道路：A 段（沿 X）+ B 段（沿 Y），在原点附近相接
box(stage, "/World/Environment/Road_A", (CFG["leg_a"], W, T),
    (CFG["leg_a"] / 2, 0, T / 2), mat_road, collide=True, semantic="road")
box(stage, "/World/Environment/Road_B", (W, CFG["leg_b"], T),
    (0, CFG["leg_b"] / 2, T / 2), mat_road, collide=True, semantic="road")

# 3.3 混凝土墙：沿 A 段、在 -Y 侧（检测 ROI，后续贴裂缝材质）
wall_y = -(W / 2 + CFG["wall_gap"] + CFG["wall_thickness"] / 2)
box(stage, "/World/Environment/Wall_Crackable",
    (CFG["wall_length"], CFG["wall_thickness"], CFG["wall_height"]),
    (CFG["wall_x0"] + CFG["wall_length"] / 2, wall_y, CFG["wall_height"] / 2),
    mat_wall, collide=True, semantic="wall_surface")

# 3.4 禁入区：靠水一侧（+Y）的条带，仅作视觉标记；
#     Nav2 用的栅格 mask 在后续步骤单独烘焙（可通行=254 / 禁入=0）
ko_y = W / 2 + CFG["keepout_width"] / 2
box(stage, "/World/Zones/Keepout_Strip",
    (CFG["leg_a"], CFG["keepout_width"], 0.02),
    (CFG["leg_a"] / 2, ko_y, T + 0.011),
    mat_keepout, collide=False, semantic="keepout")

# 3.5 障碍物：锥桶沿 A 段错位摆放 + 一个窄通道 + 几块落石
for i in range(CFG["cone_count"]):
    x = 8.0 + i * 6.0
    y = (-1.0 if i % 2 == 0 else 1.0) * (W / 2 - 0.6)
    box(stage, f"/World/Obstacles/Cone_{i+1:02d}",
        (CFG["cone_r"] * 2, CFG["cone_r"] * 2, CFG["cone_h"]),
        (x, y, CFG["cone_h"] / 2), mat_cone, collide=True, semantic="obstacle")

# 窄通道：两块挡板，净宽 narrow_pass_width（放在 B 段上）
half_gap = CFG["narrow_pass_width"] / 2
plate_w = (W - CFG["narrow_pass_width"]) / 2
for sign, name in ((-1, "L"), (1, "R")):
    box(stage, f"/World/Obstacles/NarrowPass_{name}",
        (plate_w, 0.3, 1.0),
        (sign * (half_gap + plate_w / 2), 20.0, 0.5),
        mat_rock, collide=True, semantic="obstacle")

# 落石
for i, (x, y, s) in enumerate([(22.0, 0.8, 0.5), (35.0, -0.9, 0.4), (48.0, 0.6, 0.6)]):
    box(stage, f"/World/Obstacles/Rock_{i+1:02d}", (s, s, s), (x, y, s / 2),
        mat_rock, collide=True, semantic="obstacle")

# 3.6 引用官方资产（示例，按需打开；先把资产根目录配好）
# ISAAC_ASSETS = "/root/autodl-tmp/isaac_assets/Assets/Isaac/6.1"
# ref_asset(stage, "/World/Obstacles/OfficialCone",
#           f"{ISAAC_ASSETS}/Isaac/Props/Klt/cone.usd", (12.0, 0.0, 0.0))

# 3.7 巡检点标记（后续 Nav2 waypoint 用）
for i, (x, y) in enumerate([(5, 0), (20, 0), (40, 0), (55, 0), (0, 20), (0, 38)]):
    p = UsdGeom.Sphere.Define(stage, f"/World/Zones/InspectionPoint_{i+1:02d}")
    p.CreateRadiusAttr(0.15)
    UsdGeom.Xformable(p.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(x, y, T + 0.2))
    _tag_semantics(p.GetPrim(), "inspection_point")

# ---------------------------------------------------------------- 4. 自检（无 GUI 下的数值验收）
bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
world = stage.GetPrimAtPath("/World")
rng = bbox.ComputeWorldBounds(world)
print("=" * 60)
print(f"[check] metersPerUnit = {UsdGeom.GetStageMetersPerUnit(stage)}  (应为 1.0)")
print(f"[check] upAxis        = {UsdGeom.GetStageUpAxis(stage)}        (应为 Z)")
print(f"[check] prim 数量      = {len(list(stage.Traverse()))}")
print(f"[check] 场景包围盒     = min {rng.GetMin()}  max {rng.GetMax()}")
print("=" * 60)

# ---------------------------------------------------------------- 5. 保存
omni.usd.get_context().save_as_stage(args.out)
print(f"[ok] 场景已保存: {args.out}")

if args.usda:
    usda_path = args.out.rsplit(".", 1)[0] + ".usda"
    stage.GetRootLayer().Export(usda_path)      # 文本格式，便于 git diff
    print(f"[ok] 文本版已导出: {usda_path}")

# ---------------------------------------------------------------- 6. 渲染验收图（替代 GUI）
if args.shots:
    import os
    import omni.replicator.core as rep

    os.makedirs(args.shotdir, exist_ok=True)
    rep.orchestrator.set_capture_on_play(False)

    views = [
        ("top",     (30.0, 20.0, 70.0), (30.0, 20.0, 0.0)),   # 俯视全局
        ("along_a", (-8.0, 0.0, 1.6),   (30.0, 0.0, 1.0)),    # 沿 A 段看
        ("wall",    (20.0, 3.5, 1.5),   (25.0, wall_y, 1.2)), # 贴墙看（检查裂缝 ROI）
        ("corner",  (10.0, 10.0, 3.0),  (0.0, 0.0, 0.5)),     # 转弯处
    ]
    for name, pos, look in views:
        cam = rep.create.camera(position=pos, look_at=look)
        rp = rep.create.render_product(cam, (1280, 720))
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(output_dir=os.path.join(args.shotdir, name), rgb=True)
        writer.attach([rp])
        rep.orchestrator.run()
    print(f"[ok] 验收图已输出到: {args.shotdir}/  —— 下载到本地查看即可")

# ---------------------------------------------------------------- 7. 退出
sim_app.close()
