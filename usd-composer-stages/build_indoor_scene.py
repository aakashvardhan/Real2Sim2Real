"""Procedurally authors a simple indoor room scene for the SO-101 workshop.

Z-up, meters, matching SO-ARM101-USD.usd exactly (see docs/isaac-sim-windows-guide.md
section on the Composer-default-template Y-up/cm mismatch this sidesteps).

Room: 4m x 4m floor, 2.5m wall height, centered at the origin. A flat mount
plate at the origin marks where the SO-101 tabletop rig (lightbox + mat, see
task_env_cfg.py) should be placed -- the robot's own root is at (-0.05, 0, 0)
in that rig's local frame, so dropping the rig at the mount plate's origin
lines both conventions up directly.
"""
import sys

from pxr import Usd, UsdGeom, UsdShade, UsdLux, UsdPhysics, Sdf, Gf

OUT_PATH = sys.argv[1]

ROOM_SIZE = 4.0       # meters, floor is ROOM_SIZE x ROOM_SIZE
WALL_HEIGHT = 2.5      # meters
WALL_THICKNESS = 0.1   # meters
MOUNT_RADIUS = 0.6     # meters, visual marker plate for the tabletop rig footprint

stage = Usd.Stage.CreateNew(OUT_PATH)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())


def make_material(name, color):
    mat = UsdShade.Material.Define(stage, f"/World/Looks/{name}")
    shader = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


floor_mat = make_material("Floor", (0.55, 0.55, 0.58))
wall_mat = make_material("Wall", (0.85, 0.84, 0.80))
mount_mat = make_material("Mount", (0.9, 0.55, 0.1))


def make_box(path, center, size, material):
    """Axis-aligned box authored as a Cube prim (unit cube scaled per-axis)."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(size[0], size[1], size[2]))
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim())
    UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


half = ROOM_SIZE / 2.0

# Floor: thin box so it has real collision thickness, top surface sits at z=0.
make_box(
    "/World/Floor",
    center=(0, 0, -WALL_THICKNESS / 2.0),
    size=(ROOM_SIZE, ROOM_SIZE, WALL_THICKNESS),
    material=floor_mat,
)

# Four walls around the floor perimeter, centered on wall height.
wall_z = WALL_HEIGHT / 2.0
make_box("/World/Walls/North", (0, half, wall_z), (ROOM_SIZE, WALL_THICKNESS, WALL_HEIGHT), wall_mat)
make_box("/World/Walls/South", (0, -half, wall_z), (ROOM_SIZE, WALL_THICKNESS, WALL_HEIGHT), wall_mat)
make_box("/World/Walls/East", (half, 0, wall_z), (WALL_THICKNESS, ROOM_SIZE, WALL_HEIGHT), wall_mat)
make_box("/World/Walls/West", (-half, 0, wall_z), (WALL_THICKNESS, ROOM_SIZE, WALL_HEIGHT), wall_mat)

# Robot mount marker: flat plate at the world origin, matching the robot's own
# root position/orientation convention in so101.py (pos=(-0.05, 0, 0), yaw=90deg).
# It's a visual/authoring aid, not a physical object -- no collision applied.
mount = UsdGeom.Cylinder.Define(stage, "/World/RobotMount")
mount.CreateRadiusAttr(MOUNT_RADIUS)
mount.CreateHeightAttr(0.02)
mount.CreateAxisAttr(UsdGeom.Tokens.z)
xf = UsdGeom.Xformable(mount)
xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.01))
UsdShade.MaterialBindingAPI.Apply(mount.GetPrim())
UsdShade.MaterialBindingAPI(mount.GetPrim()).Bind(mount_mat)

# Lighting: sky dome for ambient fill + one rect light as an overhead key light.
dome = UsdLux.DomeLight.Define(stage, "/World/SkyLight")
dome.CreateIntensityAttr(800.0)
dome.CreateColorTemperatureAttr(6500.0)
dome.CreateEnableColorTemperatureAttr(True)

key = UsdLux.RectLight.Define(stage, "/World/KeyLight")
key.CreateIntensityAttr(6000.0)
key.CreateWidthAttr(1.5)
key.CreateHeightAttr(1.5)
key_xf = UsdGeom.Xformable(key)
key_xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, WALL_HEIGHT - 0.05))
# UsdLux rect lights emit along local -Z by default, which is already
# straight down here -- no rotation needed.

# Reference the robot in for a visual alignment check (task 3). The raw
# SO-ARM101-USD.usd file's own default prim already bakes in a 90deg yaw via
# its root orient op (confirmed by inspection), so only the (-0.05, 0, 0)
# spawn offset from task_env_cfg.py needs to be added here, not the rotation.
robot_xform = UsdGeom.Xform.Define(stage, "/World/Robot")
robot_xform.AddTranslateOp().Set(Gf.Vec3d(-0.05, 0.0, 0.01))
robot_xform.GetPrim().GetReferences().AddReference(
    "../source/sim_to_real_so101/assets/usd/SO-ARM101-USD.usd"
)

stage.GetRootLayer().Save()
print(f"Wrote {OUT_PATH}")
print(f"Prims: {[str(p.GetPath()) for p in stage.Traverse()]}")
