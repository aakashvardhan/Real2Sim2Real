# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Real leader-arm teleop against raw Isaac Sim 6.0.1 -- NO Isaac Lab.

Same physics setup and platform constraint as keyboard_agent_raw_isaacsim.py
(see that file's module docstring and docs/aws-cube-to-bowl-teleop-plan.md
Sec.5) -- this is that script's keyboard jog replaced with a physical SO-101
leader arm read over serial, per the plan's Sec.5 "not yet done" list:

1. Reads `robot.get_action()` from a LeRobotSO101Interface(kind="leader")
   each tick instead of keyboard deltas.
2. LeRobotSO101Interface.get_mapped_actions_vectorized() targets *radians*
   (built for the Isaac-Lab task's action space) -- converted x180/pi here
   since this raw script's joint drives are authored in degrees.
3. Requires the vendored `lerobot` fork (lerobot-sim/, `.[so101]` extra)
   installed into whatever Python runs this script -- NOT present in a
   stock Isaac Sim python by default.
4. HF_LEROBOT_CALIBRATION is pointed at this repo's calibration/ folder
   (not the ~/.cache default) so `id="my_so_arm"` resolves to the existing
   calibration/teleoperators/so_leader/my_so_arm.json.

Run with a plain Isaac Sim Python that has lerobot installed (see repo docs
for the install command), by direct file path -- NOT `-m` (same reasoning
as keyboard_agent_raw_isaacsim.py):
    C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\leader_arm_teleop_raw_isaacsim.py

Scope (deliberately minimal, matching the keyboard version): direct
leader-arm-to-joint-target passthrough, plus a keyboard 'R' to reset the
cube/bowl to their original positions (joint targets aren't reset-able here
the way the keyboard script does it -- they're driven live by the leader arm
every tick, not held state). No recording, no grasp/placement detection.

Optional real-follower mirror (Phase 1+2 of
docs/dual-teleop-sim-and-follower-plan.md): pass --follower_port (e.g. COM3)
to also mirror the same leader reading, same tick, to a real SO-101 follower
via LeRobotSO101Interface(kind="follower").send_action(). Unset (default):
behavior is unchanged from the sim-only script described above.

Optional real-cube pose mirror (Phase 2 of
docs/object-pose-mirroring-plan.md): pass --track_camera (top camera's
OpenCV index) + --marker_size_m to make AWSBuilderCube a full kinematic
puppet of its camera-tracked real-world pose every tick, via ArUco
detection (source/sim_to_real_so101/utils/marker_tracking.py) against a
one-time camera calibration (calibrate_camera_intrinsics.py,
calibrate_camera_extrinsics.py). Needs Phase 0's physical marker placement
and camera calibration done first -- fails fast before Kit boots if the
calibration files aren't there. Unset (default): behavior is unchanged
from the sim-only script described above (dynamic, grippable, resettable
cube).
"""
import argparse
import os
import sys
import time

# Make sim_to_real_so101 importable without requiring an editable pip install
# in whatever Python this script is run with (matches
# keyboard_agent_raw_isaacsim.py -- this repo's package is only
# pip-installed into the Isaac-Lab venvs, not into a plain Isaac Sim python).
_REPO_SOURCE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_SOURCE_DIR not in sys.path:
    sys.path.insert(0, _REPO_SOURCE_DIR)
_REPO_ROOT_DIR = os.path.dirname(_REPO_SOURCE_DIR)

parser = argparse.ArgumentParser(description="Raw Isaac Sim SO-101 leader-arm teleop agent (no Isaac Lab).")
parser.add_argument("--headless", action="store_true", default=False, help="Run without a viewport window.")
parser.add_argument("--port", type=str, default=os.getenv("TELEOP_PORT", "COM4"), help="Leader arm serial port.")
parser.add_argument(
    "--robot_id",
    type=str,
    default=os.getenv("TELEOP_ID", "my_so_arm"),
    help="Leader arm calibration id (must match a file under calibration/teleoperators/so_leader/).",
)
parser.add_argument(
    "--calibration_dir",
    type=str,
    default=os.path.join(_REPO_ROOT_DIR, "calibration"),
    help="Root calibration directory (expects <this>/teleoperators/so_leader/<robot_id>.json). "
    "Sets HF_LEROBOT_CALIBRATION -- must be set before `lerobot` is imported.",
)
parser.add_argument(
    "--follower_port",
    type=str,
    default=None,
    help="Real follower arm serial port (e.g. COM3). Unset (default) = today's sim-only behavior, "
    "nothing else changed. When given, the same leader reading driving the sim is also mirrored "
    "to a real follower on this port each tick -- see docs/dual-teleop-sim-and-follower-plan.md.",
)
parser.add_argument(
    "--follower_robot_id",
    type=str,
    default="my_so_arm",
    help="Follower arm calibration id (must match a file under calibration/robots/so_follower/). "
    "Only used when --follower_port is given.",
)
parser.add_argument(
    "--track_camera",
    type=int,
    default=None,
    help="Top camera's OpenCV index. Unset (default) = today's unchanged behavior (dynamic, "
    "grippable, resettable AWSBuilderCube). When given, the cube becomes a kinematic puppet of "
    "its camera-tracked real-world pose every tick -- see docs/object-pose-mirroring-plan.md Phase 2.",
)
parser.add_argument(
    "--marker_size_m",
    type=float,
    default=None,
    help="Cube's printed ArUco marker side length, meters. Required when --track_camera is given.",
)
parser.add_argument(
    "--cube_marker_id", type=int, default=0, help="ArUco id printed on the cube's marker. Only used with --track_camera."
)
parser.add_argument(
    "--camera_calibration_dir",
    type=str,
    default=os.path.join(_REPO_ROOT_DIR, "calibration", "camera"),
    help="Directory with top_camera_intrinsics.json / top_camera_extrinsics.json (Phase 0 output of "
    "calibrate_camera_intrinsics.py / calibrate_camera_extrinsics.py). Only used with --track_camera.",
)
args_cli = parser.parse_args()

# Fail fast on a bad --robot_id/--calibration_dir (e.g. a typo) before paying
# for Kit's ~30s+ boot below -- lerobot's own error for a missing calibration
# file only surfaces later, inside connect(), with a much less obvious
# traceback originating deep in its serial handshake code.
_CALIBRATION_FILE = os.path.join(
    args_cli.calibration_dir, "teleoperators", "so_leader", f"{args_cli.robot_id}.json"
)
if not os.path.isfile(_CALIBRATION_FILE):
    print(f"[ERROR]: Calibration file not found: {_CALIBRATION_FILE}", file=sys.stderr)
    print(
        "[ERROR]: Check --robot_id / --calibration_dir (or TELEOP_ID / TELEOP_PORT env vars).",
        file=sys.stderr,
    )
    sys.exit(1)

if args_cli.follower_port is not None:
    _FOLLOWER_CALIBRATION_FILE = os.path.join(
        args_cli.calibration_dir, "robots", "so_follower", f"{args_cli.follower_robot_id}.json"
    )
    if not os.path.isfile(_FOLLOWER_CALIBRATION_FILE):
        print(f"[ERROR]: Follower calibration file not found: {_FOLLOWER_CALIBRATION_FILE}", file=sys.stderr)
        print("[ERROR]: Check --follower_robot_id / --calibration_dir.", file=sys.stderr)
        sys.exit(1)

if args_cli.track_camera is not None:
    if args_cli.marker_size_m is None:
        print("[ERROR]: --marker_size_m is required when --track_camera is given.", file=sys.stderr)
        sys.exit(1)
    _CAMERA_INTRINSICS_FILE = os.path.join(args_cli.camera_calibration_dir, "top_camera_intrinsics.json")
    _CAMERA_EXTRINSICS_FILE = os.path.join(args_cli.camera_calibration_dir, "top_camera_extrinsics.json")
    for _camera_calib_file in (_CAMERA_INTRINSICS_FILE, _CAMERA_EXTRINSICS_FILE):
        if not os.path.isfile(_camera_calib_file):
            print(f"[ERROR]: Camera calibration file not found: {_camera_calib_file}", file=sys.stderr)
            print(
                "[ERROR]: Run calibrate_camera_intrinsics.py then calibrate_camera_extrinsics.py first "
                "(see docs/object-pose-mirroring-plan.md Phase 0).",
                file=sys.stderr,
            )
            sys.exit(1)

# Must happen before any `lerobot` import (including transitively, via
# sim_to_real_so101.utils.lerobot_interface below) -- lerobot.utils.constants
# reads this env var at import time to compute HF_LEROBOT_CALIBRATION, and
# that constant is what resolves calibration/teleoperators/so_leader/<id>.json.
os.environ["HF_LEROBOT_CALIBRATION"] = args_cli.calibration_dir

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args_cli.headless})

"""Rest everything follows -- Kit is booted, pxr/omni/carb are importable now."""

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom, UsdPhysics  # noqa: E402

from sim_to_real_so101.utils.keyboard import KeyboardControl  # noqa: E402
from sim_to_real_so101.utils.lerobot_interface import LeRobotSO101Interface  # noqa: E402
from sim_to_real_so101.utils.physics_material import apply_friction_material  # noqa: E402
from sim_to_real_so101.utils.scene_reset import restore_prim_pose, snapshot_xform_ops  # noqa: E402
from sim_to_real_so101.utils.version_banner import print_simulator_version_banner  # noqa: E402

if args_cli.track_camera is not None:
    import cv2  # noqa: E402

    from sim_to_real_so101.utils.marker_tracking import (  # noqa: E402
        detect_markers,
        load_camera_calibration,
        marker_pose_to_world,
        solve_marker_pose_camera_frame,
    )

print_simulator_version_banner()

_DEMO_DIR = os.path.join(_REPO_SOURCE_DIR, "sim_to_real_so101", "demo")
REAL_TO_SIM_USD = os.path.join(_DEMO_DIR, "real-to-sim.usd")

ROBOT_PRIM_PATH = "/World/SO_ARM101_USD"
AWS_CUBE_PRIM_PATH = "/World/AWSBuilderCube"
AWS_CUBE_MASS_KG = 0.05
# See keyboard_agent_raw_isaacsim.py's identical constant -- friction is a
# collision-shape property (PhysicsMaterialAPI), not a rigid-body property.
AWS_CUBE_COLLISION_MESH_PATH = "/World/AWSBuilderCube/Geometry/AWSBuilderCube_Geo"
PAPER_BOWL_PRIM_PATH = "/World/PaperBowl"

# See keyboard_agent_raw_isaacsim.py -- gripper pad collision shapes.
GRIPPER_COLLISION_PATH = "/World/SO_ARM101_USD/gripper/collisions"
JAW_COLLISION_PATH = "/World/SO_ARM101_USD/jaw/collisions"

# See keyboard_agent_raw_isaacsim.py -- rigid PVC-like friction for the cube,
# chosen over PP (more slippery) since the goal is to stop the cube slipping
# out of the gripper. Neither cube nor gripper had any physics material
# before this.
AWS_CUBE_STATIC_FRICTION = 0.5
AWS_CUBE_DYNAMIC_FRICTION = 0.45
AWS_CUBE_RESTITUTION = 0.0

# See keyboard_agent_raw_isaacsim.py -- rubber/silicone-like friction for the
# gripper pads, higher than the cube's.
GRIPPER_STATIC_FRICTION = 0.9
GRIPPER_DYNAMIC_FRICTION = 0.8
GRIPPER_RESTITUTION = 0.0

# Exact xformOp:translate / xformOp:orient authored on /World/SO_ARM101_USD
# in real-to-sim.usd -- see keyboard_agent_raw_isaacsim.py's identical
# constant for the full derivation.
ROBOT_POS = Gf.Vec3f(0.0, 0.3, 0.72)

# Isaac-Lab-tuned actuator gains, used AS-IS unconverted on the raw
# PhysicsDriveAPI attributes -- see keyboard_agent_raw_isaacsim.py's
# JOINT_GAINS for the full empirical justification (unconverted values
# produce correct grasp torque; the naive pi/180-scaled version doesn't).
# Dict order here also doubles as the action-tensor order expected from
# LeRobotSO101Interface.get_mapped_actions_vectorized() -- both this and
# the Isaac-Lab task's ActionsCfg.joint_positions.joint_names use the same
# positional correspondence to SO101_JOINT_ORDER (shoulder_pan, shoulder_lift,
# elbow_flex, wrist_flex, wrist_roll, gripper), not a name-based mapping.
JOINT_GAINS = {
    "Rotation": dict(stiffness=55, damping=0.7, effort_limit=30),
    "Pitch": dict(stiffness=30, damping=0.8, effort_limit=30),
    "Elbow": dict(stiffness=25, damping=0.7, effort_limit=30),
    "Wrist_Pitch": dict(stiffness=12, damping=0.5, effort_limit=30),
    "Wrist_Roll": dict(stiffness=7, damping=0.5, effort_limit=30),
    # See keyboard_agent_raw_isaacsim.py's identical constant for the full
    # empirical justification -- effort_limit=30 (untuned for load) let the
    # jaw destabilize the contact solve and eject the cube when driven to
    # full closure against it. effort_limit=3 is still ~250x this 0.05kg
    # cube's actual holding-force requirement, so grip strength isn't lost.
    "Jaw": dict(stiffness=4, damping=0.3, effort_limit=3),
}
JOINT_ORDER = list(JOINT_GAINS.keys())
RAD_TO_DEG = 180.0 / 3.141592653589793

# Phase 2 timing measurement -- how often to flush the tick/send-time summary
# (see the main loop). Deliberately periodic, not per-tick, per the plan's
# "no per-tick spam" requirement.
STATS_PRINT_INTERVAL_S = 5.0


def main():
    usd_context = omni.usd.get_context()
    usd_context.open_stage(REAL_TO_SIM_USD)
    simulation_app.update()
    stage = usd_context.get_stage()

    # See keyboard_agent_raw_isaacsim.py -- the raw file authors no
    # RigidBodyAPI/MassAPI on the cube.
    cube_prim = stage.GetPrimAtPath(AWS_CUBE_PRIM_PATH)
    if not cube_prim.IsValid():
        raise RuntimeError(f"Expected prim not found: {AWS_CUBE_PRIM_PATH}")
    if not cube_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(cube_prim)
    if not cube_prim.HasAPI(UsdPhysics.MassAPI):
        UsdPhysics.MassAPI.Apply(cube_prim).CreateMassAttr(AWS_CUBE_MASS_KG)

    # Phase 2 of docs/object-pose-mirroring-plan.md: a kinematic rigid body
    # still participates in PhysX collision *response on other bodies* (the
    # gripper pads won't clip through it) but its own pose is driven
    # externally every tick instead of being force-integrated -- exactly
    # what a camera-tracked puppet needs. AWSBuilderCube authors only
    # xformOp:translate (verified 2026-08-18, no xformOp:orient) -- add one
    # so orientation can be written each tick too, matching PaperBowl's own
    # existing translate+orient+scale convention (float-precision quatf).
    cube_orient_op = None
    if args_cli.track_camera is not None:
        UsdPhysics.RigidBodyAPI(cube_prim).CreateKinematicEnabledAttr().Set(True)
        cube_xformable = UsdGeom.Xformable(cube_prim)
        for op in cube_xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                cube_orient_op = op
                break
        if cube_orient_op is None:
            cube_orient_op = cube_xformable.AddOrientOp()

    cube_collision_mesh = stage.GetPrimAtPath(AWS_CUBE_COLLISION_MESH_PATH)
    if not cube_collision_mesh.IsValid():
        raise RuntimeError(f"Expected prim not found: {AWS_CUBE_COLLISION_MESH_PATH}")
    apply_friction_material(
        cube_collision_mesh, AWS_CUBE_STATIC_FRICTION, AWS_CUBE_DYNAMIC_FRICTION, AWS_CUBE_RESTITUTION
    )

    for collision_path in (GRIPPER_COLLISION_PATH, JAW_COLLISION_PATH):
        collision_prim = stage.GetPrimAtPath(collision_path)
        if not collision_prim.IsValid():
            raise RuntimeError(f"Expected prim not found: {collision_path}")
        apply_friction_material(
            collision_prim, GRIPPER_STATIC_FRICTION, GRIPPER_DYNAMIC_FRICTION, GRIPPER_RESTITUTION
        )

    bowl_prim = stage.GetPrimAtPath(PAPER_BOWL_PRIM_PATH)
    if not bowl_prim.IsValid():
        raise RuntimeError(f"Expected prim not found: {PAPER_BOWL_PRIM_PATH}")

    # Captured before the timeline plays, so this is each prim's originally
    # authored pose -- restored on 'R' (cube skipped once --track_camera is
    # set, since software can't move the real object -- see the main loop).
    cube_orig_pose = snapshot_xform_ops(cube_prim)
    bowl_orig_pose = snapshot_xform_ops(bowl_prim)

    cube_translate_op = None
    if args_cli.track_camera is not None:
        for op in UsdGeom.Xformable(cube_prim).GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                cube_translate_op = op
                break
        if cube_translate_op is None:
            raise RuntimeError(f"Expected xformOp:translate on {AWS_CUBE_PRIM_PATH}, found none.")

    # See keyboard_agent_raw_isaacsim.py -- root_joint's body0 binds to the
    # physics scene's literal origin, not the ancestor Xform's real position.
    root_joint_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/root_joint")
    if not root_joint_prim.IsValid():
        raise RuntimeError(f"Expected prim not found: {ROBOT_PRIM_PATH}/root_joint")
    local_rot1 = root_joint_prim.GetAttribute("physics:localRot1").Get()
    root_joint_prim.GetAttribute("physics:localPos0").Set(ROBOT_POS)
    root_joint_prim.GetAttribute("physics:localRot0").Set(local_rot1)

    # Apply the tuned actuator gains and cache each joint's limits/attributes.
    joints = {}
    for joint_name, gains in JOINT_GAINS.items():
        joint_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/joints/{joint_name}")
        if not joint_prim.IsValid():
            raise RuntimeError(f"Expected joint prim not found: {joint_name}")

        joint_prim.GetAttribute("drive:angular:physics:stiffness").Set(float(gains["stiffness"]))
        joint_prim.GetAttribute("drive:angular:physics:damping").Set(float(gains["damping"]))
        joint_prim.GetAttribute("drive:angular:physics:maxForce").Set(float(gains["effort_limit"]))

        target_attr = joint_prim.GetAttribute("drive:angular:physics:targetPosition")
        joints[joint_name] = {
            "target_attr": target_attr,
            "lower": joint_prim.GetAttribute("physics:lowerLimit").Get(),
            "upper": joint_prim.GetAttribute("physics:upperLimit").Get(),
        }

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()

    robot_iface = LeRobotSO101Interface(
        device="cpu",
        port=args_cli.port,
        id=args_cli.robot_id,
        cameras={},
        fps=30,
        kind="leader",
    )
    try:
        robot_iface.init_device()
        robot_iface.connect()
    except Exception as exc:
        raise RuntimeError(
            f"Failed to connect to the leader arm on port={args_cli.port} id={args_cli.robot_id}: {exc}. "
            "Check the arm is powered, plugged into the given --port, and not already held open by "
            "another process (e.g. a previous run that didn't exit cleanly)."
        ) from exc
    connected = True

    # Follower mirror -- only connected when --follower_port is given (see
    # docs/dual-teleop-sim-and-follower-plan.md Phase 1). device="cuda" per
    # the plan: this machine has an NVIDIA GPU, unlike the leader interface
    # above which stays device="cpu" (unchanged, out of scope here).
    follower_iface = None
    follower_connected = False
    follower_send_failed = False
    if args_cli.follower_port is not None:
        follower_iface = LeRobotSO101Interface(
            device="cuda",
            port=args_cli.follower_port,
            id=args_cli.follower_robot_id,
            cameras={},
            fps=30,
            kind="follower",
        )
        try:
            follower_iface.init_device()
            follower_iface.connect()
            follower_connected = True
        except Exception as exc:
            raise RuntimeError(
                f"Failed to connect to the follower arm on port={args_cli.follower_port} "
                f"id={args_cli.follower_robot_id}: {exc}. Check the arm is powered, plugged into the "
                "given --follower_port, and not already held open by another process."
            ) from exc
        print(f"[INFO]: Follower arm connected: port={args_cli.follower_port} id={args_cli.follower_robot_id}")

    # Cube pose tracking -- only opened when --track_camera is given (see
    # docs/object-pose-mirroring-plan.md Phase 2). last_known_cube_pos/quat
    # start None and are only ever updated on a successful detection, so a
    # missed/occluded frame holds the cube at its last good pose instead of
    # snapping to origin or writing a NaN transform.
    cube_camera_capture = None
    cube_calibration = None
    last_known_cube_pos = None
    last_known_cube_quat = None
    if args_cli.track_camera is not None:
        cube_calibration = load_camera_calibration(_CAMERA_INTRINSICS_FILE, _CAMERA_EXTRINSICS_FILE)
        cube_camera_capture = cv2.VideoCapture(args_cli.track_camera)
        if not cube_camera_capture.isOpened():
            raise RuntimeError(
                f"Failed to open top camera index={args_cli.track_camera}. Check it's plugged in and "
                "not held open by another process (e.g. Windows Camera app -- a real cause seen "
                "getting this pipeline working, see docs/object-pose-mirroring-plan.md)."
            )
        print(
            f"[INFO]: Tracking cube (marker id={args_cli.cube_marker_id}) on top camera "
            f"index={args_cli.track_camera}."
        )

    keyboard_control = KeyboardControl()

    print_simulator_version_banner()
    print(f"[INFO]: Leader arm connected: port={args_cli.port} id={args_cli.robot_id}")
    print(f"[INFO]: HF_LEROBOT_CALIBRATION={os.environ['HF_LEROBOT_CALIBRATION']}")
    print("[INFO]: Driving joints from the leader arm. Ctrl+C or close the window to stop.")
    print("[INFO]: Click 'R' to reset the cube/bowl to their original positions")

    # Phase 2: measure, don't assume, the cost of a blocking follower serial
    # write on Kit's clock. Accumulated and flushed as a periodic summary
    # (not per-tick, to avoid log spam) so a --follower_port run's tick time
    # can be compared against a --follower_port-unset run.
    tick_count = 0
    tick_time_total = 0.0
    send_time_total = 0.0
    send_count = 0
    last_stats_print = time.perf_counter()

    try:
        while simulation_app.is_running():
            tick_start = time.perf_counter()

            if keyboard_control.reset_world:
                if cube_camera_capture is None:
                    restore_prim_pose(cube_prim, cube_orig_pose, zero_velocity=True)
                else:
                    print("[INFO]: Cube reset skipped -- camera-tracked, can't move the real object.")
                restore_prim_pose(bowl_prim, bowl_orig_pose)
                keyboard_control.reset_world = False

            try:
                real_action = robot_iface.robot.get_action()
            except Exception as exc:
                print(f"[ERROR]: Lost connection to the leader arm: {exc}")
                break

            raw_tensor = robot_iface.get_raw_actions_tensor(real_action)
            mapped_deg = robot_iface.get_mapped_actions_vectorized(raw_tensor) * RAD_TO_DEG

            for joint_name, target_deg in zip(JOINT_ORDER, mapped_deg.tolist()):
                info = joints[joint_name]
                clamped = max(info["lower"], min(info["upper"], target_deg))
                info["target_attr"].Set(clamped)

            # Cube pose puppet -- full kinematic override every tick, same
            # spirit as the follower-arm mirror below: the sim cube tracks
            # the real cube regardless of what the sim gripper is doing
            # (see docs/object-pose-mirroring-plan.md's "full kinematic
            # puppet" design decision). A miss this frame (occlusion,
            # marker out of view) just holds last_known_cube_pos/quat --
            # deliberately not an error.
            if cube_camera_capture is not None:
                cam_ok, cam_frame = cube_camera_capture.read()
                if cam_ok:
                    detected_markers = detect_markers(cam_frame)
                    if args_cli.cube_marker_id in detected_markers:
                        rvec, tvec = solve_marker_pose_camera_frame(
                            detected_markers[args_cli.cube_marker_id], args_cli.marker_size_m, cube_calibration
                        )
                        last_known_cube_pos, last_known_cube_quat = marker_pose_to_world(
                            rvec, tvec, cube_calibration
                        )
                if last_known_cube_pos is not None:
                    px, py, pz = last_known_cube_pos.tolist()
                    cube_translate_op.Set(Gf.Vec3d(px, py, pz))
                    qw, qx, qy, qz = last_known_cube_quat.tolist()
                    cube_orient_op.Set(Gf.Quatf(qw, qx, qy, qz))

            # Mirror the same leader reading to the real follower, same tick,
            # unmodified raw dict -- no unit conversion needed, real_action is
            # already in the native lerobot key space a follower's
            # send_action() expects. See dual-teleop-sim-and-follower-plan.md
            # Phase 1 (why no conversion) and Phase 2 (why the try/except and
            # the timing measurement below).
            if follower_connected and not follower_send_failed:
                send_start = time.perf_counter()
                try:
                    follower_iface.robot.send_action(real_action)
                except Exception as exc:
                    follower_send_failed = True
                    print(
                        f"[ERROR]: Lost connection to the follower arm, no longer mirroring to it "
                        f"for the rest of this run (sim continues normally): {exc}"
                    )
                else:
                    send_time_total += time.perf_counter() - send_start
                    send_count += 1

            simulation_app.update()

            tick_count += 1
            tick_time_total += time.perf_counter() - tick_start
            now = time.perf_counter()
            if now - last_stats_print >= STATS_PRINT_INTERVAL_S:
                avg_tick_ms = (tick_time_total / tick_count) * 1000.0
                if send_count:
                    avg_send_ms = (send_time_total / send_count) * 1000.0
                    print(
                        f"[INFO]: avg tick={avg_tick_ms:.2f} ms over {tick_count} ticks "
                        f"(avg follower send_action={avg_send_ms:.2f} ms over {send_count} sends)"
                    )
                else:
                    print(f"[INFO]: avg tick={avg_tick_ms:.2f} ms over {tick_count} ticks")
                tick_count = 0
                tick_time_total = 0.0
                send_time_total = 0.0
                send_count = 0
                last_stats_print = now
    finally:
        keyboard_control.cleanup()
        if connected:
            try:
                robot_iface.robot.disconnect()
            except Exception as exc:
                print(f"[WARN]: Failed to cleanly disconnect the leader arm: {exc}")
        if follower_connected:
            try:
                follower_iface.robot.disconnect()
            except Exception as exc:
                print(f"[WARN]: Failed to cleanly disconnect the follower arm: {exc}")
        if cube_camera_capture is not None:
            cube_camera_capture.release()
        timeline.stop()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
