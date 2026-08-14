# Real-to-Sim Teleop Plan: AWSBuilderCube → PaperBowl

**Goal:** teleoperate the real SO-101 leader arm (COM4) to drive `SO_ARM101_USD` inside
[`source/sim_to_real_so101/demo/real-to-sim.usd`](../source/sim_to_real_so101/demo/real-to-sim.usd)
for a pick-and-place task — place `AWSBuilderCube` into `PaperBowl`.

**Target platform constraint: Isaac Sim 6.0.1.** The machine this plan was researched on has
Isaac Sim **5.1.0.0** installed (`Y:\e` venv, `pip show isaacsim` → `5.1.0.0`), not 6.0.1. See
[Isaac Sim 6.0.1 constraint](#isaac-sim-601-constraint) before implementing — it changes the
risk profile for the whole repo, not just this task.

Status: **planning only, not yet implemented.**

---

## 1. What already exists — don't rebuild it

- [`scripts/lerobot_agent.py`](../source/sim_to_real_so101/scripts/lerobot_agent.py) +
  [`utils/lerobot_interface.py`](../source/sim_to_real_so101/utils/lerobot_interface.py)'s
  `LeRobotSO101Interface(kind="leader", port=..., id=...)` already implement "real leader arm
  drives an Isaac Lab task" end to end — connect, read `robot.get_action()`, map to sim
  joint-radian space, `env.step()`, optional `--repo_id/--repo_root/--task_name` dataset
  recording. **No new teleop script needed.** The only missing piece is a gym-registered task
  config for this scene.
- The robot referenced by `real-to-sim.usd` (`/World/SO_ARM101_USD` →
  `../assets/usd/SO-ARM101-USD.usd`) is the **exact same file** already wired up as
  `SO101_CFG` in [`assets/so101.py`](../source/sim_to_real_so101/assets/so101.py), with joint
  names (`Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw`) matching `ActionsCfg` in
  [`tasks/so101_env_cfg.py`](../source/sim_to_real_so101/tasks/so101_env_cfg.py) exactly. No
  new robot articulation config needed.

## 2. Facts verified by inspecting the stage directly

Raw pxr isn't importable via plain `python.bat`; verified instead with a headless
`isaaclab.app.AppLauncher` script run through `Y:\e\Scripts\python.exe` (the venv this repo's
own scripts document running with), traversing the stage with `Usd.Stage.Open()`. See
[Isaac Sim inspection technique](#isaac-sim-inspection-technique-gotcha) for the buffering
gotcha hit while doing this.

| Prim | State | Implication |
|---|---|---|
| `/World/SO_ARM101_USD` | world pos `(0, 0.3, 0.72)`, references `SO-ARM101-USD.usd` | Reuse `SO101_CFG`, override `init_state.pos/rot` only |
| `/World/AWSBuilderCube` | mesh `AWSBuilderCube_Geo`: extent ±0.025 (5cm cube, 8 pts), `physics:approximation = convexHull` **already set**, only `PhysicsCollisionAPI`/`PhysicsMeshCollisionAPI` — **no `RigidBodyAPI`/`MassAPI`** | Frozen as authored. Needs a rigid body + mass added; collision shape is already correct, no need to touch it |
| `/World/AWSCubePaper` | sibling of `AWSBuilderCube` (not a child), world pos `(0, 0.03, 0.7504)` — coincident with the cube's bottom face | Paper label decal. Skip for v1 (see §3) |
| `/World/PaperBowl` | mesh `Bowl_Geo`: extent `x:[-0.05,0.05] y:[-0.0375,0.0375] z:[0,0.032]` (**not circular** — 10cm×7.5cm, 3.2cm tall), `physics:approximation = none` (exact triangle mesh), no rigid body, world pos `(0.2, 0.03, 0.75)` | Leave static — matches the real fixed bowl position. Placement check must use a local-frame XY **box** bound sized to this extent, not a circular radius |
| Cameras | only `/World/SO_ARM101_USD/gripper/WristCamera` | No external/D455 camera in this scene, unlike the indoor-room task. Only matters once dataset recording is added (§4 stage 3) |
| `demo/room-and-table-with-aws-cube.usd` | checked directly — it's a **strict subset** of `real-to-sim.usd` (room+tables+mount+cube, no robot/bowl/paper) | Not useful as a separate base; drop it from consideration entirely |

## 3. Asset strategy — thin reference wrappers, not full extraction

USD references can target a *specific prim path* inside another layer
(`references = @real-to-sim.usd@</World/AWSBuilderCube>`), not just a whole file's
defaultPrim. This avoids hand-flattening/duplicating any geometry or materials — two small
new files under `assets/usd/` are enough:

1. **`aws-cube-bowl-room.usda`** — references `real-to-sim.usd`'s `/World`, with `over`
   blocks setting:
   - `/World/SO_ARM101_USD.active = false` (we spawn our own `SO101_CFG` copy separately —
     leaving this active would double-spawn a robot on top of it)
   - `/World/AWSBuilderCube.active = false` (we spawn a separate dynamic rigid-body copy —
     same double-spawn concern)
   - `PaperBowl` and everything else (walls, tables, mount, lights) stay active, untouched.
     **No separate bowl asset needed** — read its pose at runtime via
     `sim_utils.find_matching_prims()` off the room's resolved prim path, exactly like
     `randomize_mat_rotation`/`randomize_camera_pose` in
     [`mdp/resets.py`](../source/sim_to_real_so101/mdp/resets.py) already do for other static
     nested prims.
2. **`AWSBuilderCube.usda`** — references just `real-to-sim.usd</World/AWSBuilderCube>` as
   its defaultPrim, nothing else authored in the file. `RigidObjectCfg.spawn.rigid_props` /
   `.mass_props` in Python add `RigidBodyAPI`/`MassAPI` at spawn time — the same mechanism
   already used for `Vial_opaque.usda` in
   [`tasks/vials_to_rack_env_cfg.py`](../source/sim_to_real_so101/tasks/vials_to_rack_env_cfg.py).
   No physics schemas need to be hand-authored inside the wrapper.
3. **Skip `AWSCubePaper`** (the label decal) for v1. Its position is coincident with the
   cube's bottom face; reparenting it correctly means solving a local-offset problem for a
   purely cosmetic sticker — not worth it before the functional task works. A plain textured
   cube with no sticker is not a blocker.

## 4. Task config — staged, mirroring `vials_to_rack_env_cfg.py`

New module `tasks/aws_cube_to_bowl_env_cfg.py`, scene built off `LerobotSo101BaseSceneCfg`
(**not** `SO101TaskSceneCfg` — that one assumes the indoor-room/mat/lightstudio layout, a
different physical setup than this two-table/RobotMount demo).

Build and test in stages — each stage runnable with `keyboard_agent.py` (no hardware) before
moving to the next:

**Stage 1 — bare teleop.** Validates the real unknown (does the cube's rigid-body physics
behave sensibly under gripper contact) before adding any bookkeeping on top.
- `robot`: `S0101_CONTACT_GRASP_CFG.replace(prim_path=..., init_state=...)` positioned at
  `(0, 0.3, 0.72)` — **use `S0101_CONTACT_GRASP_CFG`, not plain `SO101_CFG`**, even at this
  stage, so the contact sensor added in stage 2 doesn't silently read zero force because
  `activate_contact_sensors` was never set (easy to forget, matches what
  `vials_to_rack_env_cfg.py`'s scene already does).
- `room`: the `aws-cube-bowl-room.usda` wrapper.
- `aws_cube`: `RigidObjectCfg` on the `AWSBuilderCube.usda` wrapper.
- No sensors, no cameras, no observations/terminations beyond the base `ObservationsCfg`.
- Register `Lerobot-So101-Teleop-Aws-Cube-To-Bowl` in `tasks/__init__.py`, same pattern as
  the existing `Lerobot-So101-Teleop-Vials-To-Rack` registration.

**Stage 2 — dataset-recording support** (only needed once `--repo_id` recording is wanted):
- `contact_grasp`: `ContactSensorCfg` on `.../jaw`, `filter_prim_paths_expr=["...AwsCube"]`.
- New `mdp/terms.py` functions `cube_grasped` / `cube_placed_in_bowl`, near-direct copies of
  `any_vial_grasped` / `vial_placed_on_rack`, but with the rack's rectangular XY-bounds check
  replaced by a box check sized to the bowl's real extent (`x:[-0.05,0.05] y:[-0.0375,0.0375]`
  from §2), not a circular radius.
- New `mdp/resets.py` function `reset_aws_cube`, a simplified single-object version of
  `reset_vials_rack` (reuse the existing `random_asset_pose` helper already in that file).
- `camera_ego`: point at `.../gripper/WristCamera` (verify this exact prim name at
  implementation time — see the note in §2's camera row).

**Stage 3 — eval variant** (only if/when automated eval is wanted, not required for
teleoperation itself): `Lerobot-So101-Teleop-Aws-Cube-To-Bowl-Eval` with a
`cube_placed_in_bowl_termination` `DoneTerm`, copying `vial_placed_on_rack_termination`'s
confirm-steps structure.

**Teleop command (unchanged at every stage):**
```
python -m sim_to_real_so101.scripts.lerobot_agent \
    --task Lerobot-So101-Teleop-Aws-Cube-To-Bowl \
    --port COM4 --robot_id armatron_leader
```

## 5. Isaac Sim 6.0.1 constraint

Update: **Isaac Sim 6.0.1 is in fact installed locally** — `C:\Isaac-Sim` (`VERSION` file:
`6.0.1-rc.7+release.42383.32955d8d.gl`), a full GUI-capable install accessible under this
session's user account. There's also a second environment, `C:\ilab\e6`
(`isaaclab==3.0.0b2.post1` + `isaacsim==6.0.1.0`, per `pip install` log
`C:\ilab\install_isaaclab3.log`), with real validation logs already produced on 2026-08-13
(`smoke_isaaclab3.log`, `list_envs_e6.log`, `zero_agent_e6.log`/`.err.log`) — but that venv
was created under a **different Windows user profile** (`GUA-ADMIN`, per its
`pyvenv.cfg` and the paths in its logs, e.g.
`C:\Users\GUA-ADMIN\Desktop\Aakash\Sim-to-Real-SO-101-Workshop\...`), so this session
(`OMNI-User`) gets `permission denied (os error 5)` trying to execute its Python directly —
its logs are read-only evidence, not something this session can currently re-run. Worth
confirming with the user whether that's a teammate's/setup script's environment they intend
to keep using, or whether a fresh `OMNI-User`-owned Isaac Lab 3.0 venv should be created.

**Isaac Sim 6.0.1 pairs with Isaac Lab 3.0 (currently Beta 2 - Patch 1)**, described by Isaac
Lab's own release notes as introducing "a ground-up architectural overhaul" — a factory-based
multi-backend physics architecture (PhysX / Newton / OVPhysX). There is no Isaac Lab 2.x
release targeting Isaac Sim 6.0.1. This is a whole-repo concern, not something scoped to just
this new task — confirmed concretely below.

### What the existing `e6` validation logs already show

- `smoke_isaaclab3.log`: basic Isaac Lab 3.0 functionality works — "stepped 10 sim steps OK,
  device = cuda:0", "197 Isaac tasks registered in gym", "SMOKE: ALL OK".
- `list_envs_e6.log`: running **this repo's own** `sim_to_real_so101.scripts.list_envs`
  against Isaac Lab 3.0 / Isaac Sim 6.0.1 successfully lists all 6 currently-registered gym
  tasks (`Lerobot-So101-Teleop-Base`, `-Task`, `-Vials-To-Rack`, `-Vials-To-Rack-DR`,
  `-Vials-To-Rack-Eval`, `-Vials-To-Rack-DR-Eval`) — package import and gym registration
  succeed end to end.
- `zero_agent_e6.err.log`: **but actually running one of these tasks crashes**:
  ```
  File ".../source/sim_to_real_so101/tasks/task_env_cfg.py", line 26, in <module>
      from isaacsim.core.utils.rotations import euler_angles_to_quat
  ModuleNotFoundError: No module named 'isaacsim.core.utils'
  ```

### Confirmed root cause and fix

Checked this exact import against the locally-installed `C:\Isaac-Sim` 6.0.1 (via
`isaacsim.SimulationApp`, same technique as §6): **`isaacsim.core.utils.rotations` imports
fine there.** So this isn't "Isaac Sim 6.0.1 removed the old Core API" outright — it's
launch-path-dependent (Isaac Lab 3.0's new "kit-less execution mode" is the likely
differentiator: `list_envs.py` and the smoke test evidently loaded whatever extension provides
`isaacsim.core.utils`, `zero_agent.py`'s launch path didn't). That makes it a landmine that
may or may not trigger depending on exactly how the app is launched — not something safe to
leave alone just because it happens to work in one invocation.

Verified the replacement directly (`inspect.signature` + a live call, same technique):
`isaacsim.core.experimental.utils.transform.euler_angles_to_quaternion` is the Core
Experimental API equivalent — confirmed present, and numerically identical for the same input
(`[0,0,90]` degrees → `(0.7071, 0, 0, 0.7071)` from both). Same semantics (`euler_angles`,
`degrees`, `extrinsic` params; returns quaternion as **(w,x,y,z)**, matching the convention
already recorded for `look_at_quaternion` in the same module — see
[[isaac_sim_look_at_quaternion_order]]). Two call-site changes needed when migrating:
- `degrees` becomes **keyword-only** (`def euler_angles_to_quaternion(euler_angles, *, degrees=False, ...)`) — positional calls like `euler_angles_to_quat(arr, True)` would break; every call site in this repo already uses `degrees=True` as a keyword, so this is a non-issue in practice, just worth knowing.
- Return type is **`wp.array`, not `numpy.ndarray`** — call sites that hand the result straight
  to `InitialStateCfg(rot=...)`/`AssetBaseCfg.InitialStateCfg(rot=...)` (a plain-array-like
  field, not a live physics tensor) need `.numpy()` appended, e.g.
  `euler_angles_to_quaternion(np.array([0,0,90]), degrees=True).numpy()`.

**Exact call sites needing this migration** (grepped repo-wide for
`from isaacsim.core` — all four use the old rotations import except the second):
1. [`assets/so101.py:22`](../source/sim_to_real_so101/assets/so101.py) —
   `from isaacsim.core.utils.rotations import euler_angles_to_quat`
2. [`mdp/resets.py:28`](../source/sim_to_real_so101/mdp/resets.py) —
   `from isaacsim.core.prims import XFormPrim` (different symbol/submodule; imported fine in
   the same spot-check on `C:\Isaac-Sim`, but treat that as "not yet proven safe under the
   `e6`/kit-less launch path either," precisely because we just demonstrated that "works in
   one environment" didn't predict the `zero_agent_e6` failure)
3. [`tasks/task_env_cfg.py:26`](../source/sim_to_real_so101/tasks/task_env_cfg.py) —
   `from isaacsim.core.utils.rotations import euler_angles_to_quat`
4. [`tasks/vials_to_rack_env_cfg.py:26`](../source/sim_to_real_so101/tasks/vials_to_rack_env_cfg.py) —
   same import

**This directly blocks the new task as planned, not just the existing ones.** §4's scene
deliberately avoids `SO101TaskSceneCfg`/`task_env_cfg.py` to sidestep the indoor-room/mat
layout mismatch — but it still imports `SO101_CFG`/`S0101_CONTACT_GRASP_CFG` from
`assets/so101.py` (site #1 above), which is unavoidable (every task in this repo needs the
robot config), and stage 2's `reset_aws_cube` would live in `mdp/resets.py` (site #2). **Fix
site #1 (and, defensively, #2) before or alongside writing the new task** — this is a
prerequisite, not an optional smoke-test-later item.

### Other requirements for new code in this plan

1. Every new `mdp/terms.py`/`mdp/resets.py` function (`cube_grasped`, `cube_placed_in_bowl`,
   `cube_placed_in_bowl_termination`, `reset_aws_cube`) **must** wrap every `.data.*` read in
   `as_torch(...)`, matching `any_vial_grasped`/`vial_placed_on_rack` exactly.
   [`mdp/_compat.py`](../source/sim_to_real_so101/mdp/_compat.py)'s docstring confirms exactly
   why: *"Isaac Lab 2.x returns torch.Tensor from `.data.*` properties; Isaac Lab 3.0 returns
   warp.array instead."* This was already a known, partially-solved problem in this codebase
   before this planning session.
2. If/when stage 2 adds a camera, prefer `CameraCfg` over `TiledCameraCfg` — 3.0 folds
   `TiledCamera` into `Camera` ("existing tiled-camera aliases remain as compatibility surface
   where available, but new code should use `Camera` and `CameraCfg`"). The existing
   `camera_object` in `task_env_cfg.py` still uses `TiledCameraCfg` and should keep working
   via the alias; no reason for *new* code to take on that deprecation.
3. Keep `ImplicitActuatorCfg` for the robot's actuators (what `SO101_CFG` already uses) —
   implicit PD-drive actuators are the simpler, lower-risk surface; nothing suggests they're
   going away, unlike the explicit/Newton-unified actuator path.

### Process recommendation

Fix the 4 known import sites (§5 above) first, confirming each against whichever concrete
Isaac Sim 6.0.1 / Isaac Lab 3.0 Python environment the user settles on (either get access
sorted out for the existing `e6` venv, or stand up an `OMNI-User`-owned one) — then re-run
`zero_agent.py`/`list_envs.py` against the **existing**, unmodified
`Lerobot-So101-Teleop-Vials-To-Rack` task to confirm it actually steps physics end to end
(not just imports/registers) before writing any of this plan's new task code. That task
already exercises nearly every mechanism this plan needs: `ArticulationCfg`, `RigidObjectCfg`
with `mass_props`/`rigid_props`, `ContactSensorCfg`, manager-based
observations/events/terminations.

## 6. Isaac Sim inspection technique (gotcha)

For reference when re-verifying any of the above, or checking new USD edits:

- `C:\Isaac-Sim\python.bat` alone can't import `pxr` — it needs `isaacsim.SimulationApp` (or
  `isaaclab.app.AppLauncher`) to boot Kit first, which registers the extensions that make
  `pxr` importable.
- For scripts needing `isaaclab` APIs specifically (not just raw Isaac Sim), use this repo's
  actual Isaac Lab venv, `Y:\e\Scripts\python.exe` (the one `docs/isaac-sim-windows-guide.md`
  documents running the repo's own scripts with), with the same
  `isaaclab.app.AppLauncher` boot pattern `keyboard_agent.py`/`lerobot_agent.py` use.
- **Buffered stdout is silently dropped when `simulation_app.close()` runs** — Kit's shutdown
  appears to force-terminate the process rather than doing a clean CPython exit, so anything
  still sitting in Python's stdout buffer (block-buffered when redirected to a file/pipe)
  never flushes; the log just cuts off mid-line with no error. Symptom looks like a hang or
  silent crash. Fix: write results to your own file opened in the script and call `.flush()`
  after every write, don't rely on `print()` + shell redirection alone.

Full details also saved to memory: `isaac_sim_local_rtx_verification` and
`real_to_sim_aws_cube_bowl_task`.
