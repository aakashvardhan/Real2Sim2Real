# How to Run: AWSBuilderCube Teleop (Isaac Sim 6.0.1, no Isaac Lab)

Quick step-by-step for running the two raw-Isaac-Sim teleop scripts against
`source/sim_to_real_so101/demo/real-to-sim.usd`. For the full technical
background (why these exist, physics fixes, verification history), see
[aws-cube-to-bowl-teleop-plan.md](aws-cube-to-bowl-teleop-plan.md).

Both scripts run on a plain Isaac Sim 6.0.1 Python (`C:\Isaac-Sim\python.bat`)
with **no Isaac Lab installed** — always launch by direct file path from the
repo root, never `-m`.

---

## Option A — Keyboard jog (no hardware needed)

```powershell
C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\keyboard_agent_raw_isaacsim.py
```

Run from the repo root. Boots in ~30-45s. Click into the viewport once it
opens — it needs focus to receive key events. If the robot isn't visible,
select `SO_ARM101_USD` in the Stage outliner and press `F` to frame it.

**Controls** (hold to move, release to stop):

| Joint | Increase (+) | Decrease (−) |
|---|---|---|
| Rotation (base yaw) | `D` | `A` |
| Pitch (shoulder) | `W` | `X` |
| Elbow | `E` | `V` |
| Wrist_Pitch | `T` | `G` |
| Wrist_Roll | `Y` | `H` |
| Jaw (gripper) | `U` | `J` |

`R` resets all joint targets to 0° **and** snaps the cube/bowl back to their
starting positions.

**Grasp walkthrough**: press `U` to open the jaw (it starts nearly closed),
jog `D/A W/X E/V T/G` down to the cube (world pos roughly `(0, 0.03, 0.775)`,
robot base at `(0, 0.3, 0.72)`), press `J` to close, then `W`/`T` to lift.

Add `--headless` for no viewport, `--joint_step <degrees>` to change jog
speed (default `0.5°`/tick).

---

## Option B — Real leader arm

### One-time setup (already done on this machine, keep for reference)

The vendored `lerobot` fork needs installing into Isaac Sim's own Python,
pinned to match Isaac Sim's bundled torch/torchvision exactly (mismatched
versions crash Kit's native extensions):

```powershell
cd lerobot-sim
C:\Isaac-Sim\python.bat -m pip install -e ".[so101]"
C:\Isaac-Sim\python.bat -m pip install "torch==2.11.0+cu128" "torchvision==0.26.0+cu128" --index-url https://download.pytorch.org/whl/cu128
```

Leader-arm calibration lives in this repo at
`calibration/teleoperators/so_leader/my_so_arm.json` — the script points
`HF_LEROBOT_CALIBRATION` there itself, no manual env var needed.

### Run

```powershell
C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py
```

Run from the repo root. Same boot time as Option A. The sim gripper tracks
the physical leader arm live once you see:

```
[INFO]: Leader arm connected: port=COM4 id=my_so_arm
[INFO]: Driving joints from the leader arm. Ctrl+C or close the window to stop.
```

Defaults to port `COM4`, id `my_so_arm` — override with `--port` / `--robot_id`
if your setup differs.

`R` (keyboard, needs viewport focus) resets the cube/bowl to their starting
positions — joint targets aren't reset here since they're driven live by the
physical arm every tick, not held state.

**Grasp walkthrough**: open the gripper, position it over the cube in the
viewport (same world position as above), close, lift, then try some
rotation/lateral motion while holding — that's the real test for slipping,
not just a straight vertical lift.

---

## Troubleshooting

- **Nothing happens on key press**: click into the viewport window first.
- **Boot looks stuck / high GPU use, no window**: if launched from an
  automated/agent terminal rather than an interactive one, the window may
  never actually attach to the desktop. Run from a real interactive terminal.
- **Leader arm won't connect**: check the physical arm is on the port you
  passed (`--port`, default `COM4`) and that `calibration/teleoperators/so_leader/`
  has a file matching `--robot_id` (default `my_so_arm`).
- **Cube launches away instead of being held**: already fixed (Jaw
  `effort_limit` was 30, way more torque than a 50g cube needs against an
  unreachable "fully closed" target — reduced to 3). If it's still happening,
  say so — there's more room to tune.
