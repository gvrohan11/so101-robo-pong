# SO-101 underhand toss (copy of xArm "far")

```
python main.py               # dry run: print + limit/rate-check the plan, write trajectory.csv
python main.py --go --slow   # LIVE, swing at 25% speed — do this first
python main.py --go          # LIVE, full speed
```
`main.py` holds the constants and hardware I/O; `toss_motion.py` holds planning,
checks and streaming. Sequence: current pose → START (≤20°/s) → wind-up (2 s) →
swing + hard brake → 0.6 s settle → START (2.5 s). Asks for Enter before moving
(`--yes` skips). Port auto-detects if one `/dev/tty.usbmodem*` exists, else `--port`.

## Start / end pose
Upside-down "J": pan centred, upper arm ~vertical tilted back, forearm forward and
slightly up, wrist bent sharply down, open gripper hanging. Values were read off the
arm with the `my_awesome_follower_arm` calibration (LeRobot degrees, 0 = calibration
midpoint). wrist_flex START = 92.6°, only 4.5° under its 97.1° limit.

## Joint mapping
| xArm FAR | SO-101 | FAR Δ | SO-101 Δ (×0.8) |
|---|---|---|---|
| J1 shoulder | shoulder_lift | −49.9° | −39.9° |
| J2 elbow | elbow_flex | −40.0° | −32.0° |
| J4 wrist pitch | wrist_flex | +35.4° | +28.3° |

Pan and roll are held fixed. On the SO-101: shoulder_lift + = forward, elbow_flex − = forearm up,
wrist_flex + = gripper up/forward (manual encoder checks). The hand-measured clear
wind-up (shoulder +23°, elbow +45° from START) sets the swing direction to shoulder −, elbow −,
wrist +, which matches FAR's signs.

## Scaling
- **Amplitude:** FAR's joint ratios × 0.8. 75% of the shoulder/elbow swing is
  wind-up behind START, 25% is overshoot. The wrist is 95% wind-up because it has no headroom above START.
- **Shape:** same as FAR. All three joints share one time base. The lead joint (shoulder) ramps at
  1000°/s², cruises at 150°/s, then brakes at 3000°/s². Swing ≈ 0.37 s. These are
  STS3215 choices. FAR used ~165°/s and 1145°/s² on the xArm. The ball leaves at the brake.
- **--slow** stretches the swing 4× (same shape, 25% speed).
- No servo speed, accel or PID registers are written, so there is nothing to restore. On exit or error the
  arm is held at its present position with torque on. Support it before cutting power.

## Not yet verified
- The wind-up's wrist (−27°, gripper swung back and down) was not hand-checked for clearance. Run `--go --slow` first.
- How hard the brake really is depends on the servo's PID and onboard acceleration setting. Check the
  execution log `trajectory.execution.jsonl`.

## Gripper / release
Closed (`GRIPPER_CLOSED`) at START and through the wind-up, so it will hold the ball.
Opens to `GRIPPER_OPEN` during the release and closes again on the return. The open command
uses the same rule as Ball-Toss-In-Cup `throw.release_delay()`: it is sent at
`max(brake_start - RELEASE_LEAD_S, RELEASE_MIN_DELAY_S)` after the swing starts, so the fingers are open as the arm
brakes. On the xArm, `lead` was 0.43 s (~0.30 s gripper latency + 0.13 s margin). The SO-101 values
(latency 0.04 s, margin 0.06 s, min delay 0.05 s) are estimates to tune on the arm.

The swing amounts and speed will later be set from the cup's distance from the base.
