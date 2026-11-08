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

## Per-throw variables (`toss_params.py`)
These 14 values are the only things that change from throw to throw. Before each
throw, `params_for_target(distance_m)` returns them; for now it returns the
hand-tuned defaults, and later a table, fit or trained RL/ML policy will. `TossParams`
has `to_vector()` / `from_vector()` (fixed order) for ML, plus JSON I/O
(`python main.py --params p.json`). Every set is checked against joint limits and
speed caps before the arm moves.

| Variable | Default | Unit |
|---|---|---|
| windup_move_shoulder / elbow / wrist | +20.0 / +44.0 / −6.9 | deg |
| windup_speed_shoulder / elbow / wrist | 18.1 / 39.8 / 6.2 | deg/s (peak) |
| release_move_shoulder / elbow / wrist | −30.0 / −52.0 / −15.0 | deg |
| release_speed_shoulder / elbow / wrist | 130 / 225 / 65 | deg/s (peak) |
| gripper_move | 52.1 (= 1% → 40%) | deg of gripper-servo rotation |
| gripper_release_speed | 600 | deg/s (commanded) |

Signs: shoulder_lift + = upper arm forward, elbow_flex + = forearm folds in,
wrist_flex + = gripper tucks back, gripper + = opens. Each joint ramps up and
brakes on its own profile, and all joints start together. To keep the joints
finishing together, keep each speed proportional to its move.

## Fixed setup (`main.py`)
START_POSE, ramp shapes (`WINDUP_RAMP_S`, `RELEASE_RAMP_S`), the release-timing rule
from Ball-Toss-In-Cup (`RELEASE_LEAD_S`, `RELEASE_MIN_DELAY_S`; open command at
max(brake start − lead, min delay)), gripper torque boost, safety caps, control rate.
`--slow` stretches only the release 4×. On exit the gripper torque limit is restored
and the arm holds its position with torque on.

## Origin of the defaults
Started from the xArm FAR swing (J1 −49.9°, J2 −40°, J4 +35.4° → shoulder_lift,
elbow_flex, wrist_flex) scaled ×0.8. They were then hand-tuned on the arm on Nov 7:
more elbow fold, less shoulder in the wind-up, the wrist reversed in the release,
and a faster release and wind-up.
