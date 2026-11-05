# SO-101 Robo Pong

Self-contained underhand-toss motion baseline. No imports, paths, or runtime dependency on `so101-setup`. No Viam or xArm dependency. Detection, pickup, targeting and RL are future work.

## Offline preview (Python 3.10+; no dependencies)

```bash
python main.py
python main.py --release --output release-preview.csv
python -m unittest -v
```

`toss_motion.py` models ready -> wind-up -> forward swing -> ready using minimum-jerk quintic interpolation. Shoulder lift, elbow flex and wrist flex reverse between wind-up and swing; base pan and wrist roll remain fixed. Each joint's displacement and the shared swing duration determine its velocity. Peak speed and acceleration are checked analytically. `--speed` shortens ONLY the swing duration, increasing speed linearly and acceleration quadratically. This is a position trajectory model, not a verified human-motion retargeting or ballistic model.

`toss.example.json` contains illustrative degree angles, not measured SO-101 throwing poses. Its zero pose is NOT a known safe home. The example cannot execute live. Never copy xArm angles or speed tables into this arm.

## Prepare a hardware profile

1. Calibrate using your normal LeRobot tools. Copy the follower calibration JSON into this project's own `calibration/` directory, keeping its robot-ID filename. No calibration files are imported from the setup project at runtime.
2. Create `toss.local.json` from the example. Measure ready, wind-up and follow-through poses on your own arm and specify conservative joint limits and speed/acceleration caps. All angles use LeRobot degree normalization; gripper is percent, not degrees. Determine positive opening direction before holding a ball.
3. Review the complete swept arm/gripper/cable path and tabletop clearance, including return. This first version does not contain an SO-101 URDF or automatic collision checker. Set `hardware_validated` and `swept_path_reviewed` only after checking these on your hardware.
4. Install LeRobot with Feetech support as an independent dependency in this project's environment (not an editable install pointing at so101-setup). The adapter targets `lerobot.robots.so_follower.SO101Follower`, matching the inspected setup's API. Version differences need checking.
5. Place the arm at the measured ready pose. Initial errors and excessive tracking errors abort execution. Use an empty gripper for the first slow test and have someone at the power disconnect.

```bash
python main.py --config toss.local.json --port YOUR_FOLLOWER_PORT --robot-id my_awesome_follower_arm --calibration-dir calibration --go
```

Add `--release` only after empty-arm motion is verified. Measure physical opening latency; the default 0.15 seconds is a placeholder, not a measurement. The release command is advanced by `latency_s` relative to `swing_fraction`; opening moves gradually at `opening_rate_pct_s`. Actual departure depends on finger geometry and contact, so validate with video. The scheduler includes USB/read/write latency, aborts missed deadlines, and logs measured positions and command times to JSONL.

Hardware connect configures LeRobot's motor mode/gains. On failure or Ctrl+C no automatic return is attempted: with no collision model, that could compound a problem. Disconnect retains servo torque; support the arm before removing power. Ball recovery is manual for this initial motion-only project. No motor speed/acceleration registers are raised by this code.

The program assumes the ball is already held. It does not claim to predict landing distance or guarantee a toss. Build a new SO-101 dataset after measuring trajectory tracking, release delay, and landing outcomes.
