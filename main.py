"""SO-101 underhand toss (copy of the xArm "far" open-hand toss).

    python main.py               # DRY RUN: print + limit-check the plan, no robot contact
    python main.py --slow        # dry run of the 25%-speed swing
    python main.py --go --slow   # LIVE, swing at 25% speed (do this first)
    python main.py --go          # LIVE, full speed

Sequence: current pose -> START (slow) -> wind-up (slow) -> swing + hard brake
-> settle -> START (slow). Motion logic lives in toss_motion.py.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
from pathlib import Path

from toss_motion import (ARM_JOINTS, JOINTS, SWING_JOINTS, check_limits, check_rates,
                         execute, limits_from_calibration, plan_toss)

HERE = Path(__file__).resolve().parent

# START / END pose (same pose). Upside-down "J" / hook, seen from the side:
# pan centred toward the target; upper arm ~vertical, tilted slightly back; elbow
# bent ~90 deg so the forearm points forward and slightly up (wrist is the highest
# point); wrist bent sharply down so the open gripper hangs ~20-30 deg from
# vertical in front of and below the elbow; roll neutral.
# Read off the real arm (my_awesome_follower_arm calibration, LeRobot degrees,
# 0 = calibration midpoint; gripper in percent) on Nov 5 2026.
START_POSE = {
    "shoulder_pan": -0.44,
    "shoulder_lift": -14.07,
    "elbow_flex": -8.66,
    "wrist_flex": 92.57,   # NOTE: only ~4.5 deg below the calibrated limit (97.1)
    "wrist_roll": -6.81,
    "gripper": 1.0,        # CLOSED at start/end (overridden by GRIPPER_CLOSED)
}

# Gripper (LeRobot percent, 0 = fully closed). Closed at START and through the
# wind-up (it will hold the ball), opened during the release, closed on return.
GRIPPER_CLOSED = 1.0
GRIPPER_OPEN = 40.0
# During the release the open command is overdriven to this (fully open): the
# position servo moves faster the larger its error, so the jaws open faster.
# After the swing it settles back to GRIPPER_OPEN.
GRIPPER_OPEN_CMD = 100.0
# Release timing, same rule as Ball-Toss-In-Cup throw.release_delay():
# open command at max(brake_start - RELEASE_LEAD_S, RELEASE_MIN_DELAY_S), measured
# from swing start. xArm FAR used lead 0.43 s = ~0.30 s gripper latency + open
# ~0.13 s before braking, and min_delay 0.10 s. The SO-101 gripper servo reacts in
# ~1-2 control steps, so: latency ~0.04 s + the same 0.13 s early-open margin,
# scaled to the SO-101's shorter swing. ESTIMATES: measure and tune on the arm.
GRIPPER_LATENCY_S = 0.04
OPEN_BEFORE_BRAKE_S = 0.10
RELEASE_LEAD_S = GRIPPER_LATENCY_S + OPEN_BEFORE_BRAKE_S
RELEASE_MIN_DELAY_S = 0.05

# xArm FAR swing (throw.py FAR["swing"]), xArm joint index -> degrees.
XARM_FAR_SWING = {1: -49.9, 2: -40.0, 4: +35.4}

# xArm joint -> (SO-101 joint, sign). Signs come from manual encoder checks on
# the SO-101 (Nov 5): shoulder_lift + = upper arm forward, elbow_flex - = forearm
# up, wrist_flex + = gripper up/forward. On the xArm, J1/J2 negative = forward/up
# and J4 positive = flick up. The measured wind-up (shoulder +23, elbow +45 from
# START, "gripper behind and below") fixes the swing direction as shoulder -,
# elbow -, wrist +.
JOINT_MAP = {1: ("shoulder_lift", +1), 2: ("elbow_flex", +1), 4: ("wrist_flex", +1)}

# Amplitude: FAR's joint ratios, scaled to 80% (the SO-101 links are much shorter
# and this keeps the wind-up close to the hand-measured clear wind-up pose).
SWING_SCALE = 0.8

# Fraction of each joint's swing that happens BEFORE START (i.e. during wind-up).
# The rest is overshoot past START before braking. Wrist is 0.95 because START is
# at the top of its range; shoulder/elbow overshoot by 25%.
WINDUP_FRACTION = {"shoulder_lift": 0.75, "elbow_flex": 0.75, "wrist_flex": 0.95}

# Extra wrist tuck in the wind-up only (deg, gripper curls further back under the
# arm). Set from the Nov 7 photo: wind-up gripper hung straight down, needed more tuck.
# The swing then flicks the wrist through this extra angle too.
WRIST_TUCK_DEG = 20.0
# Extra elbow fold in the wind-up only (deg). + folds the forearm in toward the
# upper arm (+ = forearm down). Wrist is near its limit, so the elbow adds the tuck.
ELBOW_TUCK_DEG = 20.0
# Shoulder moves only this much (deg, + = upper arm forward) from START in the
# wind-up, instead of its FAR share (+29.9). Follow-through is unchanged, so the
# shoulder's release swing shrinks to (SHOULDER_WINDUP_DEG + 10) deg.
SHOULDER_WINDUP_DEG = 20.0

# Swing speed profile of the lead joint (largest travel = shoulder_lift). Others
# are time-synchronized so they keep FAR's ratios. FAR ran ~165 deg/s with 1145
# deg/s^2 on the xArm; these are SO-101 (STS3215) choices, not copies:
SWING_PEAK_VEL = 150.0     # deg/s, cruise speed of the lead joint
SWING_ACCEL = 1000.0       # deg/s^2, ramp-up
SWING_BRAKE = 3000.0       # deg/s^2, the hard stop that releases the ball
RELEASE_SPEEDUP = 1.5      # full-speed (--go without --slow) swing runs 1.5x faster, same shape
SLOW_FACTOR = 4.0          # --slow stretches the swing 4x (25% speed, same shape)

SLOW_VEL = 20.0            # deg/s cap for approach / return moves
WINDUP_SPEEDUP = 1.1       # wind-up runs 1.1x faster than its 2.0 s base
WINDUP_ACCEL_FRAC = 0.4    # wind-up speeds up over its first 40%...
WINDUP_DECEL_FRAC = 0.08   # ...and stops over its last 15% (short = no visible pause)
WINDUP_TIME_S = 2.0 / WINDUP_SPEEDUP        # START -> wind-up
SETTLE_TIME_S = 0.6        # hold after braking (ball leaves, servos settle)
RETURN_TIME_S = 2.5        # follow-through -> START
RATE_HZ = 50               # target streaming rate
LIMIT_MARGIN_DEG = 2.0     # stay this far inside calibrated encoder range
MAX_VEL_CAP = 250.0        # sanity caps on the whole plan
MAX_ACC_CAP = 7000.0
SLOW_TOLERANCE_DEG = 8.0   # abort slow phases if the arm lags this much
MAX_LATENESS_S = 0.05


def toss_poses(start: dict) -> tuple[dict, dict]:
    """Wind-up and follow-through poses from START + the mapped, scaled FAR swing."""
    start = dict(start, gripper=GRIPPER_CLOSED)
    windup, follow = dict(start), dict(start, gripper=GRIPPER_OPEN)
    for xj, deg in XARM_FAR_SWING.items():
        j, sign = JOINT_MAP[xj]
        d = sign * deg * SWING_SCALE
        windup[j] = start[j] - WINDUP_FRACTION[j] * d
        follow[j] = start[j] + (1 - WINDUP_FRACTION[j]) * d
    windup["shoulder_lift"] = start["shoulder_lift"] + SHOULDER_WINDUP_DEG
    windup["elbow_flex"] += ELBOW_TUCK_DEG
    windup["wrist_flex"] += WRIST_TUCK_DEG     # + tucks the gripper back (Nov 7 photo check)
    return windup, follow


def find_port() -> str | None:
    ports = sorted(glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/ttyACM*"))
    return ports[0] if len(ports) == 1 else None


def build_plan(current: dict, start: dict, limits: dict, slow: bool):
    start = dict(start, gripper=GRIPPER_CLOSED)
    windup, follow = toss_poses(start)
    samples = plan_toss(current, start, windup, follow, rate_hz=RATE_HZ,
                        gripper_closed=GRIPPER_CLOSED, gripper_open=GRIPPER_OPEN,
                        gripper_open_cmd=GRIPPER_OPEN_CMD,
                        release_lead=RELEASE_LEAD_S, release_min_delay=RELEASE_MIN_DELAY_S,
                        peak_vel=SWING_PEAK_VEL, accel=SWING_ACCEL, brake=SWING_BRAKE,
                        slow_vel=SLOW_VEL, windup_time=WINDUP_TIME_S,
                        windup_accel_frac=WINDUP_ACCEL_FRAC, windup_decel_frac=WINDUP_DECEL_FRAC,
                        settle_time=SETTLE_TIME_S, return_time=RETURN_TIME_S,
                        time_scale=SLOW_FACTOR if slow else 1.0 / RELEASE_SPEEDUP)
    check_limits(samples, limits)
    peaks = check_rates(samples, MAX_VEL_CAP, MAX_ACC_CAP)
    return samples, windup, follow, peaks


def report(samples, windup, follow, peaks, limits, csv_path: Path):
    fmt = lambda p: "  ".join(f"{j}={p[j]:7.2f}" for j in JOINTS)
    print("START    ", fmt(START_POSE))
    print("WIND-UP  ", fmt(windup))
    print("FOLLOW   ", fmt(follow))
    for j in SWING_JOINTS:
        print(f"  {j:13s} swing {follow[j]-windup[j]:+6.1f} deg   limits "
              f"[{limits[j][0]:.1f}, {limits[j][1]:.1f}]   peak {peaks[j][0]:5.0f} deg/s "
              f"{peaks[j][1]:5.0f} deg/s^2")
    swing = [s for s in samples if s.phase == "swing"]
    t_open = next(s.t for s in swing if s.q["gripper"] == GRIPPER_OPEN_CMD) - swing[0].t + 1/RATE_HZ
    print(f"Gripper: closed {GRIPPER_CLOSED}% -> open {GRIPPER_OPEN_CMD}% commanded "
          f"{t_open:.2f}s into the swing")
    print(f"Swing {swing[-1].t - swing[0].t + 1/RATE_HZ:.2f}s, total {samples[-1].t:.2f}s, "
          f"{len(samples)} samples -> {csv_path.name}")
    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "phase", *JOINTS])
        w.writerows([round(s.t, 4), s.phase, *(round(s.q[j], 3) for j in JOINTS)] for s in samples)
    print("Limit and rate checks: PASS")


def main() -> None:
    ap = argparse.ArgumentParser(description="SO-101 underhand toss (dry run unless --go)")
    ap.add_argument("--go", action="store_true", help="actually move the arm")
    ap.add_argument("--slow", action="store_true", help="swing at 25%% speed (first test)")
    ap.add_argument("--yes", action="store_true", help="skip the Enter confirmation")
    ap.add_argument("--port", help="follower serial port (auto-detected if only one)")
    ap.add_argument("--robot-id", default="my_awesome_follower_arm")
    ap.add_argument("--calibration-dir", type=Path, default=HERE / "calibration")
    ap.add_argument("--output", type=Path, default=HERE / "trajectory.csv")
    args = ap.parse_args()

    cal_file = args.calibration_dir / f"{args.robot_id}.json"
    limits = limits_from_calibration(json.loads(cal_file.read_text()), LIMIT_MARGIN_DEG)

    if not args.go:
        samples, windup, follow, peaks = build_plan(START_POSE, START_POSE, limits, args.slow)
        report(samples, windup, follow, peaks, limits, args.output)
        print("DRY RUN: arm not contacted. Add --go (and --slow first) to move.")
        return

    port = args.port or find_port()
    if not port:
        ap.error("could not auto-detect a single serial port; pass --port /dev/tty.usbmodemXXXX")

    try:
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    except ImportError:
        from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

    robot = SO101Follower(SO101FollowerConfig(port=port, id=args.robot_id,
                          calibration_dir=args.calibration_dir, use_degrees=True,
                          disable_torque_on_disconnect=False))
    if not robot.calibration:
        ap.error(f"no calibration found at {cal_file}")

    def read() -> dict:
        obs = robot.bus.sync_read("Present_Position")
        return {j: float(obs[j]) for j in JOINTS}

    def send(q: dict) -> None:
        robot.bus.sync_write("Goal_Position", {j: q[j] for j in JOINTS})

    log_path = args.output.with_suffix(".execution.jsonl")
    try:
        # bus.connect() only (not robot.connect()): configure() would briefly drop
        # torque and rewrite PID registers. We change NO speed/accel/PID registers,
        # so there is nothing to restore afterwards.
        robot.bus.connect()
        if not robot.is_calibrated:
            raise RuntimeError("motor calibration differs from the saved file; refusing")
        current = read()
        start = dict(START_POSE)
        samples, windup, follow, peaks = build_plan(current, start, limits, args.slow)
        report(samples, windup, follow, peaks, limits, args.output)
        print(f"\nLIVE on {port}. Clear the workspace; be ready to cut power.")
        if not args.yes:
            input("Press Enter to move (Ctrl-C to cancel)... ")
        # Hold in place: set every goal to its present position (raw ticks), then
        # enable torque. Safe whether or not a motor is already powered: a powered
        # motor whose goal differs (e.g. a stalled gripper) just stops pushing.
        raw = robot.bus.sync_read("Present_Position", normalize=False)
        robot.bus.sync_write("Goal_Position", raw, normalize=False)
        robot.bus.enable_torque(JOINTS)
        with log_path.open("w") as log:
            execute(samples, read, send, SLOW_TOLERANCE_DEG, MAX_LATENESS_S,
                    lambda e: log.write(json.dumps(e) + "\n"))
        end = read()
        print("Done. Final error vs START:",
              {j: round(end[j] - start[j], 1) for j in ARM_JOINTS}, f"log: {log_path.name}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if robot.bus.is_connected:
            try:   # hold safely where it is: target = present position, torque stays on
                here = read()
                send(here)
            except Exception as e:
                print("hold command failed:", e)
            robot.bus.disconnect(disable_torque=False)
        print("Disconnected; torque left ON so the arm holds. Support it before powering off.")


if __name__ == "__main__":
    main()
