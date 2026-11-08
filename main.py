"""SO-101 underhand toss (copy of the xArm "far" open-hand toss).

    python main.py               # DRY RUN: print + limit-check the plan, no robot contact
    python main.py --slow        # dry run of the 25%-speed release
    python main.py --go --slow   # LIVE, release at 25% speed (do this first)
    python main.py --go          # LIVE, full speed
    python main.py --params p.json   # use per-throw variables from a JSON file

The 14 per-throw variables (movements and speeds) live in toss_params.py and are
chosen per throw by params_for_target(). Everything below is fixed setup.
Sequence: current pose -> START (slow) -> wind-up -> release + hard brake ->
settle -> START (slow). Motion planning lives in toss_motion.py.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
from pathlib import Path

from toss_motion import (ARM_JOINTS, JOINTS, SWING_JOINTS, check_limits, check_rates,
                         execute, limits_from_calibration, plan_toss)
from toss_params import NAMES, TossParams, params_for_target

HERE = Path(__file__).resolve().parent

# START / END pose (same pose). Upside-down "J" / hook, seen from the side:
# pan centred toward the target; upper arm ~vertical, tilted slightly back; elbow
# bent ~90 deg so the forearm points forward and slightly up (wrist is the highest
# point); wrist bent sharply down, gripper closed; roll neutral.
# Read off the real arm (my_awesome_follower_arm calibration, LeRobot degrees,
# 0 = calibration midpoint; gripper in percent) on Nov 5 2026.
START_POSE = {
    "shoulder_pan": -0.44,
    "shoulder_lift": -14.07,
    "elbow_flex": -8.66,
    "wrist_flex": 92.57,   # NOTE: only ~4.5 deg below the calibrated limit (97.1)
    "wrist_roll": -6.81,
    "gripper": 1.0,        # closed
}
GRIPPER_CLOSED = 1.0       # percent; closed at START and through the wind-up

# Fixed motion shape (not per-throw variables):
WINDUP_RAMP_S = (0.58, 0.12)    # wind-up: speed up over 0.58 s, stop over 0.12 s
RELEASE_RAMP_S = (0.10, 0.033)  # release: speed up over 0.10 s, hard brake over 0.033 s
SLOW_FACTOR = 4.0               # --slow stretches the release 4x (25% speed, same shape)

# Release timing, same rule as Ball-Toss-In-Cup throw.release_delay(): open command
# at max(brake_start - RELEASE_LEAD_S, RELEASE_MIN_DELAY_S) from release start.
# xArm FAR: lead 0.43 s (~0.30 s gripper latency + 0.13 s early), min 0.10 s.
# SO-101 values are ESTIMATES (servo reacts in ~1-2 control steps); tune on the arm.
GRIPPER_LATENCY_S = 0.04
OPEN_BEFORE_BRAKE_S = 0.10
RELEASE_LEAD_S = GRIPPER_LATENCY_S + OPEN_BEFORE_BRAKE_S
RELEASE_MIN_DELAY_S = 0.05
# LeRobot caps the gripper servo at 50% torque, which slows opening. --go raises
# its runtime Torque_Limit (RAM, not saved; resets on power-off) and restores it.
GRIPPER_TORQUE_LIMIT = 1000

SLOW_VEL = 20.0            # deg/s cap for approach / return moves
SETTLE_TIME_S = 0.6        # hold after braking (ball leaves, servos settle)
RETURN_TIME_S = 2.5        # follow-through -> START
RATE_HZ = 50               # target streaming rate
LIMIT_MARGIN_DEG = 2.0     # stay this far inside calibrated encoder range
MAX_VEL_CAP = 250.0        # safety caps on the whole plan (deg/s, deg/s^2)
MAX_ACC_CAP = 7000.0
SLOW_TOLERANCE_DEG = 8.0   # abort slow phases if the arm lags this much
MAX_LATENESS_S = 0.05


def gripper_pct_per_deg(calibration: dict) -> float:
    """LeRobot gripper percent per degree of gripper-servo rotation."""
    c = calibration["gripper"]
    return 100.0 / ((c["range_max"] - c["range_min"]) * 360.0 / 4096.0)


def find_port() -> str | None:
    ports = sorted(glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/ttyACM*"))
    return ports[0] if len(ports) == 1 else None


def build_plan(current: dict, start: dict, limits: dict, slow: bool,
               params: TossParams, calibration: dict):
    params.validate(MAX_VEL_CAP)
    samples, windup, follow, info = plan_toss(
        current, start, params, gripper_closed=GRIPPER_CLOSED,
        gripper_pct_per_deg=gripper_pct_per_deg(calibration), rate_hz=RATE_HZ,
        slow_vel=SLOW_VEL, windup_ramp_s=WINDUP_RAMP_S, release_ramp_s=RELEASE_RAMP_S,
        release_lead=RELEASE_LEAD_S, release_min_delay=RELEASE_MIN_DELAY_S,
        settle_time=SETTLE_TIME_S, return_time=RETURN_TIME_S,
        release_time_scale=SLOW_FACTOR if slow else 1.0)
    check_limits(samples, limits)
    peaks = check_rates(samples, MAX_VEL_CAP, MAX_ACC_CAP)
    return samples, windup, follow, peaks, info


def report(samples, windup, follow, peaks, info, params, csv_path: Path):
    print("Per-throw variables (toss_params.py):")
    for n in NAMES:
        print(f"  {n:24s} {getattr(params, n):8.1f}")
    fmt = lambda p: "  ".join(f"{j}={p[j]:7.2f}" for j in JOINTS)
    print("START    ", fmt(START_POSE))
    print("WIND-UP  ", fmt(windup))
    print("FOLLOW   ", fmt(follow))
    for j in SWING_JOINTS:
        print(f"  {j:13s} planned peak {peaks[j][0]:5.0f} deg/s  {peaks[j][1]:5.0f} deg/s^2")
    print(f"Wind-up {info['windup_s']:.2f}s; release {info['release_s']:.2f}s, brake at "
          f"{info['brake_at_s']:.2f}s, gripper open cmd at {info['open_at_s']:.2f}s "
          f"(+{info['open_pct']:.1f}%)")
    print(f"Total {samples[-1].t:.2f}s, {len(samples)} samples -> {csv_path.name}")
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
    ap.add_argument("--params", type=Path, help="JSON of per-throw variables (toss_params.py names)")
    ap.add_argument("--distance", type=float, help="cup distance from base (m), for params_for_target")
    args = ap.parse_args()

    cal_file = args.calibration_dir / f"{args.robot_id}.json"
    calibration = json.loads(cal_file.read_text())
    limits = limits_from_calibration(calibration, LIMIT_MARGIN_DEG)
    params = TossParams.from_json(args.params) if args.params else params_for_target(args.distance)

    if not args.go:
        samples, windup, follow, peaks, info = build_plan(START_POSE, START_POSE, limits,
                                                          args.slow, params, calibration)
        report(samples, windup, follow, peaks, info, params, args.output)
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
    old_grip_torque = None
    try:
        # bus.connect() only (not robot.connect()): configure() would briefly drop
        # torque and rewrite PID registers. Only the gripper's runtime torque limit
        # is changed (restored in finally).
        robot.bus.connect()
        if not robot.is_calibrated:
            raise RuntimeError("motor calibration differs from the saved file; refusing")
        current = read()
        start = dict(START_POSE)
        samples, windup, follow, peaks, info = build_plan(current, start, limits, args.slow,
                                                          params, calibration)
        report(samples, windup, follow, peaks, info, params, args.output)
        print(f"\nLIVE on {port}. Clear the workspace; be ready to cut power.")
        if not args.yes:
            input("Press Enter to move (Ctrl-C to cancel)... ")
        # Hold in place: set every goal to its present position (raw ticks), then
        # enable torque. Safe whether or not a motor is already powered: a powered
        # motor whose goal differs (e.g. a stalled gripper) just stops pushing.
        raw = robot.bus.sync_read("Present_Position", normalize=False)
        robot.bus.sync_write("Goal_Position", raw, normalize=False)
        robot.bus.enable_torque(JOINTS)
        try:
            old_grip_torque = int(robot.bus.read("Torque_Limit", "gripper", normalize=False))
            robot.bus.write("Torque_Limit", "gripper", GRIPPER_TORQUE_LIMIT, normalize=False)
            print(f"Gripper torque limit {old_grip_torque} -> {GRIPPER_TORQUE_LIMIT} (restored on exit)")
        except Exception as e:
            old_grip_torque = None
            print("Could not raise gripper torque limit, opening at normal speed:", e)
        with log_path.open("w") as log:
            execute(samples, read, send, SLOW_TOLERANCE_DEG, MAX_LATENESS_S,
                    lambda e: log.write(json.dumps(e) + "\n"))
        end = read()
        print("Done. Final error vs START:",
              {j: round(end[j] - start[j], 1) for j in ARM_JOINTS}, f"log: {log_path.name}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if robot.bus.is_connected and old_grip_torque is not None:
            try:
                robot.bus.write("Torque_Limit", "gripper", old_grip_torque, normalize=False)
                print(f"Gripper torque limit restored to {old_grip_torque}")
            except Exception as e:
                print("Restoring gripper torque limit failed (power-cycle resets it):", e)
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
