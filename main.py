"""Execution entry point. No hardware is imported or contacted in preview mode."""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path

from toss_motion import JOINTS, execute, plan


def main() -> None:
    parser = argparse.ArgumentParser(description="SO-101 underhand toss: dry run by default")
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("toss.example.json"))
    parser.add_argument("--output", type=Path, default=Path("trajectory.csv"))
    parser.add_argument("--speed", type=float, default=1.0, help="swing time multiplier (1 = configured duration)")
    parser.add_argument("--release", action="store_true", help="enable gripper opening; otherwise stays closed")
    parser.add_argument("--go", action="store_true", help="execute measured, reviewed profile on real hardware")
    parser.add_argument("--port", help="follower USB serial port")
    parser.add_argument("--robot-id", default="my_awesome_follower_arm")
    parser.add_argument("--calibration-dir", type=Path,
                        default=Path(__file__).resolve().parent / "calibration")
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    samples = plan(config, args.speed, args.release)
    with args.output.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["time_s", "phase", *JOINTS])
        writer.writerows([s.time_s, s.phase, *(s.positions[j] for j in JOINTS)] for s in samples)
    print(f"Plan: {len(samples)} samples, {samples[-1].time_s:.2f}s; saved {args.output}")
    print("Ready -> wind-up -> forward swing -> return to ready")
    print("Release enabled" if args.release else "Gripper remains closed")
    if not args.go:
        print("DRY RUN: no robot connection or motion")
        return
    if not config.get("hardware_validated") or not config.get("swept_path_reviewed"):
        parser.error("Example profile is preview-only. Measure poses/limits and review the swept path first.")
    if not args.port or args.calibration_dir is None:
        parser.error("--go requires --port and --calibration-dir containing copied calibration")
    calibration_file = args.calibration_dir / f"{args.robot_id}.json"
    if not calibration_file.is_file():
        parser.error(f"Calibration file missing: {calibration_file}")
    # Public LeRobot interface verified against the user's installed source.
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.robot_id,
        calibration_dir=args.calibration_dir, use_degrees=True, disable_torque_on_disconnect=False))
    if not robot.calibration:
        parser.error("Saved calibration is required; this program never runs calibration")
    print("LIVE: keep the workspace clear and have someone at the power disconnect.")
    log_path = args.output.with_suffix(".execution.jsonl")
    try:
        robot.connect(calibrate=False)
        if not robot.is_calibrated:
            raise RuntimeError("Motor calibration differs from the saved file; refusing motion")
        # Bound user-defined limits by the actual calibrated motor ranges.
        for j in JOINTS[:-1]:
            cal = robot.calibration[j]
            resolution = robot.bus.resolution(robot.bus.motors[j].model)
            half_range = (cal.range_max-cal.range_min)*180/(resolution-1)
            lo, hi = config["limits"][j]
            if lo < -half_range or hi > half_range:
                raise ValueError(f"Configured {j} limits exceed the calibrated motor range")
        def read():
            obs = robot.get_observation()
            return {j: float(obs[f"{j}.pos"]) for j in JOINTS}
        def send(q):
            robot.send_action({f"{j}.pos": q[j] for j in JOINTS})
        with log_path.open("w") as log:
            execute(samples, read, send, config["tracking_tolerance"], config["max_lateness_s"],
                    lambda event: log.write(json.dumps(event)+"\n"))
        print(f"Completed; measurements saved to {log_path}")
    finally:
        if robot.is_connected:
            robot.disconnect()
        print("Connection closed; torque retained. Support the arm before disconnecting power.")


if __name__ == "__main__":
    main()
