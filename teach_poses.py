"""Keyboard control for an SO-101 follower.

Arm joints: degrees.
Gripper: percentage of calibrated range (0–100).

Space toggles pause. Escape exits.
"""

import json
from pathlib import Path
import threading
import time

from pynput import keyboard

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


PORT = "/dev/tty.usbmodem5C821074031"
ROBOT_ID = "my_awesome_follower_arm"
FPS = 30

ARM_SPEED = 10.0       # Degrees per second
GRIPPER_SPEED = 20.0   # Percentage points per second
MAX_DT = 0.05         # Prevent large steps after a slow loop

# Positive / negative direction for each joint.
CONTROLS = {
    "shoulder_pan": ("q", "a"),
    "shoulder_lift": ("w", "s"),
    "elbow_flex": ("e", "d"),
    "wrist_flex": ("r", "f"),
    "wrist_roll": ("t", "g"),
    "gripper": ("y", "h"),
}
CONTROL_KEYS = {key for pair in CONTROLS.values() for key in pair}


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def get_joint_limits(robot):
    """Match this installed LeRobot's degree normalization."""
    limits = {}

    for joint in CONTROLS:
        if joint == "gripper":
            limits[joint] = (0.0, 100.0)
            continue

        calibration = robot.calibration[joint]
        motor = robot.bus.motors[joint]
        resolution = robot.bus.resolution(motor.model)
        midpoint = (
            calibration.range_min + calibration.range_max
        ) / 2.0
        degrees_per_tick = 360.0 / (resolution - 1)

        limits[joint] = (
            (calibration.range_min - midpoint) * degrees_per_tick,
            (calibration.range_max - midpoint) * degrees_per_tick,
        )

    return limits


def main():
    robot = SO101Follower(
        SO101FollowerConfig(
            port=PORT,
            id=ROBOT_ID,
            use_degrees=True,
            calibration_dir=Path(__file__).resolve().parent / "calibration",
            disable_torque_on_disconnect=False,
        )
    )

    if not robot.calibration:
        raise FileNotFoundError(
            f"No saved calibration at {robot.calibration_fpath}. "
            "Calibrate my_follower first."
        )

    lock = threading.Lock()
    stop = threading.Event()
    pressed = set()
    paused = True
    listener = None
    connected = False
    pending_capture = None
    taught = {}

    def on_press(key):
        nonlocal paused, pending_capture

        if key == keyboard.Key.esc:
            stop.set()
            return False

        char = getattr(key, "char", None)
        token = "space" if key == keyboard.Key.space else (
            char.lower() if char else None
        )

        if token in ("1", "2", "3"):
            with lock:
                if paused:
                    pending_capture = {"1": "ready", "2": "windup", "3": "follow_through"}[token]
                else:
                    print("Pause with Space before recording a pose.", flush=True)
            return None
        if token != "space" and token not in CONTROL_KEYS:
            return None

        with lock:
            first_press = token not in pressed
            pressed.add(token)

            if token == "space" and first_press:
                paused = not paused
                # Require fresh movement presses after pause/resume.
                pressed.intersection_update({"space"})
                print(
                    "\nPAUSED — release movement keys."
                    if paused else
                    "\nACTIVE — hold a movement key.",
                    flush=True,
                )
            elif first_press:
                print(f"Key received: {token.upper()}", flush=True)

        return None

    def on_release(key):
        char = getattr(key, "char", None)
        token = "space" if key == keyboard.Key.space else (
            char.lower() if char else None
        )
        with lock:
            pressed.discard(token)

    try:
        # Check input permission before connecting to the arm.
        # if not getattr(keyboard.Listener, "IS_TRUSTED", True):
        #     raise RuntimeError(
        #         "macOS has not granted keyboard monitoring permission. "
        #         "Enable Accessibility / Input Monitoring for your terminal "
        #         "in System Settings > Privacy & Security, then restart it."
        #     )

        listener = keyboard.Listener(
            on_press=on_press,
            on_release=on_release,
        )
        listener.start()
        listener.wait()

        print("Connecting to follower...", flush=True)
        robot.connect(calibrate=False)
        connected = True
        if not robot.is_calibrated:
            raise RuntimeError("Saved calibration differs from motor calibration; refusing control")

        limits = get_joint_limits(robot)
        observation = robot.get_observation()
        targets = {
            joint: float(observation[f"{joint}.pos"])
            for joint in CONTROLS
        }

        # Do not silently clamp an out-of-range startup position:
        # that would command a different pose immediately.
        for joint, value in targets.items():
            lower, upper = limits[joint]
            if not lower <= value <= upper:
                raise RuntimeError(
                    f"{joint}: current position {value:.2f} is outside "
                    f"the calibrated range [{lower:.2f}, {upper:.2f}]. "
                    "Check the pose and calibration before continuing."
                )

        print(
            "\nConnected. Starting PAUSED.\n"
            "Q/A: shoulder pan    W/S: shoulder lift\n"
            "E/D: elbow flex      R/F: wrist flex\n"
            "T/G: wrist roll      Y/H: gripper\n"
            "Space: pause/resume  Escape: exit\n"
            "While PAUSED: 1 saves ready, 2 windup, 3 follow-through.\n"
            "Only use keys in this Terminal; listener is system-wide. Pause before changing apps.\n"
            "Press a movement key while paused to check input.\n"
            "Keep this terminal focused while operating.\n",
            flush=True,
        )

        tick = 1.0 / FPS
        previous = time.monotonic()

        while not stop.is_set():
            started = time.monotonic()
            dt = min(started - previous, MAX_DT)
            previous = started

            if not listener.is_alive():
                raise RuntimeError(
                    "Keyboard listener stopped. Ending control."
                )

            with lock:
                keys = pressed.copy()
                is_paused = paused
                capture = pending_capture
                pending_capture = None
            if capture:
                observation = robot.get_observation()
                taught[capture] = {joint: float(observation[f"{joint}.pos"]) for joint in CONTROLS}
                output = Path(__file__).resolve().parent / "taught_poses.local.json"
                output.write_text(json.dumps({"poses": taught, "units": "arm degrees; gripper percent"}, indent=2)+"\n")
                print(f"Saved {capture} to {output}", flush=True)

            if not is_paused:
                for joint, (positive, negative) in CONTROLS.items():
                    direction = int(positive in keys) - int(negative in keys)
                    speed = (
                        GRIPPER_SPEED if joint == "gripper"
                        else ARM_SPEED
                    )
                    lower, upper = limits[joint]
                    targets[joint] = clamp(
                        targets[joint] + direction * speed * dt,
                        lower,
                        upper,
                    )

            # Paused or released keys retain the last commanded pose.
            robot.send_action({
                f"{joint}.pos": target
                for joint, target in targets.items()
            })

            stop.wait(max(0.0, tick - (time.monotonic() - started)))

    except KeyboardInterrupt:
        print("\nInterrupted.", flush=True)
    finally:
        stop.set()
        if listener is not None:
            listener.stop()
        if connected or robot.is_connected:
            robot.disconnect()
        print("Disconnected; torque retained. Support arm before powering off.", flush=True)


if __name__ == "__main__":
    main()