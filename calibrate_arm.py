"""Interactive SO-101 calibration, stored only in this project."""
from pathlib import Path
import shutil
from datetime import datetime
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


def main():
    directory = Path(__file__).resolve().parent / 'calibration'
    directory.mkdir(exist_ok=True)
    saved = directory / 'my_awesome_follower_arm.json'
    if saved.exists():
        backup = saved.with_name(saved.stem + '.backup-' + datetime.now().strftime('%Y%m%d-%H%M%S') + '.json')
        shutil.copy2(saved, backup)
        print('Previous calibration backed up:', backup, flush=True)
    print('Support the arm: calibration disables torque. Keep fingers clear of joints.', flush=True)
    input('When you are holding/supporting the arm, press ENTER to connect and calibrate: ')
    robot = SO101Follower(SO101FollowerConfig(port='/dev/tty.usbmodem5C821074031',
        id='my_awesome_follower_arm', calibration_dir=directory, use_degrees=True,
        disable_torque_on_disconnect=False))
    try:
        robot.connect(calibrate=False)
        print("At the saved-calibration prompt, type c then ENTER. Do not reuse the stale file.", flush=True)
        robot.calibrate()
        if not robot.is_calibrated:
            raise RuntimeError('Motor values do not match the newly saved calibration')
        print('SUCCESS: saved calibration matches all motors.', flush=True)
        print('Calibration file:', robot.calibration_fpath, flush=True)
    finally:
        if robot.is_connected:
            robot.disconnect()
        print('Disconnected. Support the arm before switching power off.', flush=True)


if __name__ == '__main__':
    main()
