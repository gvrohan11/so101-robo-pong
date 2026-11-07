"""Turn off torque on all SO-101 servos. THE ARM WILL DROP: support it first."""
import glob, sys, time
from pathlib import Path
try:
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
except ImportError:
    from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

ports = sys.argv[1:] or sorted(glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/ttyACM*"))
if len(ports) != 1:
    sys.exit(f"Pass the port explicitly; found {ports}")
r = SO101Follower(SO101FollowerConfig(port=ports[0], id="my_awesome_follower_arm",
                  calibration_dir=Path(__file__).resolve().parent / "calibration"))
r.bus.connect()
for i in (5, 4, 3, 2, 1):
    print(f"Support the arm! Torque off in {i}...", flush=True); time.sleep(1)
r.bus.disable_torque()
r.bus.disconnect(disable_torque=False)
print("Torque OFF.")
