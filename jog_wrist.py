"""Small wrist-direction test. Dry-run by default; independent of setup scripts."""
from pathlib import Path
import argparse
import math
import time
from motor_startup import seed_targets


def smooth(t):
    return 10*t**3-15*t**4+6*t**5


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--go',action='store_true')
    ap.add_argument('--port',default='/dev/tty.usbmodem5C821074031')
    args=ap.parse_args()
    if not args.go:
        print('Preview: wrist_flex only, 1 degree toward midpoint in 2 s, then back in 2 s. No connection.')
        return
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    root=Path(__file__).resolve().parent
    robot=SO101Follower(SO101FollowerConfig(port=args.port,id='my_awesome_follower_arm',
        calibration_dir=root/'calibration',use_degrees=True,disable_torque_on_disconnect=False))
    joint='wrist_flex'
    bus=robot.bus
    try:
        bus.connect()
        if not bus.is_calibrated:
            raise RuntimeError('Calibration mismatch; no movement')
        # Do not run configure(): it changes gains and temporarily releases torque.
        start=float(bus.read('Present_Position',joint))
        goal=float(bus.read('Goal_Position',joint))
        powered=int(bus.read('Torque_Enable',joint,normalize=False))
        if not math.isfinite(start):raise RuntimeError('Invalid encoder reading')
        cal=robot.calibration[joint]
        half=(cal.range_max-cal.range_min)*180/(bus.resolution(bus.motors[joint].model)-1)
        target=start-1 if start>=0 else start+1
        if not all(-half+2<=q<=half-2 for q in (start,target)):
            raise RuntimeError('Too close to calibrated limit')
        if powered and abs(goal-start)>2:
            raise RuntimeError('Existing target differs from actual position; another command may be active')
        print(f'wrist_flex: {start:.3f} -> {target:.3f} -> {start:.3f} deg',flush=True)
        seed_targets(bus,robot.calibration,[joint])
        if not powered:
            bus.enable_torque([joint])
        if int(bus.read('Torque_Enable',joint,normalize=False)) != 1:
            raise RuntimeError('Wrist torque did not enable; no jog')
        if abs(float(bus.read('Goal_Position',joint))-start) > .2:
            raise RuntimeError('Hold target changed on enable; refusing jog')
        began=time.monotonic()
        for leg,(a,b) in enumerate(((start,target),(target,start))):
            for i in range(1,41):
                deadline=began+leg*2+i*.05
                time.sleep(max(0,deadline-time.monotonic()))
                if time.monotonic()-deadline>.15:
                    raise RuntimeError('Missed motion deadline; stopped advancing targets')
                actual=float(bus.read('Present_Position',joint))
                if not math.isfinite(actual) or abs(actual-start)>2:
                    raise RuntimeError('Unexpected encoder motion; stopped advancing targets')
                desired=a+(b-a)*smooth(i/40)
                bus.write('Goal_Position',joint,desired)
                stored=float(bus.read('Goal_Position',joint))
                if abs(stored-desired)>.2 or not int(bus.read('Torque_Enable',joint,normalize=False)):
                    raise RuntimeError('Target write or torque verification failed; stopping jog')
            print(f'Leg {leg+1}: measured {float(bus.read("Present_Position",joint)):.3f} deg',flush=True)
        time.sleep(.3)
        final=float(bus.read('Present_Position',joint))
        if abs(final-start)>.7:raise RuntimeError(f'Return not verified: final {final:.3f}, initial {start:.3f}')
        print(f'COMPLETE: final {final:.3f} deg. Gripper and other joint targets unchanged.',flush=True)
    finally:
        if bus.is_connected:bus.disconnect(disable_torque=False)
        print('Disconnected with disable_torque=False; actual torque must be checked before the next run.',flush=True)


if __name__=='__main__':main()
