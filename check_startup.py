"""Inspect motor startup state. --seed-targets writes only unpowered hold targets."""
from pathlib import Path
import argparse
import json
from motor_startup import seed_targets


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--port',default='/dev/tty.usbmodem5C821074031')
    ap.add_argument('--seed-targets',action='store_true')
    args=ap.parse_args()
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    root=Path(__file__).resolve().parent
    r=SO101Follower(SO101FollowerConfig(port=args.port,id='my_awesome_follower_arm',
        calibration_dir=root/'calibration',use_degrees=True,disable_torque_on_disconnect=False))
    bus=r.bus
    try:
        bus.connect()
        if not bus.is_calibrated:raise RuntimeError('Calibration mismatch')
        if args.seed_targets:
            result=seed_targets(bus,r.calibration,list(bus.motors))
            print(json.dumps(result,indent=2),flush=True)
            print('SUCCESS: targets read back near measured positions; torque states unchanged.',flush=True)
        else:
            for joint in bus.motors:
                print(joint,{reg:bus.read(reg,joint,normalize=False) for reg in
                    ('Present_Position','Goal_Position','Torque_Enable')},flush=True)
    finally:
        if bus.is_connected:bus.disconnect(disable_torque=False)
        print('Serial port closed; no torque enable or movement sequence requested.',flush=True)


if __name__=='__main__':main()
