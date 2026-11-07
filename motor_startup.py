"""Verified startup target writes; no automatic torque enabling or calibration writes."""
from __future__ import annotations
from collections.abc import Sequence
from typing import Any


def seed_targets(bus: Any, calibration: dict, joints: Sequence[str]) -> list[dict]:
    """Seed unpowered motors using raw encoder ticks, with acknowledged read-back.

    Powered motors are left untouched and must already target their current pose.
    A failure never enables torque. Caller must reverify targets immediately before
    any later enable. Calibration ranges are encoder limits, not collision checks.
    """
    snapshots = []
    for joint in joints:
        cal = calibration[joint]
        position = int(bus.read('Present_Position', joint, normalize=False))
        target = int(bus.read('Goal_Position', joint, normalize=False))
        torque = int(bus.read('Torque_Enable', joint, normalize=False))
        if torque not in (0, 1):
            raise RuntimeError(f'{joint}: unknown torque state {torque}')
        if not cal.range_min <= position <= cal.range_max:
            raise RuntimeError(f'{joint}: encoder position outside saved calibration')
        if torque and abs(target-position) > 5:
            raise RuntimeError(f'{joint}: powered motor has a different target; stop other controllers')
        snapshots.append(dict(joint=joint, position_ticks=position, old_target_ticks=target, torque=torque))
    for item in snapshots:
        joint = item['joint']
        if int(bus.read('Torque_Enable', joint, normalize=False)) != item['torque']:
            raise RuntimeError(f'{joint}: torque changed during preflight')
        if not item['torque']:
            bus.write('Goal_Position', joint, item['position_ticks'], normalize=False)
        goal = int(bus.read('Goal_Position', joint, normalize=False))
        actual = int(bus.read('Present_Position', joint, normalize=False))
        if abs(goal-item['position_ticks']) > 1 or abs(actual-goal) > 5:
            raise RuntimeError(f'{joint}: target write/read-back or stability check failed')
        if int(bus.read('Torque_Enable', joint, normalize=False)) != item['torque']:
            raise RuntimeError(f'{joint}: torque changed during target verification')
        item['verified_target_ticks'] = goal
        item['verified_position_ticks'] = actual
    return snapshots
