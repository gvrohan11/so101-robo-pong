"""The 14 per-throw toss variables.

Before each throw (ball in hand, cup located) these are computed from the cup's
distance by `params_for_target()` - today a fixed default, later a tuned table,
regression, or learned RL/ML policy. Everything else in main.py (START pose,
limits, control rate, release-timing rule) stays fixed.

Units: joint angles in LeRobot degrees (calibrated), speeds in deg/s.
Movement = signed change over that phase (start -> finish of the phase).
Signs on the SO-101: shoulder_lift + = upper arm forward; elbow_flex + = forearm
folds in / down; wrist_flex + = gripper tucks back. Gripper + = opening.
Speeds are each joint's PEAK speed in that phase. Joints start together; each
joint's phase time = |movement| / speed + ramp time, so keep speed proportional
to movement if you want the joints to finish together.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, fields


@dataclass
class TossParams:
    # ---- wind-up: START -> wind-up pose
    windup_move_shoulder: float = 20.0     # joint 2, deg
    windup_move_elbow: float = 44.0        # joint 3, deg
    windup_move_wrist: float = -6.9        # joint 4, deg
    windup_speed_shoulder: float = 18.1    # joint 2, deg/s (peak)
    windup_speed_elbow: float = 39.8       # joint 3, deg/s (peak)
    windup_speed_wrist: float = 6.2        # joint 4, deg/s (peak)
    # ---- release: wind-up pose -> follow-through (then the arm brakes hard)
    release_move_shoulder: float = -30.0   # joint 2, deg
    release_move_elbow: float = -52.0      # joint 3, deg
    release_move_wrist: float = -15.0      # joint 4, deg
    release_speed_shoulder: float = 130.0  # joint 2, deg/s (peak)
    release_speed_elbow: float = 225.0     # joint 3, deg/s (peak)
    release_speed_wrist: float = 65.0      # joint 4, deg/s (peak)
    # ---- gripper: closed through the wind-up, opens during the release
    gripper_move: float = 52.1             # deg of gripper-servo rotation (52.1 = 1% -> 40%)
    gripper_release_speed: float = 600.0   # deg/s commanded opening rate

    # helpers ------------------------------------------------------------
    def windup_move(self) -> dict:
        return {"shoulder_lift": self.windup_move_shoulder, "elbow_flex": self.windup_move_elbow,
                "wrist_flex": self.windup_move_wrist}

    def windup_speed(self) -> dict:
        return {"shoulder_lift": self.windup_speed_shoulder, "elbow_flex": self.windup_speed_elbow,
                "wrist_flex": self.windup_speed_wrist}

    def release_move(self) -> dict:
        return {"shoulder_lift": self.release_move_shoulder, "elbow_flex": self.release_move_elbow,
                "wrist_flex": self.release_move_wrist}

    def release_speed(self) -> dict:
        return {"shoulder_lift": self.release_speed_shoulder, "elbow_flex": self.release_speed_elbow,
                "wrist_flex": self.release_speed_wrist}

    def to_vector(self) -> list[float]:
        """Fixed-order vector (for ML/RL action spaces). Order = NAMES."""
        return [getattr(self, n) for n in NAMES]

    @classmethod
    def from_vector(cls, v) -> "TossParams":
        return cls(**dict(zip(NAMES, map(float, v))))

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_json(cls, path) -> "TossParams":
        with open(path) as f:
            return cls(**json.load(f))

    def validate(self, max_speed: float) -> None:
        for n in NAMES:
            if "speed" in n:
                v = getattr(self, n)
                cap = max_speed if n != "gripper_release_speed" else 2000.0
                if not 0 < v <= cap:
                    raise ValueError(f"{n}={v} must be in (0, {cap}] deg/s")
        if self.gripper_move < 0:
            raise ValueError("gripper_move must be >= 0 (it opens)")


NAMES = tuple(f.name for f in fields(TossParams))
DEFAULT = TossParams()


def params_for_target(distance_m: float | None = None) -> TossParams:
    """Per-throw variables from the cup's distance from the arm base (metres).

    Placeholder: returns the hand-tuned default. Replace with a lookup table,
    fit, or trained policy; whatever it returns is checked against joint limits
    and speed caps before the arm moves.
    """
    return TossParams()
