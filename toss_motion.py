"""SO-101 joint-space underhand toss model. Offline planning needs only Python."""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
SWING_JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")


@dataclass(frozen=True)
class Sample:
    time_s: float
    phase: str
    positions: dict[str, float]


def blend(u: float) -> float:
    """Minimum-jerk interpolation: zero velocity and acceleration at endpoints."""
    return 10*u**3 - 15*u**4 + 6*u**5


def finite(value: object) -> float:
    x = float(value)
    if not math.isfinite(x):
        raise ValueError("All motion values must be finite")
    return x


def plan(config: dict, speed: float = 1.0, release: bool = False) -> list[Sample]:
    """Speed is a trajectory time multiplier, NOT a servo speed register.

    Gripper opening is rate-limited in percentage points/s. Release time is the
    desired physical release time; opening starts earlier by the measured latency.
    This models joint motion, not ball flight or collision clearance.
    """
    if not 0.1 <= finite(speed) <= 3:
        raise ValueError("--speed must be 0.1..3")
    hz = finite(config["rate_hz"])
    if not 10 <= hz <= 100:
        raise ValueError("rate_hz must be 10..100")
    poses = config["poses"]
    for name in ("ready", "windup", "follow_through"):
        if set(poses[name]) != set(JOINTS):
            raise ValueError(f"{name} must specify exactly the six named actuators")
        for j in JOINTS:
            x = finite(poses[name][j])
            lo, hi = map(finite, config["limits"][j])
            if lo >= hi or not lo <= x <= hi:
                raise ValueError(f"{name}: {j} outside configured limits")
    for j in ("shoulder_pan", "wrist_roll", "gripper"):
        if len({poses[n][j] for n in poses}) != 1:
            raise ValueError(f"Keep {j} fixed in all three poses; release is configured separately")
    for j in SWING_JOINTS:
        a, b, c = (finite(poses[n][j]) for n in ("ready", "windup", "follow_through"))
        if (b-a)*(c-b) >= 0:
            raise ValueError(f"{j}: wind-up and swing must reverse direction")
    durations = {k: finite(config["durations_s"][k]) for k in ("windup", "swing", "return")}
    durations["swing"] /= speed
    if any(t <= 0 for t in durations.values()):
        raise ValueError("Durations must be positive")
    # Quintic interpolation has peak derivative 1.875 and peak second derivative 10/sqrt(3).
    for phase, a, b in (("windup", "ready", "windup"), ("swing", "windup", "follow_through"),
                        ("return", "follow_through", "ready")):
        for j in JOINTS[:-1]:
            delta = abs(poses[b][j] - poses[a][j])
            if 1.875*delta/durations[phase] > finite(config["max_velocity_deg_s"][j]):
                raise ValueError(f"{phase}: {j} exceeds velocity limit")
            if (10/math.sqrt(3))*delta/durations[phase]**2 > finite(config["max_acceleration_deg_s2"][j]):
                raise ValueError(f"{phase}: {j} exceeds acceleration limit")
    rel = config["release"]
    fraction = finite(rel["swing_fraction"])
    latency = finite(rel["latency_s"])
    opening_rate = finite(rel["opening_rate_pct_s"])
    hold = finite(poses["ready"]["gripper"])
    opened = finite(rel["open_pct"])
    if not 0 < fraction < 1 or latency < 0 or opening_rate <= 0:
        raise ValueError("Invalid release fraction, latency, or opening rate")
    lo, hi = config["limits"]["gripper"]
    if not 0 <= lo < hi <= 100 or not hold < opened <= hi:
        raise ValueError("Gripper must open to a larger percentage within 0..100")
    start_open = durations["windup"] + fraction*durations["swing"] - latency
    if release and start_open < durations["windup"]:
        raise ValueError("Release latency would require opening during wind-up; slow the swing or release later")
    samples = [Sample(0.0, "ready", dict(poses["ready"]))]
    offset = 0.0
    for phase, a, b in (("windup", "ready", "windup"), ("swing", "windup", "follow_through"),
                        ("return", "follow_through", "ready")):
        duration = durations[phase]
        count = math.ceil(duration*hz)
        for i in range(1, count+1):
            u = i/count
            t = offset + duration*u
            q = {j: poses[a][j] + (poses[b][j]-poses[a][j])*blend(u) for j in JOINTS}
            if release:
                q["gripper"] = min(opened, hold + max(0.0, t-start_open)*opening_rate)
            samples.append(Sample(t, phase, q))
        offset += duration
    return samples


def execute(samples: list[Sample], read: Callable[[], dict[str, float]],
            send: Callable[[dict[str, float]], None], tolerance: float,
            max_lateness: float, record: Callable[[dict], None]) -> None:
    """Stream against absolute deadlines. Abort rather than catch up a stalled bus.

    On failure, stop changing targets; do not attempt an unplanned home move.
    Caller disconnects while retaining servo torque to prevent a gravity fall.
    """
    initial = read()
    if any(abs(initial[j]-samples[0].positions[j]) > tolerance for j in JOINTS):
        raise RuntimeError("Arm must already be at the measured ready pose before execution")
    start = time.monotonic()
    for s in samples:
        remaining = start+s.time_s-time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        if time.monotonic()-(start+s.time_s) > max_lateness:
            raise RuntimeError("Control loop missed its deadline; aborting trajectory")
        actual = read()
        if any(abs(actual[j]-s.positions[j]) > tolerance for j in JOINTS):
            raise RuntimeError("Tracking error too large; aborting trajectory")
        send(s.positions)
        record({"planned_time_s": s.time_s, "actual_time_s": time.monotonic()-start,
                "phase": s.phase, "target": s.positions, "measured_before_command": actual})
