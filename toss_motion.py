"""SO-101 underhand toss: trajectory planning, limit checks, and streaming.

Pure Python (no hardware imports) so the plan can be dry-run anywhere.
The toss copies the xArm "far" preset (Ball-Toss-In-Cup/throw.py):
all three pitch joints move together on one synchronized time base, ramp up to a
cruise speed, then brake hard. The ball leaves because the arm stops.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
SWING_JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")
ARM_JOINTS = JOINTS[:-1]


@dataclass(frozen=True)
class Sample:
    t: float            # seconds from the start of the plan
    phase: str          # approach | windup | swing | settle | return
    q: dict             # joint -> LeRobot degrees (gripper: percent)


# ---------------------------------------------------------------- limits
def limits_from_calibration(calibration: dict, margin_deg: float) -> dict:
    """Joint limits in LeRobot degree units (0 = calibration midpoint, 4096 ticks = 360 deg)."""
    out = {}
    for j in ARM_JOINTS:
        c = calibration[j]
        half = (c["range_max"] - c["range_min"]) * 180.0 / 4095.0
        out[j] = (-half + margin_deg, half - margin_deg)
    out["gripper"] = (0.0, 100.0)
    return out


def check_limits(samples: list[Sample], limits: dict) -> None:
    for s in samples:
        for j in JOINTS:
            lo, hi = limits[j]
            if not (lo <= s.q[j] <= hi) or not math.isfinite(s.q[j]):
                raise ValueError(f"{s.phase} t={s.t:.2f}s: {j}={s.q[j]:.1f} outside [{lo:.1f}, {hi:.1f}]")


def check_rates(samples: list[Sample], max_vel: float, max_acc: float) -> dict:
    """Finite-difference peak speed/accel per joint; raise if above the caps."""
    peaks = {j: [0.0, 0.0] for j in ARM_JOINTS}
    for a, b, c in zip(samples, samples[1:], samples[2:]):
        for j in ARM_JOINTS:
            v1 = (b.q[j] - a.q[j]) / (b.t - a.t)
            v2 = (c.q[j] - b.q[j]) / (c.t - b.t)
            peaks[j][0] = max(peaks[j][0], abs(v1), abs(v2))
            peaks[j][1] = max(peaks[j][1], abs(v2 - v1) / ((c.t - a.t) / 2))
    for j, (v, acc) in peaks.items():
        if v > max_vel * 1.001:
            raise ValueError(f"{j} peak speed {v:.0f} deg/s exceeds cap {max_vel:.0f}")
        if acc > max_acc * 1.05:
            raise ValueError(f"{j} peak accel {acc:.0f} deg/s^2 exceeds cap {max_acc:.0f}")
    return peaks


# ---------------------------------------------------------------- profiles
def min_jerk(u: float) -> float:
    """Quintic 0->1 with zero velocity/acceleration at both ends (slow moves)."""
    return 10 * u**3 - 15 * u**4 + 6 * u**5


def trapezoid(u: float, fa: float, fd: float) -> float:
    """Normalized 0->1 trapezoid: accelerate over the first `fa` of the time,
    cruise, decelerate over the last `fd`. A short `fd` means a quick stop
    instead of min-jerk's long, slow creep into the pose."""
    v = 1.0 / (1.0 - fa / 2 - fd / 2)          # cruise speed (normalized)
    if u <= fa:
        return 0.5 * v / fa * u * u
    if u <= 1 - fd:
        return 0.5 * v * fa + v * (u - fa)
    r = 1.0 - u
    return 1.0 - 0.5 * v / fd * r * r


def slow_move(a: dict, b: dict, max_vel: float, min_time: float) -> float:
    """Duration so a min-jerk move a->b stays under max_vel (peak = 1.875 * d / T)."""
    d = max(abs(b[j] - a[j]) for j in ARM_JOINTS)
    return max(min_time, 1.875 * d / max_vel)


def brake_profile(peak_vel: float, accel: float, brake: float, dist: float):
    """Trapezoid for the lead joint: ramp at `accel`, cruise at `peak_vel`, stop at `brake`.

    Returns (duration, s(t)) with s normalized 0->1. This mirrors the xArm FAR swing:
    servo-mode move at constant speed with ACCEL ramps, ending in an abrupt stop.
    """
    t_a, t_d = peak_vel / accel, peak_vel / brake
    d_a, d_d = 0.5 * accel * t_a**2, 0.5 * brake * t_d**2
    if d_a + d_d > dist:                       # triangular: never reaches cruise
        peak_vel = math.sqrt(2 * dist / (1 / accel + 1 / brake))
        t_a, t_d = peak_vel / accel, peak_vel / brake
        d_a, d_d = 0.5 * accel * t_a**2, 0.5 * brake * t_d**2
    t_c = (dist - d_a - d_d) / peak_vel
    total = t_a + t_c + t_d
    t_brake = t_a + t_c                        # when braking starts (s from swing start)

    def s(t: float) -> float:
        if t <= 0:
            return 0.0
        if t < t_a:
            x = 0.5 * accel * t**2
        elif t < t_a + t_c:
            x = d_a + peak_vel * (t - t_a)
        elif t < total:
            r = total - t
            x = dist - 0.5 * brake * r**2
        else:
            x = dist
        return x / dist
    return total, s, t_brake


def release_time(t_brake: float, lead: float, min_delay: float) -> float:
    """Open-command time (s from swing start), same rule as Ball-Toss-In-Cup
    throw.release_delay(): command `lead` seconds before braking starts so the
    fingers are open as the arm brakes, but never earlier than `min_delay`."""
    return max(t_brake - lead, min_delay)


# ---------------------------------------------------------------- plan
def plan_toss(current: dict, start: dict, windup: dict, follow: dict, *, rate_hz: float,
              peak_vel: float, accel: float, brake: float, slow_vel: float,
              windup_time: float, settle_time: float, return_time: float,
              time_scale: float = 1.0, gripper_closed: float | None = None,
              gripper_open: float | None = None, release_lead: float = 0.0,
              release_min_delay: float = 0.0, gripper_open_cmd: float | None = None,
              windup_accel_frac: float = 0.4,
              windup_decel_frac: float = 0.15) -> list[Sample]:
    """current -> START (slow) -> wind-up (slow) -> SWING + hard brake -> settle -> START (slow).

    time_scale > 1 stretches only the swing (e.g. 4 = 25% speed, same shape).
    Pan and roll are held at START. Gripper: closed (start/windup values) until the
    open command time in the swing, then stepped to `follow['gripper']` (open);
    the return closes it again. Release timing is computed in un-scaled swing time
    then stretched with the swing, so --slow keeps the same release point.
    """
    dt = 1.0 / rate_hz
    out: list[Sample] = []
    t0 = 0.0

    def add_min_jerk(a, b, T, phase, profile=min_jerk):
        nonlocal t0
        n = max(1, math.ceil(T * rate_hz))
        for i in range(1, n + 1):
            u = profile(i / n)
            out.append(Sample(t0 + T * i / n, phase, {j: a[j] + (b[j] - a[j]) * u for j in JOINTS}))
        t0 += T

    out.append(Sample(0.0, "approach", dict(current)))
    add_min_jerk(current, start, slow_move(current, start, slow_vel, 2.0), "approach")
    # Wind-up: gentle start, short stop, so it flows straight into the swing
    # (min-jerk spent its last ~0.3 s creeping <1 deg, which looked like a pause).
    add_min_jerk(start, windup, windup_time, "windup",
                 lambda u: trapezoid(u, windup_accel_frac, windup_decel_frac))

    lead = max(SWING_JOINTS, key=lambda j: abs(follow[j] - windup[j]))
    T, s, t_brake = brake_profile(peak_vel, accel, brake, abs(follow[lead] - windup[lead]))
    t_open = release_time(t_brake, release_lead, release_min_delay)
    T_real = T * time_scale
    n = max(2, math.ceil(T_real * rate_hz))
    for i in range(1, n + 1):
        u = s(T * i / n)
        q = {j: windup[j] + (follow[j] - windup[j]) * u for j in JOINTS}
        # Overdrive: the servo's speed grows with position error, so command a
        # wider target than the final opening during the swing, then settle back.
        opened = follow["gripper"] if gripper_open_cmd is None else gripper_open_cmd
        q["gripper"] = opened if T * i / n >= t_open else windup["gripper"]
        out.append(Sample(t0 + T_real * i / n, "swing", q))
    t0 += T_real
    out.append(Sample(t0 + settle_time, "settle", dict(follow)))   # hold: ball flies off
    t0 += settle_time
    add_min_jerk(follow, start, return_time, "return")
    return out


# ---------------------------------------------------------------- execution
def execute(samples: list[Sample], read: Callable[[], dict], send: Callable[[dict], None],
            slow_tolerance: float, max_lateness: float, log: Callable[[dict], None]) -> None:
    """Stream targets on absolute deadlines.

    Slow phases read the arm every step and abort if tracking error > slow_tolerance.
    The swing skips reads to keep timing tight (servo lag there is expected).
    On any exception the caller holds the arm where it is.
    """
    first = read()
    for j in JOINTS[:-1]:
        if abs(first[j] - samples[0].q[j]) > slow_tolerance:
            raise RuntimeError(f"{j} moved since planning ({first[j]:.1f} vs {samples[0].q[j]:.1f})")
    t_start = time.monotonic()
    for s in samples:
        wait = t_start + s.t - time.monotonic()
        if wait > 0:
            time.sleep(wait)
        late = time.monotonic() - (t_start + s.t)
        if late > max_lateness and s.phase != "settle":
            raise RuntimeError(f"control loop {late*1000:.0f} ms late in {s.phase}; aborting")
        actual = None
        if s.phase in ("approach", "windup", "return"):
            actual = read()
            err = max(abs(actual[j] - s.q[j]) for j in ARM_JOINTS)
            if err > slow_tolerance:
                raise RuntimeError(f"tracking error {err:.1f} deg in {s.phase}; aborting")
        send(s.q)
        log({"t_plan": s.t, "t_actual": time.monotonic() - t_start, "phase": s.phase,
             "target": s.q, "measured": actual})
