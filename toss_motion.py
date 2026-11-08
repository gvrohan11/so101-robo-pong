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


def slow_move(a: dict, b: dict, max_vel: float, min_time: float) -> float:
    """Duration so a min-jerk move a->b stays under max_vel (peak = 1.875 * d / T)."""
    d = max(abs(b[j] - a[j]) for j in ARM_JOINTS)
    return max(min_time, 1.875 * d / max_vel)


def release_time(t_brake: float, lead: float, min_delay: float) -> float:
    """Open-command time (s from swing start), same rule as Ball-Toss-In-Cup
    throw.release_delay(): command `lead` seconds before braking starts so the
    fingers are open as the arm brakes, but never earlier than `min_delay`."""
    return max(t_brake - lead, min_delay)


# ---------------------------------------------------------------- plan
def ramp(dist: float, vmax: float, t_acc: float, t_dec: float):
    """One joint's move of signed `dist` at peak speed `vmax` (deg/s), speeding up
    over `t_acc` s and stopping over `t_dec` s. Returns (duration, brake_start, pos(t))."""
    d = abs(dist)
    if d == 0:
        return 0.0, 0.0, lambda t: 0.0
    v = min(vmax, 2 * d / (t_acc + t_dec))     # too short to reach vmax: triangle
    ta, td = v / vmax * t_acc, v / vmax * t_dec
    T = d / v + (ta + td) / 2
    sign = 1.0 if dist > 0 else -1.0

    def pos(t: float) -> float:
        if t <= 0:
            x = 0.0
        elif t < ta:
            x = 0.5 * v / ta * t * t
        elif t < T - td:
            x = 0.5 * v * ta + v * (t - ta)
        elif t < T:
            r = T - t
            x = d - 0.5 * v / td * r * r
        else:
            x = d
        return sign * x
    return T, T - td, pos


def release_time(t_brake: float, lead: float, min_delay: float) -> float:
    """Open-command time (s from release start), same rule as Ball-Toss-In-Cup
    throw.release_delay(): `lead` s before braking starts, never before `min_delay`."""
    return max(t_brake - lead, min_delay)


def plan_toss(current: dict, start: dict, p, *, gripper_closed: float, gripper_pct_per_deg: float,
              rate_hz: float, slow_vel: float, windup_ramp_s: tuple, release_ramp_s: tuple,
              release_lead: float, release_min_delay: float, settle_time: float,
              return_time: float, release_time_scale: float = 1.0):
    """current -> START (slow) -> WIND-UP -> RELEASE + hard brake -> settle -> START (slow).

    `p` is a TossParams. Pan and roll are held at START. Gripper is closed until the
    open time in the release, then opens at p.gripper_release_speed by p.gripper_move,
    and closes on the return. `release_time_scale` > 1 stretches the release (--slow).
    Returns (samples, windup_pose, follow_pose, info).
    """
    dt = 1.0 / rate_hz
    start = dict(start, gripper=gripper_closed)
    out = [Sample(0.0, "approach", dict(current))]
    t0 = 0.0

    def add_min_jerk(a, b, T, phase):
        nonlocal t0
        n = max(1, math.ceil(T * rate_hz))
        for i in range(1, n + 1):
            u = min_jerk(i / n)
            out.append(Sample(t0 + T * i / n, phase, {j: a[j] + (b[j] - a[j]) * u for j in JOINTS}))
        t0 += T

    def add_ramps(a, moves, speeds, ramps, phase, scale=1.0, gripper=None):
        """Every joint runs its own ramp from pose `a`, all starting together."""
        nonlocal t0
        prof = {j: ramp(moves[j], speeds[j], *ramps) for j in moves}
        T = max(pr[0] for pr in prof.values())
        if gripper:
            T = max(T, gripper["t_open"] + gripper["dur"])
        n = max(1, math.ceil(T * scale * rate_hz))
        for i in range(1, n + 1):
            t = T * i / n
            q = dict(a)
            for j, (_, _, pos) in prof.items():
                q[j] = a[j] + pos(t)
            if gripper and t >= gripper["t_open"]:
                q["gripper"] = a["gripper"] + gripper["move"] * min(1.0, (t - gripper["t_open"]) / max(gripper["dur"], 1e-9))
            out.append(Sample(t0 + T * scale * i / n, phase, q))
        t0 += T * scale
        return prof, T, dict(out[-1].q)

    add_min_jerk(current, start, slow_move(current, start, slow_vel, 2.0), "approach")
    _, T_w, windup = add_ramps(start, p.windup_move(), p.windup_speed(), windup_ramp_s, "windup")

    # release timing from the joint that brakes last
    prof = {j: ramp(p.release_move()[j], p.release_speed()[j], *release_ramp_s) for j in SWING_JOINTS}
    t_brake = max(pr[1] for pr in prof.values())
    t_open = release_time(t_brake, release_lead, release_min_delay)
    g_move = p.gripper_move * gripper_pct_per_deg
    g = {"t_open": t_open, "move": g_move, "dur": p.gripper_move / p.gripper_release_speed}
    _, T_r, follow = add_ramps(windup, p.release_move(), p.release_speed(), release_ramp_s,
                               "swing", release_time_scale, g)
    out.append(Sample(t0 + settle_time, "settle", dict(follow)))   # hold: ball flies off
    t0 += settle_time
    add_min_jerk(follow, start, max(return_time, slow_move(follow, start, slow_vel, 0.0)), "return")
    info = {"windup_s": T_w, "release_s": T_r * release_time_scale, "brake_at_s": t_brake * release_time_scale,
            "open_at_s": t_open * release_time_scale, "open_pct": g_move}
    return out, windup, follow, info


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
