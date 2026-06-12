"""Sim-agnostic race planner.

One planner, multiple sims. The planner works in an abstract world frame:
    x, y  : horizontal axes (any consistent world frame)
    alt   : altitude, positive UP
and outputs normalized efforts that a per-sim adapter maps to actual
commands (RC PWM for Elodin/Betaflight, quaternion+thrust for the AIGP sim).

    PlannerOutput.tilt_x   ∈ [-1, 1]  desired tilt toward +x (1 = max tilt)
    PlannerOutput.tilt_y   ∈ [-1, 1]  desired tilt toward +y
    PlannerOutput.vertical ∈ [-1, 1]  vertical effort around hover
                                      (0 = hover, 1 = max climb authority)

Adapters own: frame conversion (e.g. NED → x,y,alt), yaw handling, arming,
race-start gating, and scaling efforts to their sim's command units.

The control law mirrors the Elodin baseline solver (proven 3/3 gates):
per-axis PD on position toward the active gate, altitude PD+I on thrust,
horizontal translation gated until the drone is at flying altitude.
"""

import math
from dataclasses import dataclass, field
from typing import Sequence, Tuple


Vec3 = Tuple[float, float, float]  # (x, y, alt)


@dataclass(frozen=True)
class PlannerConfig:
    # Horizontal PD, per axis (normalized tilt per m / per m/s)
    kp_x: float = 1.4
    kd_x: float = 0.6
    kp_y: float = 0.7
    kd_y: float = 1.6

    # Altitude PD+I (normalized vertical effort per m / per m/s)
    kp_alt: float = 0.301
    kd_alt: float = 0.097
    ki_alt: float = 0.0172
    i_clamp: float = 0.172

    # Takeoff: climb (no translation) until clear of the spawn point or near
    # goal altitude. Spawn-relative, so it works whether the course sits
    # above the spawn (Elodin easy), descends below it (VQ1), or starts at
    # spawn height (official sim platform).
    takeoff_clear_m: float = 1.0        # alt above spawn that ends takeoff
    takeoff_goal_band_m: float = 1.0    # ...or climbing to within this of goal alt
    takeoff_climb_rate: float = 1.0     # m/s climb target during takeoff
    takeoff_vz_gain: float = 0.35       # effort per m/s of climb-rate error

    # Aim this far PAST the active gate along the course direction, so the
    # drone flies through the plane instead of parking at the center.
    # The sim advances the gate index at the crossing.
    pass_through_m: float = 2.5


@dataclass
class PlannerState:
    """Mutable controller state. One instance per flight; reset between runs."""
    i_term: float = 0.0
    last_t: float = 0.0
    spawn: Vec3 | None = None       # captured on first plan() call
    airborne: bool = False          # latched once takeoff completes
    retry_latch: bool = False       # committed to the approach point
    retry_idx: int = -999           # gate index the latch belongs to


@dataclass(frozen=True)
class PlannerOutput:
    tilt_x: float
    tilt_y: float
    vertical: float


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def select_goal(
    gates: Sequence[Vec3],
    next_gate_index: int,
    spawn: Vec3,
    pass_through_m: float,
    pos: Vec3 | None = None,
    retry_behind_m: float = 4.0,
    state: "PlannerState | None" = None,
) -> Vec3:
    """Aim point: active gate center pushed past the gate plane.

    The through-direction is the unit vector from the previous gate (or the
    spawn point, for gate 0) to the active gate. Aiming past the plane keeps
    speed up through the crossing; the sim advances the index at the plane.
    Out-of-range index → last gate (race finished / pre-start default).

    Gate-miss recovery: if `pos` is already PAST the gate plane but the index
    hasn't advanced (we crossed outside the inner square), aim at a point
    upstream of the gate so the drone loops back and re-attempts the pass.
    """
    if not gates:
        return (0.0, 0.0, 0.0)
    idx = next_gate_index if 0 <= next_gate_index < len(gates) else len(gates) - 1
    gate = gates[idx]
    prev = gates[idx - 1] if idx >= 1 else spawn
    # Horizontal-only offset: altitude target stays at the gate center, so a
    # climbing/descending course still crosses the plane inside the inner
    # square (a sloped offset shifted the crossing altitude out of the box).
    dx = gate[0] - prev[0]
    dy = gate[1] - prev[1]
    norm = math.sqrt(dx * dx + dy * dy)
    if norm < 1e-6:
        return gate
    ux, uy = dx / norm, dy / norm

    if pos is not None and 0 <= next_gate_index < len(gates):
        # Position relative to the gate, decomposed along/across its axis.
        rx = pos[0] - gate[0]
        ry = pos[1] - gate[1]
        past = rx * ux + ry * uy            # signed distance past the plane
        lateral = abs(-rx * uy + ry * ux)   # distance off the gate axis

        # Missed the plane, or off-axis near the gate → retreat to the
        # approach point upstream ON the axis. The decision is LATCHED
        # (hysteresis): flickering between aim and retry at the threshold
        # caused ±20° command chatter while hovering at the plane (run 14).
        trigger = past > pass_through_m * 0.6 or (lateral > 1.2 and past > -retry_behind_m * 0.75)
        if state is not None:
            if state.retry_idx != next_gate_index:
                state.retry_latch = False
                state.retry_idx = next_gate_index
            if not state.retry_latch and trigger:
                state.retry_latch = True
            elif state.retry_latch and past < -retry_behind_m * 0.5 and lateral <= 1.0:
                state.retry_latch = False
            retreat = state.retry_latch
        else:
            retreat = trigger
        if retreat:
            return (gate[0] - ux * retry_behind_m, gate[1] - uy * retry_behind_m, gate[2])

    return (gate[0] + ux * pass_through_m, gate[1] + uy * pass_through_m, gate[2])


def plan(
    state: PlannerState,
    cfg: PlannerConfig,
    t: float,
    pos: Vec3,
    vel: Vec3,
    gates: Sequence[Vec3],
    next_gate_index: int,
    integrate_alt: bool = True,
) -> PlannerOutput:
    """One planning step. Returns normalized efforts (see module docstring)."""
    x, y, alt = pos
    vx, vy, v_alt = vel
    dt = max(1e-3, t - state.last_t)
    state.last_t = t

    if state.spawn is None:
        state.spawn = (x, y, alt)

    goal = select_goal(gates, next_gate_index, state.spawn, cfg.pass_through_m, pos=pos, state=state)

    # Takeoff completes (and latches) once we've climbed clear of the spawn
    # or risen to within a band of the goal altitude. The latch matters on
    # descending courses: later legs drop below spawn altitude, and takeoff
    # must not re-engage there.
    if not state.airborne:
        cleared_spawn = alt > state.spawn[2] + cfg.takeoff_clear_m
        near_goal = abs(goal[2] - alt) < cfg.takeoff_goal_band_m and v_alt > 0.0
        if cleared_spawn or near_goal:
            state.airborne = True

    # ── Vertical ─────────────────────────────────────────────────────
    alt_err = goal[2] - alt
    if integrate_alt:
        state.i_term = _clamp(
            state.i_term + alt_err * dt * cfg.ki_alt,
            -cfg.i_clamp,
            cfg.i_clamp,
        )

    if not state.airborne:
        # Climb-rate controller: ~0.35 effort on a pad (matches the proven
        # baseline takeoff), saturates to recover from an initial descent.
        vertical = cfg.takeoff_vz_gain * (cfg.takeoff_climb_rate - v_alt)
    else:
        vertical = cfg.kp_alt * alt_err - cfg.kd_alt * v_alt + state.i_term
    vertical = _clamp(vertical, -1.0, 1.0)

    # ── Horizontal (gated until airborne) ────────────────────────────
    tilt_x = 0.0
    tilt_y = 0.0
    if state.airborne:
        tilt_x = _clamp(cfg.kp_x * (goal[0] - x) - cfg.kd_x * vx, -1.0, 1.0)
        tilt_y = _clamp(cfg.kp_y * (goal[1] - y) - cfg.kd_y * vy, -1.0, 1.0)

    return PlannerOutput(tilt_x=tilt_x, tilt_y=tilt_y, vertical=vertical)
