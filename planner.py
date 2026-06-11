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

    # Takeoff: below this altitude, climb at fixed effort and don't translate
    min_translation_alt_m: float = 1.0
    takeoff_effort: float = 0.355
    takeoff_climb_rate_max: float = 0.7  # m/s; above this, hand over to PD


@dataclass
class PlannerState:
    """Mutable controller state. One instance per flight; reset between runs."""
    i_term: float = 0.0
    last_t: float = 0.0


@dataclass(frozen=True)
class PlannerOutput:
    tilt_x: float
    tilt_y: float
    vertical: float


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def select_goal(gates: Sequence[Vec3], next_gate_index: int) -> Vec3:
    """Pick the gate to fly toward. Out-of-range index → last gate."""
    if not gates:
        return (0.0, 0.0, 0.0)
    if 0 <= next_gate_index < len(gates):
        return gates[next_gate_index]
    return gates[-1]


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
    goal = select_goal(gates, next_gate_index)

    x, y, alt = pos
    vx, vy, v_alt = vel
    dt = max(1e-3, t - state.last_t)
    state.last_t = t

    # ── Vertical ─────────────────────────────────────────────────────
    alt_err = goal[2] - alt
    if integrate_alt:
        state.i_term = _clamp(
            state.i_term + alt_err * dt * cfg.ki_alt,
            -cfg.i_clamp,
            cfg.i_clamp,
        )

    if alt < cfg.min_translation_alt_m and v_alt < cfg.takeoff_climb_rate_max:
        vertical = cfg.takeoff_effort
    else:
        vertical = cfg.kp_alt * alt_err - cfg.kd_alt * v_alt + state.i_term
    vertical = _clamp(vertical, -1.0, 1.0)

    # ── Horizontal (gated until at flying altitude) ──────────────────
    tilt_x = 0.0
    tilt_y = 0.0
    if alt >= cfg.min_translation_alt_m:
        tilt_x = _clamp(cfg.kp_x * (goal[0] - x) - cfg.kd_x * vx, -1.0, 1.0)
        tilt_y = _clamp(cfg.kp_y * (goal[1] - y) - cfg.kd_y * vy, -1.0, 1.0)

    return PlannerOutput(tilt_x=tilt_x, tilt_y=tilt_y, vertical=vertical)
