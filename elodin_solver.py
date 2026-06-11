"""Elodin adapter for our sim-agnostic planner.

Run from the elodin repo root:

    PYTHONPATH=/path/to/Drone_Grand_Prix RACE_SOLVER=elodin_solver \
        elodin run sim/main.py

Maps planner efforts → Betaflight RC PWM. Scaling and signs reproduce the
proven solver.baseline numbers exactly:
    tilt window: ±50 PWM around 1500 (baseline clamped pitch/roll to [1450, 1550])
    throttle:    hover 1135, climb authority +465 (clamp ceiling 1600)
    pitch = 1500 + 50·tilt_x   (drone faces +X with yaw untouched, so
    roll  = 1500 − 50·tilt_y    world x→pitch, y→roll map directly)

Gate list is imported from the sim's own course definition — no hardcoding.
The Anduril adapter will instead feed gates from MAVLink track data into the
same planner.
"""

from solver.api import RCCommand, SensorUpdate  # elodin repo package
from sim.course import active_course            # elodin repo package

from planner import PlannerConfig, PlannerState, plan

_COURSE, _SPAWN, _SIM_TIME = active_course()

# Arm/land phase boundaries (sim seconds), same dance as solver.baseline:
# Betaflight needs to see the arm switch low, then a clean low→high transition.
T_DISARMED_END = 0.50
T_ARM_IDLE_END = 0.75
T_LAND_END = _SIM_TIME - 1.0

# PWM mapping. Hover point measured in ANGLE mode (run db009: equilibrium
# climb 0.2 m/s at 1265 PWM → hover ≈ 1255). The baseline's 1135 was an
# artifact of flying acro+airmode.
PWM_CENTER = 1500
TILT_PWM_RANGE = 50          # ±PWM around center at |tilt| = 1
HOVER_PWM = 1255
CLIMB_PWM_AUTHORITY = 345    # PWM above hover at vertical = +1 (ceiling 1600)
THROTTLE_MIN = 1000
THROTTLE_MAX = 1600

GATES = tuple(g.center for g in _COURSE)  # ENU (x, y, z-up) == planner frame

_cfg = PlannerConfig()
_state = PlannerState()


def reset_state() -> None:
    """Reset between runs (mirrors baseline contract, used by tests)."""
    global _state
    _state = PlannerState()


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def autopilot(update: SensorUpdate) -> RCCommand:
    t = update.t

    pos = (float(update.world_pos[4]), float(update.world_pos[5]), float(update.world_pos[6]))
    vel = (
        float(update.world_vel[3]) if update.world_vel.size > 3 else 0.0,
        float(update.world_vel[4]) if update.world_vel.size > 4 else 0.0,
        float(update.world_vel[5]) if update.world_vel.size > 5 else 0.0,
    )

    # Run the planner every tick — even before arming — so it captures the
    # true spawn altitude at t=0. Its output is discarded until flight.
    out = plan(
        _state,
        _cfg,
        t,
        pos,
        vel,
        GATES,
        int(update.next_gate_index),
        integrate_alt=update.baro_fresh,
    )

    # aux2=1800 engages ANGLE mode (configure_betaflight.py maps it to
    # 1700-2100). Without it Betaflight flies acro: stick deflection becomes a
    # rotation RATE, not an angle, and airmode holds hover thrust during the
    # resulting stabilization fight — the drone can't descend. The baseline
    # solver has this same latent bug; its straight, level course hides it.
    if t < T_DISARMED_END:
        return RCCommand(arm=1000, throttle=1000, aux2=1800)
    if t < T_ARM_IDLE_END:
        return RCCommand(arm=1800, throttle=1000, aux2=1800)
    if t >= T_LAND_END:
        return RCCommand(arm=1000, throttle=1000, aux2=1800)

    throttle = int(round(_clamp(
        HOVER_PWM + out.vertical * CLIMB_PWM_AUTHORITY, THROTTLE_MIN, THROTTLE_MAX
    )))
    pitch = int(round(_clamp(
        PWM_CENTER + out.tilt_x * TILT_PWM_RANGE,
        PWM_CENTER - TILT_PWM_RANGE,
        PWM_CENTER + TILT_PWM_RANGE,
    )))
    roll = int(round(_clamp(
        PWM_CENTER - out.tilt_y * TILT_PWM_RANGE,
        PWM_CENTER - TILT_PWM_RANGE,
        PWM_CENTER + TILT_PWM_RANGE,
    )))

    return RCCommand(arm=1800, throttle=throttle, pitch=pitch, roll=roll, aux2=1800)
