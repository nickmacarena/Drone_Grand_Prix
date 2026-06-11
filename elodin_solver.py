"""Elodin adapter: planner → our own flight stack → direct motor commands.

Bypasses Betaflight entirely (the harness's BF bridge is broken — see
docs/JOURNAL.md 2026-06-11). The patched sim/main.py reads DIRECT_MOTORS
from this module each tick and feeds it straight to the physics.

Run from the elodin repo root:

    PYTHONPATH=/path/to/Drone_Grand_Prix RACE_SOLVER=elodin_solver \
        elodin run sim/main.py        # or tools/run_elodin.sh <course>
"""

import numpy as np

from solver.api import RCCommand, SensorUpdate  # elodin repo package
from sim.course import active_course            # elodin repo package

from flight_stack import HOVER_CMD, motor_commands
from planner import PlannerConfig, PlannerState, plan

_COURSE, _SPAWN, _SIM_TIME = active_course()
GATES = tuple(g.center for g in _COURSE)  # ENU (x, y, z-up) == planner frame

T_LAND_END = _SIM_TIME - 1.0

# Vertical effort → collective: ±VERT_AUTH around hover at |vertical| = 1.
# 0.15 → ±8.4 N → ±13 m/s² authority. Planner gains below assume this.
VERT_AUTH = 0.15
THRUST_MIN_CMD = 0.02
THRUST_MAX_CMD = 0.95

# Planner gains for the real plant:
#   horizontal: max tilt 25° → ~4.6 m/s² authority; ωn≈1.2, ζ≈1
#   vertical:   ±13 m/s² authority; ωn≈1.5, ζ≈1
_cfg = PlannerConfig(
    kp_x=0.31, kd_x=0.52,
    kp_y=0.31, kd_y=0.52,
    kp_alt=0.17, kd_alt=0.23, ki_alt=0.02, i_clamp=0.30,
    takeoff_clear_m=1.0,
    takeoff_goal_band_m=1.0,
    takeoff_climb_rate=1.0,
    takeoff_vz_gain=0.35,
)
_state = PlannerState()

# Read by the patched sim/main.py every tick. First element < 0 = inactive.
DIRECT_MOTORS = np.array([-1.0, -1.0, -1.0, -1.0])


def reset_state() -> None:
    global _state
    _state = PlannerState()
    DIRECT_MOTORS[:] = -1.0


def autopilot(update: SensorUpdate) -> RCCommand:
    t = update.t

    pos = (float(update.world_pos[4]), float(update.world_pos[5]), float(update.world_pos[6]))
    vel = (
        float(update.world_vel[3]) if update.world_vel.size > 3 else 0.0,
        float(update.world_vel[4]) if update.world_vel.size > 4 else 0.0,
        float(update.world_vel[5]) if update.world_vel.size > 5 else 0.0,
    )

    out = plan(
        _state, _cfg, t, pos, vel, GATES,
        int(update.next_gate_index),
        integrate_alt=update.baro_fresh,
    )

    if t >= T_LAND_END:
        DIRECT_MOTORS[:] = 0.0
        return RCCommand(arm=1000, throttle=1000)

    thrust_cmd = HOVER_CMD + out.vertical * VERT_AUTH
    thrust_cmd = max(THRUST_MIN_CMD, min(THRUST_MAX_CMD, thrust_cmd))

    quat = tuple(float(update.world_pos[i]) for i in range(4))
    motors = motor_commands(
        quat,
        update.gyro,
        out.tilt_x,
        out.tilt_y,
        thrust_cmd,
    )
    DIRECT_MOTORS[:] = motors

    # Betaflight is bypassed; keep it disarmed and idle.
    return RCCommand(arm=1000, throttle=1000)
