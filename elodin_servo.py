"""Stage 3a: fly the course on bearings alone — the VQ2 control architecture.

    tools/run_elodin.sh vq1 /tmp/servo.log 200 elodin_servo

What the control path is allowed to touch, matching what VQ2 actually gives:
    IMU gyro + accel   -> estimator -> attitude          (no ATTITUDE telemetry)
    gate bearing       -> servo     -> steering          (no track data)
    next_gate_index    -> which gate                     (RACE_STATUS survives)
NOT used: position, velocity, or gate coordinates in any control decision.

Ground truth is used for exactly one thing — standing in for the camera, by
projecting the active gate into the lens (`bearing.observe_from_truth`). The
controller only ever sees the resulting az/el/confidence, so swapping in a
real image detector later changes nothing here. Elodin's own camera has never
produced a frame (see JOURNAL), which is precisely why this split exists.

DETECTOR_NOISE_DEG / DETECTOR_DROP env vars degrade that stand-in, so we can
find out how good the real detector actually has to be.
"""

import math
import os
import random

import numpy as np

from solver.api import RCCommand, SensorUpdate  # elodin repo package
from sim.course import active_course            # elodin repo package

import estimator
import servo
from attitude import quat_rotate
from bearing import ELODIN_CAM, observe_from_truth
from flight_stack import HOVER_CMD, motor_commands

_COURSE, _SPAWN, _SIM_TIME = active_course()
GATES = [tuple(g.center) for g in _COURSE]
T_LAND_END = _SIM_TIME - 1.0

VERT_AUTH = 0.15
THRUST_MIN_CMD, THRUST_MAX_CMD = 0.02, 0.95

# Detector realism knobs (defaults = perfect detector).
NOISE_DEG = float(os.environ.get("DETECTOR_NOISE_DEG", "0"))
DROP_RATE = float(os.environ.get("DETECTOR_DROP", "0"))
DETECT_HZ = float(os.environ.get("DETECTOR_HZ", "30"))

_est_cfg = estimator.EstimatorConfig(up_world=(0.0, 0.0, 1.0))
_est = estimator.EstimatorState()
_servo_cfg = servo.ServoConfig()
_servo_state = servo.ServoState()
_rng = random.Random(12345)

DIRECT_MOTORS = np.array([-1.0, -1.0, -1.0, -1.0])

_last_detect_t = [-1e9]
_last_obs = [None]
_log = {"next": 0.0, "seen": 0, "total": 0}


def reset_state() -> None:
    global _est, _servo_state
    _est = estimator.EstimatorState()
    _servo_state = servo.ServoState()
    DIRECT_MOTORS[:] = -1.0


def _detect(update: SensorUpdate):
    """Stand-in for the camera: project the active gate, optionally degraded."""
    idx = int(update.next_gate_index)
    if not (0 <= idx < len(GATES)):
        return None

    q_true = [float(update.world_pos[3]), float(update.world_pos[0]),
              float(update.world_pos[1]), float(update.world_pos[2])]
    pos = (float(update.world_pos[4]), float(update.world_pos[5]),
           float(update.world_pos[6]))

    obs = observe_from_truth(ELODIN_CAM, q_true, pos, GATES[idx])
    if obs is None:
        return None
    if DROP_RATE > 0.0 and _rng.random() < DROP_RATE:
        return None
    if NOISE_DEG > 0.0:
        s = math.radians(NOISE_DEG)
        obs = type(obs)(az=obs.az + _rng.gauss(0, s), el=obs.el + _rng.gauss(0, s),
                        u=obs.u, v=obs.v, width_px=obs.width_px,
                        confidence=obs.confidence)
    return obs


def autopilot(update: SensorUpdate) -> RCCommand:
    t = update.t

    # ── Attitude from the IMU alone ─────────────────────────────────────
    estimator.update(_est, _est_cfg, t,
                     [float(v) for v in update.gyro],
                     [float(v) for v in update.accel])

    if t >= T_LAND_END:
        DIRECT_MOTORS[:] = 0.0
        return RCCommand(arm=1000, throttle=1000)
    if not _est.initialized:
        DIRECT_MOTORS[:] = HOVER_CMD
        return RCCommand(arm=1000, throttle=1000)

    # ── "Camera" at its own rate, held between frames like a real one ───
    if t - _last_detect_t[0] >= 1.0 / DETECT_HZ:
        _last_detect_t[0] = t
        _last_obs[0] = _detect(update)
        _log["total"] += 1
        if _last_obs[0] is not None:
            _log["seen"] += 1
    obs = _last_obs[0]

    out = servo.step(_servo_state, _servo_cfg, t, obs)

    # ── Body-relative steering -> world tilt, using the ESTIMATED heading.
    # Both this and motor_commands() use the same estimate, so the estimator's
    # yaw drift cancels: body-relative control is drift-immune by construction.
    qw, qx, qy, qz = _est.q
    fwd_w = quat_rotate(_est.q, (1.0, 0.0, 0.0))
    hx, hy = fwd_w[0], fwd_w[1]
    n = math.hypot(hx, hy)
    if n < 1e-6:
        hx, hy, n = 1.0, 0.0, 1.0
    hx, hy = hx / n, hy / n
    rx, ry = hy, -hx                      # right of heading in ENU (up = +z)

    tilt_x = out.tilt_fwd * hx + out.tilt_right * rx
    tilt_y = out.tilt_fwd * hy + out.tilt_right * ry

    thrust = HOVER_CMD + out.vertical * VERT_AUTH
    thrust = max(THRUST_MIN_CMD, min(THRUST_MAX_CMD, thrust))

    DIRECT_MOTORS[:] = motor_commands(
        (qx, qy, qz, qw), update.gyro, tilt_x, tilt_y, thrust,
        yaw_rate_cmd=out.yaw_rate,
    )

    if t >= _log["next"]:
        _log["next"] += 2.0
        pos = (float(update.world_pos[4]), float(update.world_pos[5]),
               float(update.world_pos[6]))
        if obs is not None:
            det = (f"az={math.degrees(obs.az):+6.1f} el={math.degrees(obs.el):+6.1f} "
                   f"rng={obs.range_m:5.1f} conf={obs.confidence:.2f}")
        else:
            det = f"NO GATE ({_servo_state.time_since_seen:.1f}s)"
        print(f"  [SERVO] t={t:5.1f} gate={update.next_gate_index} "
              f"pos=({pos[0]:6.1f},{pos[1]:5.1f},{pos[2]:5.1f}) {det} "
              f"| yaw_rate={out.yaw_rate:+.2f} fwd={out.tilt_fwd:.2f} vert={out.vertical:+.2f}",
              flush=True)

    return RCCommand(arm=1000, throttle=1000)
