"""Stage 1 validation: score the IMU estimator against Elodin ground truth.

Control is delegated to the proven elodin_solver (which flies on ground-truth
state), while the estimator runs PASSIVELY on gyro+accel only. Every tick we
compare its attitude against Elodin's true quaternion. A full VQ1-replica lap
exercises it through real dynamics — climbs, descents, hard corners.

    tools/run_elodin.sh vq1 /tmp/est_check.log 150 elodin_estimator_check

Reported per interval and cumulatively:
    tilt   — angle between true and estimated "up" in body frame. This is
             what stabilization depends on, and it is what the accelerometer
             actually observes. Must stay small.
    total  — full rotation error including heading. Grows with yaw drift,
             which is expected and unobservable without a heading reference
             (VQ2 gets that from vision instead).
"""

import math

from solver.api import RCCommand, SensorUpdate  # elodin repo package

import elodin_solver
from elodin_solver import DIRECT_MOTORS  # same array object; mutated in place
from attitude import angle_between, quat_rotate_inv
from estimator import EstimatorConfig, EstimatorState, up_in_body, update

# Elodin: FLU body, ENU world, +z up.
UP_ENU = (0.0, 0.0, 1.0)
_cfg = EstimatorConfig(up_world=UP_ENU)
_state = EstimatorState()

REPORT_PERIOD_S = 3.0
_stats = {"n": 0, "tilt_sum": 0.0, "tilt_max": 0.0,
          "total_sum": 0.0, "total_max": 0.0, "next_report": REPORT_PERIOD_S}


def reset_state() -> None:
    elodin_solver.reset_state()


def _true_quat(update_obj):
    """Elodin world_pos is [qx, qy, qz, qw]; we want [w, x, y, z]."""
    qx, qy, qz, qw = (float(update_obj.world_pos[i]) for i in range(4))
    return [qw, qx, qy, qz]


def _quat_angle_between(qa, qb) -> float:
    """Total rotation angle between two orientations, radians."""
    dot = abs(sum(qa[i] * qb[i] for i in range(4)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def autopilot(update_obj: SensorUpdate) -> RCCommand:
    # ── Estimator runs on IMU only, exactly as VQ2 will constrain it ──
    update(_state, _cfg, update_obj.t,
           [float(v) for v in update_obj.gyro],
           [float(v) for v in update_obj.accel])

    if _state.initialized:
        q_true = _true_quat(update_obj)
        tilt = math.degrees(angle_between(
            quat_rotate_inv(q_true, UP_ENU), up_in_body(_state, _cfg)))
        total = math.degrees(_quat_angle_between(q_true, _state.q))

        _stats["n"] += 1
        _stats["tilt_sum"] += tilt
        _stats["tilt_max"] = max(_stats["tilt_max"], tilt)
        _stats["total_sum"] += total
        _stats["total_max"] = max(_stats["total_max"], total)

        if update_obj.t >= _stats["next_report"]:
            _stats["next_report"] += REPORT_PERIOD_S
            n = _stats["n"]
            print(
                f"  [EST] t={update_obj.t:5.1f}  tilt now={tilt:5.2f} "
                f"mean={_stats['tilt_sum'] / n:5.2f} max={_stats['tilt_max']:5.2f} deg | "
                f"total mean={_stats['total_sum'] / n:6.2f} max={_stats['total_max']:6.2f} deg | "
                f"bias=({_state.bias[0]:+.3f},{_state.bias[1]:+.3f},{_state.bias[2]:+.3f}) rad/s",
                flush=True)

    # ── Control unchanged: the proven ground-truth solver flies the lap ──
    return elodin_solver.autopilot(update_obj)
