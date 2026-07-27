"""IMU attitude estimator (AHRS) — sim-agnostic, pure Python.

VQ2 (sim v1.0.3391 / example v4) disables ATTITUDE, LOCAL_POSITION_NED and
ODOMETRY telemetry and nulls track data. HIGHRES_IMU is the only state sensor
left, so orientation has to be reconstructed from gyro + accelerometer.

Mahony-style complementary filter:
    - gyro integration carries the high-frequency attitude,
    - the accelerometer's gravity direction corrects low-frequency drift,
    - the correction error also drives a gyro-bias estimate.

Pure Python (math only) on purpose: the Windows ARM VM has no numpy for
Python 3.14 (JOURNAL 2026-06-06), and this has to run there.

Frames are parameterized so one filter serves both sims:
    Elodin — FLU body / ENU world, up_world = (0, 0, +1)
    AIGP   — FRD body / NED world, up_world = (0, 0, -1)

Quaternions are [w, x, y, z], rotating BODY vectors into WORLD.

YAW: with no magnetometer or heading reference, yaw drifts — unbounded in
principle, slowly in practice once bias is learned. That is acceptable for
VQ2 by design: control is visual servoing (fly toward the gate you SEE, a
body-relative bearing), so absolute heading is never needed. Roll/pitch —
which the accelerometer does observe — is what stabilization depends on.
"""

import math
from dataclasses import dataclass, field

from attitude import (
    quat_from_two_vectors,
    quat_mul,
    quat_normalize,
    quat_rotate_inv,
    vec_cross,
    vec_norm,
    vec_unit,
)


G = 9.80665


@dataclass(frozen=True)
class EstimatorConfig:
    # Accelerometer correction gain (rad/s of correction per rad of tilt error).
    # ~1.0 gives a few-second time constant: fast enough to kill drift, slow
    # enough to ignore manoeuvre accelerations.
    kp: float = 1.0
    # Gyro-bias learning rate. Deliberately slow; bias is near-constant.
    ki: float = 0.05
    # Only trust the accelerometer as a gravity reference when its magnitude
    # is close to 1 g. Under acceleration it measures thrust too, not gravity.
    accel_trust_band: float = 0.20      # fraction of G
    # World "up" direction. ENU: (0,0,1). NED: (0,0,-1).
    up_world: tuple = (0.0, 0.0, 1.0)
    max_dt: float = 0.1                 # ignore absurd gaps (stalls, warmup)


@dataclass
class EstimatorState:
    q: list = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    bias: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    last_t: float | None = None
    initialized: bool = False
    # Diagnostics
    last_accel_used: bool = False
    last_tilt_err_rad: float = 0.0


def reset(state: EstimatorState) -> None:
    state.q = [1.0, 0.0, 0.0, 0.0]
    state.bias = [0.0, 0.0, 0.0]
    state.last_t = None
    state.initialized = False


def update(state: EstimatorState, cfg: EstimatorConfig, t: float, gyro, accel) -> None:
    """One filter step. `gyro` rad/s and `accel` m/s^2, both body frame."""
    if state.last_t is None:
        state.last_t = t

    dt = t - state.last_t
    state.last_t = t
    if dt <= 0.0 or dt > cfg.max_dt:
        return

    a_norm = vec_norm(accel)
    accel_usable = (
        a_norm > 1e-6
        and abs(a_norm - G) <= cfg.accel_trust_band * G
    )

    # ── Initialize from the first trustworthy accel sample ───────────────
    # Level the estimate against measured gravity; heading starts arbitrary.
    if not state.initialized:
        if not accel_usable:
            return
        # q must rotate the measured up direction (body) onto world up.
        state.q = quat_from_two_vectors(vec_unit(accel), cfg.up_world)
        state.initialized = True
        return

    # ── Accelerometer correction ────────────────────────────────────────
    err = (0.0, 0.0, 0.0)
    if accel_usable:
        # Where the filter currently thinks "up" is, in body coordinates.
        up_body_est = quat_rotate_inv(state.q, cfg.up_world)
        up_body_meas = vec_unit(accel)
        # Order matters: (meas x est), not (est x meas). Rotating the BODY
        # FRAME by +w moves a fixed world vector's body-frame representation
        # the OTHER way, so the intuitive order is positive feedback — it
        # diverged to 180 deg in tests/test_estimator.py before this fix.
        err = vec_cross(up_body_meas, up_body_est)
        state.last_tilt_err_rad = math.asin(min(1.0, vec_norm(err)))
        for i in range(3):
            state.bias[i] += cfg.ki * err[i] * dt
    state.last_accel_used = accel_usable

    # ── Integrate corrected body rates ──────────────────────────────────
    wx = gyro[0] + state.bias[0] + cfg.kp * err[0]
    wy = gyro[1] + state.bias[1] + cfg.kp * err[1]
    wz = gyro[2] + state.bias[2] + cfg.kp * err[2]

    # q_dot = 0.5 * q (x) (0, omega_body)
    dq = quat_mul(state.q, [0.0, wx, wy, wz])
    state.q = quat_normalize([
        state.q[0] + 0.5 * dq[0] * dt,
        state.q[1] + 0.5 * dq[1] * dt,
        state.q[2] + 0.5 * dq[2] * dt,
        state.q[3] + 0.5 * dq[3] * dt,
    ])


def up_in_body(state: EstimatorState, cfg: EstimatorConfig):
    """Estimated world-up direction expressed in the body frame.

    This is the frame-convention-free way to compare two attitude estimates:
    the angle between two such vectors is the tilt error, with no Euler or
    handedness ambiguity and no dependence on (drifting, unobservable) yaw.
    """
    return quat_rotate_inv(state.q, cfg.up_world)
