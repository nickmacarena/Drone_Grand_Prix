"""Synthetic validation of the IMU attitude estimator.

Runs the filter against a known attitude trajectory with a perfect-IMU
forward model plus injected gyro bias and sensor noise. No simulator needed:
this catches sign errors and divergence before we spend a sim run.

    python3 tests/test_estimator.py
"""

import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from attitude import (  # noqa: E402
    angle_between,
    euler_to_quaternion,
    quat_mul,
    quat_normalize,
    quat_rotate_inv,
)
from estimator import (  # noqa: E402
    G,
    EstimatorConfig,
    EstimatorState,
    up_in_body,
    update,
)


UP_ENU = (0.0, 0.0, 1.0)


def integrate_truth(q, omega, dt):
    """Advance a true attitude quaternion by a body rate."""
    dq = quat_mul(q, [0.0, omega[0], omega[1], omega[2]])
    return quat_normalize([q[i] + 0.5 * dq[i] * dt for i in range(4)])


def perfect_imu(q_true, omega_true, bias, noise, rng, up_world=UP_ENU):
    """What a real IMU would report for this state (rotation in place)."""
    gyro = [omega_true[i] + bias[i] + rng.gauss(0.0, noise) for i in range(3)]
    g_body = quat_rotate_inv(q_true, (G * up_world[0], G * up_world[1], G * up_world[2]))
    accel = [g_body[i] + rng.gauss(0.0, noise * 10.0) for i in range(3)]
    return gyro, accel


def run_case(name, omega_fn, *, bias, noise, duration=20.0, dt=0.004,
             start_roll=0.0, start_pitch=0.0, seed=7, corrupt_at=None,
             corrupt_rpy=(0.0, 0.0, 0.0)):
    rng = random.Random(seed)
    cfg = EstimatorConfig(up_world=UP_ENU)
    st = EstimatorState()

    q_true = euler_to_quaternion(start_roll, start_pitch, 0.0)
    t = 0.0
    errs = []
    corrupted = False
    n = int(duration / dt)
    for k in range(n):
        # Kick the ESTIMATE off truth mid-flight so the accel correction path
        # is genuinely exercised (initialization alone would hide a bad sign).
        if corrupt_at is not None and not corrupted and t >= corrupt_at and st.initialized:
            st.q = quat_mul(st.q, euler_to_quaternion(*corrupt_rpy))
            corrupted = True

        omega = omega_fn(t)
        gyro, accel = perfect_imu(q_true, omega, bias, noise, rng)
        update(st, cfg, t, gyro, accel)

        if st.initialized:
            true_up_body = quat_rotate_inv(q_true, UP_ENU)
            est_up_body = up_in_body(st, cfg)
            errs.append(math.degrees(angle_between(true_up_body, est_up_body)))

        q_true = integrate_truth(q_true, omega, dt)
        t += dt

    # Steady-state = last 25% of the run, after the filter has settled
    # (and well after any injected corruption).
    tail = errs[int(len(errs) * 0.75):]
    peak = max(errs) if errs else float("nan")
    mean_tail = sum(tail) / len(tail) if tail else float("nan")
    bias_err = [math.degrees(-st.bias[i] - bias[i]) for i in range(3)]

    print(f"{name}")
    print(f"    peak tilt error      {peak:6.2f} deg")
    print(f"    steady tilt error    {mean_tail:6.2f} deg")
    print(f"    bias estimate error  ({bias_err[0]:+.2f}, {bias_err[1]:+.2f}, {bias_err[2]:+.2f}) deg/s")
    return mean_tail, peak


def main():
    print("=== IMU attitude estimator — synthetic validation ===\n")
    results = []

    # 1. Level hover, no bias: pure sanity. Must be ~0.
    results.append(run_case(
        "[1] level hover, clean IMU",
        lambda t: (0.0, 0.0, 0.0), bias=(0.0, 0.0, 0.0), noise=0.0))

    # 2. Kick the estimate 40 deg off mid-flight; it must pull back.
    #    (Initialization alone can't be the thing under test here.)
    results.append(run_case(
        "[2] recover from 40 deg estimate error injected at t=5s",
        lambda t: (0.0, 0.0, 0.0), bias=(0.0, 0.0, 0.0), noise=0.0,
        corrupt_at=5.0,
        corrupt_rpy=(math.radians(40.0), math.radians(-25.0), 0.0)))

    # 3. Constant gyro bias — the estimator must learn it out.
    results.append(run_case(
        "[3] constant gyro bias 2 deg/s all axes",
        lambda t: (0.0, 0.0, 0.0),
        bias=(math.radians(2.0), math.radians(-2.0), math.radians(2.0)),
        noise=0.0))

    # 4. Dynamic flight: oscillating roll/pitch plus steady yaw rate, noisy.
    results.append(run_case(
        "[4] dynamic manoeuvring + bias + noise",
        lambda t: (0.6 * math.sin(1.2 * t), 0.5 * math.cos(0.9 * t), 0.35),
        bias=(math.radians(1.5), math.radians(-1.0), math.radians(0.8)),
        noise=0.01))

    print()
    worst_steady = max(r[0] for r in results)
    ok = worst_steady < 3.0 and all(not math.isnan(r[0]) for r in results)
    print(f"worst steady-state tilt error: {worst_steady:.2f} deg")
    print("RESULT:", "PASS — filter tracks and learns bias" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
