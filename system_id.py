"""System identification — measure drone parameters before tuning.

Standalone script. Run against the live sim *outside* a race. Sends a
sequence of test inputs and logs the responses. At the end prints the
measured values that should be copied into measurements.py.

Tests (in order):
    1. HOVER THRUST SWEEP — climbs/descends at several thrust values,
       measures terminal vertical velocity. Hover thrust is the value
       where vd ≈ 0.

    2. ATTITUDE STEP RESPONSE — commands target_pitch = 20° from level,
       holds for 2 seconds. Measures rise time (time to reach 18°) and
       overshoot.

    3. THRUST → VERTICAL ACCEL — at HOVER + 0.10, measures vertical
       acceleration during the first 0.5 s before drag dominates.

NOTE: This does *not* start a race. Run it from the lobby (sim connected,
heartbeat received) but before clicking RACE.
"""

import math
import time

from attitude import euler_to_quaternion
from mavlink_tx import send_arm, send_attitude_quaternion
from setup import setup_components


SIM_SERVER_UDP_IP = "0.0.0.0"
SIM_SERVER_UDP_PORT = 14550

LOOP_HZ = 250
LOOP_PERIOD = 1.0 / LOOP_HZ


def sample_state(shared):
    """Return a copy of current drone_state, or None if not yet available."""
    return shared.drone_state


def send_attitude(mavlink_conn, system_boot_ms, roll, pitch, yaw, thrust):
    q = euler_to_quaternion(roll, pitch, yaw)
    send_attitude_quaternion(mavlink_conn, system_boot_ms, q, thrust)


def keep_armed(mavlink_conn, shared, last_arm_t):
    hb = shared.heartbeat
    if hb is None or hb.armed:
        return last_arm_t
    now = time.time()
    if now - last_arm_t < 0.5:
        return last_arm_t
    send_arm(mavlink_conn)
    return now


def run_for(mavlink_conn, shared, system_boot_ms, duration_s, attitude, sample_log=None):
    """Stream the given (roll, pitch, yaw, thrust) for duration_s. Sample at 20 Hz into sample_log."""
    roll, pitch, yaw, thrust = attitude
    start = time.time()
    last_arm_t = 0.0
    sample_period = 0.05
    last_sample = 0.0

    while time.time() - start < duration_s:
        last_arm_t = keep_armed(mavlink_conn, shared, last_arm_t)
        send_attitude(mavlink_conn, system_boot_ms, roll, pitch, yaw, thrust)
        now = time.time()
        if sample_log is not None and now - last_sample >= sample_period:
            ds = sample_state(shared)
            if ds is not None:
                sample_log.append((now - start, ds))
            last_sample = now
        time.sleep(LOOP_PERIOD)


def test_hover_thrust(mavlink_conn, shared, system_boot_ms):
    print("\n=== TEST 1: hover thrust sweep ===", flush=True)
    results = {}
    for thrust in (0.40, 0.45, 0.50, 0.55, 0.60):
        print(f"  thrust={thrust:.2f} for 2.5s ...", flush=True)
        samples = []
        run_for(
            mavlink_conn, shared, system_boot_ms,
            duration_s=2.5,
            attitude=(0.0, 0.0, 0.0, thrust),
            sample_log=samples,
        )
        # Use the last 1 second of vertical velocity samples
        if not samples:
            print("    no samples", flush=True)
            continue
        recent = [ds.vd_mps for t, ds in samples if t >= 1.5]
        if not recent:
            recent = [samples[-1][1].vd_mps]
        mean_vd = sum(recent) / len(recent)
        results[thrust] = mean_vd
        print(f"    terminal vd ≈ {mean_vd:+.2f} m/s", flush=True)

    # Linear-interpolate to find vd = 0
    keys = sorted(results.keys())
    hover = None
    for a, b in zip(keys, keys[1:]):
        if results[a] * results[b] <= 0:  # sign change
            t = results[a] / (results[a] - results[b])
            hover = a + t * (b - a)
            break
    if hover is not None:
        print(f"\n  >>> HOVER_THRUST ≈ {hover:.3f}", flush=True)
    else:
        print("\n  >>> Could not bracket hover thrust. Widen the sweep.", flush=True)
    return hover


def test_attitude_response(mavlink_conn, shared, system_boot_ms):
    print("\n=== TEST 2: attitude step response (pitch 20°) ===", flush=True)
    # Hold level briefly, then command 20° pitch and measure.
    print("  pre-step level for 1s", flush=True)
    run_for(mavlink_conn, shared, system_boot_ms, 1.0, (0.0, 0.0, 0.0, 0.5))

    print("  step to pitch=20° for 2s ...", flush=True)
    samples = []
    target = math.radians(20.0)
    run_for(
        mavlink_conn, shared, system_boot_ms,
        duration_s=2.0,
        attitude=(0.0, target, 0.0, 0.5),
        sample_log=samples,
    )

    if not samples:
        print("  no samples", flush=True)
        return None

    # Rise time to 90% of target
    threshold = 0.9 * target
    rise_t = None
    overshoot = 0.0
    for t, ds in samples:
        if rise_t is None and ds.pitch_rad >= threshold:
            rise_t = t
        overshoot = max(overshoot, ds.pitch_rad - target)

    print(f"\n  >>> rise time to 90% (18°): {rise_t}", flush=True)
    print(f"  >>> overshoot beyond target: {math.degrees(overshoot):.1f}°", flush=True)
    return rise_t


def test_thrust_to_accel(mavlink_conn, shared, system_boot_ms, hover_thrust):
    print("\n=== TEST 3: thrust → vertical acceleration ===", flush=True)
    if hover_thrust is None:
        print("  skipping — no hover thrust measured", flush=True)
        return None

    test_thrust = hover_thrust + 0.10
    print(f"  thrust={test_thrust:.3f} for 1.5s ...", flush=True)
    samples = []
    run_for(
        mavlink_conn, shared, system_boot_ms,
        duration_s=1.5,
        attitude=(0.0, 0.0, 0.0, test_thrust),
        sample_log=samples,
    )

    if len(samples) < 4:
        print("  not enough samples", flush=True)
        return None

    # Look at early-window slope of vd (before drag dominates)
    early = [s for s in samples if s[0] <= 0.6]
    if len(early) < 3:
        early = samples[:5]

    t0, ds0 = early[0]
    t1, ds1 = early[-1]
    dt = t1 - t0
    accel_d = (ds1.vd_mps - ds0.vd_mps) / dt if dt > 0 else 0.0
    accel_up = -accel_d  # positive = upward

    accel_per_unit = accel_up / 0.10
    print(f"\n  >>> Vertical accel ≈ {accel_up:.2f} m/s² for +0.10 thrust", flush=True)
    print(f"  >>> VERTICAL_ACCEL_PER_UNIT_THRUST ≈ {accel_per_unit:.1f}", flush=True)
    return accel_per_unit


def main():
    system_boot_ms = int(time.time() * 1000)
    components = setup_components(SIM_SERVER_UDP_IP, SIM_SERVER_UDP_PORT, system_boot_ms)
    mavlink_conn = components["mavlink_conn"]
    shared = components["shared"]

    # Initial arm
    print("Arming...", flush=True)
    send_arm(mavlink_conn)

    # Wait for armed + telemetry
    print("Waiting for armed=True and drone_state...", flush=True)
    deadline = time.time() + 10.0
    while time.time() < deadline:
        if shared.heartbeat and shared.heartbeat.armed and shared.drone_state is not None:
            break
        send_arm(mavlink_conn)
        time.sleep(0.3)
    else:
        print("Did not get armed + telemetry. Aborting.", flush=True)
        return

    print("Ready. Running system ID...", flush=True)

    hover = test_hover_thrust(mavlink_conn, shared, system_boot_ms)
    rise_t = test_attitude_response(mavlink_conn, shared, system_boot_ms)
    accel_per_unit = test_thrust_to_accel(mavlink_conn, shared, system_boot_ms, hover)

    print("\n\n========== RESULTS ==========", flush=True)
    print(f"HOVER_THRUST                       = {hover}", flush=True)
    print(f"ATTITUDE_RISE_TIME_S               = {rise_t}", flush=True)
    print(f"VERTICAL_ACCEL_PER_UNIT_THRUST     = {accel_per_unit}", flush=True)
    print("\nCopy these into measurements.py.", flush=True)

    # Land softly: low thrust, level
    print("\nCutting thrust...", flush=True)
    run_for(mavlink_conn, shared, system_boot_ms, 2.0, (0.0, 0.0, 0.0, 0.0))

    for name in ("mavlink_rx", "timesync", "heartbeat"):
        components[name].get_thread_for_join().join(timeout=1.0)


if __name__ == "__main__":
    main()
