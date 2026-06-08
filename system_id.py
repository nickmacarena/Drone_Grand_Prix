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
import sys
import time

from attitude import euler_to_quaternion
from mavlink_tx import send_arm, send_attitude_quaternion
from setup import setup_components


SIM_SERVER_UDP_IP = "0.0.0.0"
SIM_SERVER_UDP_PORT = 14550

LOOP_HZ = 250
LOOP_PERIOD = 1.0 / LOOP_HZ

OUTPUT_FILE = "system_id_results.txt"


# Detailed output goes to file; only minimal progress to terminal.
_log_file = None


def log(msg: str = ""):
    """Write to results file; never to terminal."""
    if _log_file is not None:
        _log_file.write(msg + "\n")
        _log_file.flush()


def status(msg: str):
    """One-line terminal status."""
    sys.stdout.write(msg + "\n")
    sys.stdout.flush()
    log(msg)


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
    status("[1/3] hover thrust sweep")
    log("\n=== TEST 1: hover thrust sweep ===")
    results = {}
    for thrust in (0.40, 0.45, 0.50, 0.55, 0.60):
        log(f"  thrust={thrust:.2f} for 2.5s ...")
        samples = []
        run_for(
            mavlink_conn, shared, system_boot_ms,
            duration_s=2.5,
            attitude=(0.0, 0.0, 0.0, thrust),
            sample_log=samples,
        )
        if not samples:
            log("    no samples")
            continue
        recent = [ds.vd_mps for t, ds in samples if t >= 1.5]
        if not recent:
            recent = [samples[-1][1].vd_mps]
        mean_vd = sum(recent) / len(recent)
        results[thrust] = mean_vd
        log(f"    terminal vd ≈ {mean_vd:+.2f} m/s")

    keys = sorted(results.keys())
    hover = None
    for a, b in zip(keys, keys[1:]):
        if results[a] * results[b] <= 0:
            t = results[a] / (results[a] - results[b])
            hover = a + t * (b - a)
            break
    if hover is not None:
        log(f"\n  >>> HOVER_THRUST ≈ {hover:.3f}")
    else:
        log("\n  >>> Could not bracket hover thrust. Widen the sweep.")
    return hover


def test_attitude_response(mavlink_conn, shared, system_boot_ms):
    status("[2/3] attitude step response")
    log("\n=== TEST 2: attitude step response (pitch 20°) ===")
    log("  pre-step level for 1s")
    run_for(mavlink_conn, shared, system_boot_ms, 1.0, (0.0, 0.0, 0.0, 0.5))

    log("  step to pitch=20° for 2s ...")
    samples = []
    target = math.radians(20.0)
    run_for(
        mavlink_conn, shared, system_boot_ms,
        duration_s=2.0,
        attitude=(0.0, target, 0.0, 0.5),
        sample_log=samples,
    )

    if not samples:
        log("  no samples")
        return None

    threshold = 0.9 * target
    rise_t = None
    overshoot = 0.0
    for t, ds in samples:
        if rise_t is None and ds.pitch_rad >= threshold:
            rise_t = t
        overshoot = max(overshoot, ds.pitch_rad - target)

    log(f"\n  >>> rise time to 90% (18°): {rise_t}")
    log(f"  >>> overshoot beyond target: {math.degrees(overshoot):.1f}°")
    return rise_t


def test_thrust_to_accel(mavlink_conn, shared, system_boot_ms, hover_thrust):
    status("[3/3] thrust → vertical accel")
    log("\n=== TEST 3: thrust → vertical acceleration ===")
    if hover_thrust is None:
        log("  skipping — no hover thrust measured")
        return None

    test_thrust = hover_thrust + 0.10
    log(f"  thrust={test_thrust:.3f} for 1.5s ...")
    samples = []
    run_for(
        mavlink_conn, shared, system_boot_ms,
        duration_s=1.5,
        attitude=(0.0, 0.0, 0.0, test_thrust),
        sample_log=samples,
    )

    if len(samples) < 4:
        log("  not enough samples")
        return None

    early = [s for s in samples if s[0] <= 0.6]
    if len(early) < 3:
        early = samples[:5]

    t0, ds0 = early[0]
    t1, ds1 = early[-1]
    dt = t1 - t0
    accel_d = (ds1.vd_mps - ds0.vd_mps) / dt if dt > 0 else 0.0
    accel_up = -accel_d

    accel_per_unit = accel_up / 0.10
    log(f"\n  >>> Vertical accel ≈ {accel_up:.2f} m/s² for +0.10 thrust")
    log(f"  >>> VERTICAL_ACCEL_PER_UNIT_THRUST ≈ {accel_per_unit:.1f}")
    return accel_per_unit


def main():
    global _log_file
    _log_file = open(OUTPUT_FILE, "w")

    status(f"system_id starting — detailed log in {OUTPUT_FILE}")
    log(f"system_id run at {time.strftime('%Y-%m-%d %H:%M:%S')}")

    system_boot_ms = int(time.time() * 1000)
    components = setup_components(SIM_SERVER_UDP_IP, SIM_SERVER_UDP_PORT, system_boot_ms)
    mavlink_conn = components["mavlink_conn"]
    shared = components["shared"]

    # The sim only starts streaming position/attitude AFTER you click RACE.
    # So system ID consumes one race attempt. That's fine — VQ1 has unlimited attempts.
    status("CLICK RACE IN THE SIM to start system ID. Waiting up to 60s for telemetry...")
    send_arm(mavlink_conn)
    deadline = time.time() + 60.0
    last_arm = time.time()
    while time.time() < deadline:
        if shared.drone_state is not None and shared.heartbeat and shared.heartbeat.armed:
            break
        if time.time() - last_arm >= 0.5:
            send_arm(mavlink_conn)
            last_arm = time.time()
        time.sleep(0.05)
    else:
        status("FAILED: no telemetry. Did you click RACE? See log.")
        _log_file.close()
        return

    status("telemetry received — running tests now")

    hover = test_hover_thrust(mavlink_conn, shared, system_boot_ms)
    rise_t = test_attitude_response(mavlink_conn, shared, system_boot_ms)
    accel_per_unit = test_thrust_to_accel(mavlink_conn, shared, system_boot_ms, hover)

    log("\n\n========== RESULTS ==========")
    log(f"HOVER_THRUST                       = {hover}")
    log(f"ATTITUDE_RISE_TIME_S               = {rise_t}")
    log(f"VERTICAL_ACCEL_PER_UNIT_THRUST     = {accel_per_unit}")
    log("\nCopy these into measurements.py.")

    log("\nCutting thrust...")
    run_for(mavlink_conn, shared, system_boot_ms, 2.0, (0.0, 0.0, 0.0, 0.0))

    for name in ("mavlink_rx", "timesync", "heartbeat"):
        components[name].get_thread_for_join().join(timeout=1.0)

    _log_file.close()
    status(f"done — results in {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
