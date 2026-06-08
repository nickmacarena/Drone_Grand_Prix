"""System identification — measure drone parameters before tuning.

Standalone script. Run against the live sim during a race attempt (this
consumes one race — that's fine, VQ1 has unlimited attempts).

Strategy: a single short thrust step gives us hover thrust algebraically.
Force balance:
    accel_up = g × (thrust - hover) / hover
    →  hover = g × thrust / (accel_up + g)

So one ~1-second measurement at a known thrust → hover thrust + thrust
gain. A second brief test (pitch step) gives us attitude rise time.

Total testing time ~3 seconds. We wait for race_started=True before
sending any non-zero commands to avoid an early-start DQ.

Output goes to system_id_results.txt; terminal shows only progress.
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

G = 9.81  # m/s^2, standard gravity


_log_file = None


def log(msg: str = ""):
    if _log_file is not None:
        _log_file.write(msg + "\n")
        _log_file.flush()


def status(msg: str):
    sys.stdout.write(msg + "\n")
    sys.stdout.flush()
    log(msg)


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
    roll, pitch, yaw, thrust = attitude
    start = time.time()
    last_arm_t = 0.0
    sample_period = 0.02  # 50 Hz sampling
    last_sample = 0.0

    while time.time() - start < duration_s:
        last_arm_t = keep_armed(mavlink_conn, shared, last_arm_t)
        send_attitude(mavlink_conn, system_boot_ms, roll, pitch, yaw, thrust)
        now = time.time()
        if sample_log is not None and now - last_sample >= sample_period:
            ds = shared.drone_state
            if ds is not None:
                sample_log.append((now - start, ds))
            last_sample = now
        time.sleep(LOOP_PERIOD)


def measure_hover_and_thrust_gain(mavlink_conn, shared, system_boot_ms):
    """One short thrust step → hover thrust + thrust-to-accel gain."""
    status("[1/2] hover thrust measurement (~1s)")

    test_thrust = 0.3  # likely below max-thrust DQ but well above zero
    samples = []
    run_for(
        mavlink_conn, shared, system_boot_ms,
        duration_s=1.0,
        attitude=(0.0, 0.0, 0.0, test_thrust),
        sample_log=samples,
    )

    if len(samples) < 6:
        log(f"  Only {len(samples)} samples — not enough")
        return None, None

    # Use early-window slope to avoid drag effects:
    # average vd in first third vs middle third
    n = len(samples)
    early = samples[: n // 3]
    mid = samples[n // 3 : 2 * n // 3]

    t_e = sum(t for t, _ in early) / len(early)
    vd_e = sum(s.vd_mps for _, s in early) / len(early)
    t_m = sum(t for t, _ in mid) / len(mid)
    vd_m = sum(s.vd_mps for _, s in mid) / len(mid)

    dt = t_m - t_e
    if dt <= 0:
        log("  dt error in slope computation")
        return None, None

    accel_down = (vd_m - vd_e) / dt  # m/s^2 in NED (positive = downward)
    accel_up = -accel_down

    log(f"  Test thrust: {test_thrust:.3f}")
    log(f"  Measured accel_up: {accel_up:.2f} m/s^2")

    # Force balance: accel_up = g * (T - T_hover) / T_hover
    # → T_hover = g * T / (accel_up + g)
    hover = G * test_thrust / (accel_up + G)

    # Vertical accel per unit thrust above hover = g / hover
    accel_per_unit = G / hover if hover > 0 else None

    log(f"  >>> HOVER_THRUST ~ {hover:.3f}")
    log(f"  >>> VERTICAL_ACCEL_PER_UNIT_THRUST ~ {accel_per_unit:.1f}")
    return hover, accel_per_unit


def measure_attitude_response(mavlink_conn, shared, system_boot_ms, hover_thrust):
    """Step pitch and measure rise time."""
    status("[2/2] attitude step response (~1.5s)")

    if hover_thrust is None or hover_thrust <= 0:
        log("  no hover thrust — using 0.3")
        hover_thrust = 0.3

    # Brief settle at hover
    run_for(mavlink_conn, shared, system_boot_ms, 0.3, (0.0, 0.0, 0.0, hover_thrust))

    samples = []
    target_pitch = math.radians(15.0)
    run_for(
        mavlink_conn, shared, system_boot_ms,
        duration_s=1.0,
        attitude=(0.0, target_pitch, 0.0, hover_thrust),
        sample_log=samples,
    )

    if not samples:
        log("  no samples")
        return None

    threshold = 0.9 * target_pitch
    rise_t = None
    max_pitch = 0.0
    for t, ds in samples:
        if rise_t is None and ds.pitch_rad >= threshold:
            rise_t = t
        max_pitch = max(max_pitch, ds.pitch_rad)

    overshoot = max(0.0, max_pitch - target_pitch)
    log(f"  Commanded pitch: 15.0°")
    log(f"  Rise time to 90% (13.5°): {rise_t}")
    log(f"  Overshoot: {math.degrees(overshoot):.1f}°")
    log(f"  >>> ATTITUDE_RISE_TIME_S ~ {rise_t}")
    return rise_t


def main():
    global _log_file
    _log_file = open(OUTPUT_FILE, "w", encoding="utf-8")

    status(f"system_id starting — detailed log in {OUTPUT_FILE}")
    log(f"system_id run at {time.strftime('%Y-%m-%d %H:%M:%S')}")

    system_boot_ms = int(time.time() * 1000)
    components = setup_components(SIM_SERVER_UDP_IP, SIM_SERVER_UDP_PORT, system_boot_ms)
    mavlink_conn = components["mavlink_conn"]
    shared = components["shared"]

    status("arming")
    send_arm(mavlink_conn)

    # Wait for race_started — not just drone_state. Sending thrust before
    # the race officially starts triggers an early-start DQ.
    status("CLICK RACE in the sim. Waiting for race_started=True (up to 90s)...")
    deadline = time.time() + 90.0
    last_arm = time.time()
    while time.time() < deadline:
        ds = shared.drone_state
        rs = shared.race_status
        hb = shared.heartbeat
        if ds is not None and rs is not None and rs.race_started and hb and hb.armed:
            break
        if time.time() - last_arm >= 0.5:
            send_arm(mavlink_conn)
            last_arm = time.time()
        time.sleep(0.05)
    else:
        status("FAILED: race never started. See log.")
        _log_file.close()
        return

    status("race started — running tests now")

    hover, accel_per_unit = measure_hover_and_thrust_gain(mavlink_conn, shared, system_boot_ms)
    rise_t = measure_attitude_response(mavlink_conn, shared, system_boot_ms, hover)

    status("")
    status("========== RESULTS ==========")
    status(f"HOVER_THRUST                       = {hover}")
    status(f"VERTICAL_ACCEL_PER_UNIT_THRUST     = {accel_per_unit}")
    status(f"ATTITUDE_RISE_TIME_S               = {rise_t}")
    status("")
    status("Copy these into measurements.py.")

    # Cut thrust
    run_for(mavlink_conn, shared, system_boot_ms, 1.0, (0.0, 0.0, 0.0, 0.0))

    for name in ("mavlink_rx", "timesync", "heartbeat"):
        components[name].get_thread_for_join().join(timeout=1.0)

    _log_file.close()
    status(f"done — results in {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
