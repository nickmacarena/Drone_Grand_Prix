"""Closed-loop servo convergence, in a 20-line kinematic sim. No simulator.

Uses the REAL bearing and servo code; only the vehicle is faked (heading
integrates the yaw command, position advances along heading). That is enough
to prove the whole sign chain converges:

    gate geometry -> camera projection -> level-frame bearing -> servo
                  -> yaw/forward/vertical commands -> motion -> closes on gate

The first Elodin Stage 3a runs failed on exactly this chain: raw elevation
coupled to pitch, then a yaw command that steered AWAY (in FLU, +body-z is
UP, so positive yaw rate turns LEFT while positive azimuth means RIGHT).
Both are caught here in milliseconds instead of 90-second sim runs.

    python3 tests/test_servo.py
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import servo  # noqa: E402
from attitude import euler_to_quaternion  # noqa: E402
from bearing import (  # noqa: E402
    AIGP_CAM,
    ELODIN_CAM,
    observe_from_truth,
    stabilized_bearing,
)

# Body-z yaw-rate sign for a "turn right" command, per frame convention.
#   FLU (up = +z): +body-z rotates fwd toward LEFT  -> need -1
#   FRD (up = -z): +body-z rotates fwd toward RIGHT -> need +1
YAW_SIGN_FLU = -1.0
YAW_SIGN_FRD = +1.0

# Pitch sign that puts the NOSE DOWN for positive forward tilt. Rotating +x
# about +y by t gives (cos t, 0, -sin t): that is nose-down in FLU (+z up)
# and nose-up in FRD (+z down), so the frames need opposite signs.
PITCH_SIGN_FLU = +1.0
PITCH_SIGN_FRD = -1.0
MAX_TILT_RAD = math.radians(25.0)   # matches flight_stack

_fails = []


def check(name, cond, detail=""):
    print(f"    {'ok  ' if cond else 'FAIL'}  {name}{('  — ' + detail) if detail else ''}")
    if not cond:
        _fails.append(name)


def fly(gate, start=(0.0, 0.0, 0.0), heading0=0.0, *, cam, up_world, yaw_sign,
        pitch_sign, enu=True, speed=8.0, climb=6.0, dt=0.02, duration=25.0):
    """Kinematic pursuit. Returns (min_distance, reached, track)."""
    cfg = servo.ServoConfig()
    st = servo.ServoState()
    x, y, z = start
    heading = heading0
    t = 0.0
    best = float("inf")
    track = []

    pitch = 0.0
    while t < duration:
        # Model the pitch the flight stack would hold for this forward tilt.
        # This is NOT cosmetic: the camera is body-mounted, so nose-down
        # pitch swings its view from ~9 deg below the flight path to ~23 deg.
        # A level-flight model cannot see that mechanism and wrongly reports
        # that a low gate is unreachable.
        q = euler_to_quaternion(0.0, pitch_sign * pitch, heading)

        obs = observe_from_truth(cam, q, (x, y, z), gate)
        target = None
        if obs is not None:
            az, el = stabilized_bearing(cam, obs, q, up_world)
            target = servo.Target(az=az, el=el, confidence=obs.confidence)

        out = servo.step(st, cfg, t, target)

        # Heading is the rotation angle about the frame's own z axis, so
        # dheading = body_z_rate * dt in BOTH conventions. `yaw_sign` converts
        # the servo's "turn right" into that body-z rate:
        #   FLU (+z up)   -> +body_z turns LEFT  -> -1
        #   FRD (+z down) -> +body_z turns RIGHT -> +1
        body_z_rate = yaw_sign * out.yaw_rate
        heading += body_z_rate * dt
        pitch += 6.0 * (out.tilt_fwd * MAX_TILT_RAD - pitch) * dt  # first-order lag
        v = speed * out.tilt_fwd
        x += v * math.cos(heading) * dt
        y += v * math.sin(heading) * dt
        z += (1.0 if enu else -1.0) * climb * out.vertical * dt

        d = math.dist((x, y, z), gate)
        best = min(best, d)
        track.append((round(t, 2), round(x, 1), round(y, 1), round(z, 1), round(d, 1)))
        t += dt

    return best, best < 1.5, track


def main():
    print("=== servo closed-loop convergence (kinematic) ===\n")

    print("[Elodin FLU / ENU]")
    cases = {
        "gate straight ahead":      (25.0, 0.0, 0.0),
        "gate 20 deg to the right": (25.0, -9.0, 0.0),
        "gate 20 deg to the left":  (25.0, 9.0, 0.0),
        "gate above":               (25.0, 0.0, 6.0),
        "gate below":               (25.0, 0.0, -6.0),
    }
    for name, gate in cases.items():
        best, ok, track = fly(gate, cam=ELODIN_CAM, up_world=(0.0, 0.0, 1.0),
                              yaw_sign=YAW_SIGN_FLU, pitch_sign=PITCH_SIGN_FLU,
                              enu=True)
        check(name, ok, f"closest approach {best:.2f} m")
        if not ok:
            print("           track:", track[::50][:8])

    print("\n[AIGP FRD / NED]  (up = -z, gate 'above' is -z)")
    for name, gate in {
        "gate straight ahead":      (25.0, 0.0, 0.0),
        "gate 20 deg to the right": (25.0, 9.0, 0.0),
        "gate above":               (25.0, 0.0, -6.0),
    }.items():
        best, ok, _ = fly(gate, cam=AIGP_CAM, up_world=(0.0, 0.0, -1.0),
                          yaw_sign=YAW_SIGN_FRD, pitch_sign=PITCH_SIGN_FRD,
                          enu=False)
        check(name, ok, f"closest approach {best:.2f} m")

    print("\n[post-gate handling — official run 8 regression]")
    test_no_slam_after_gate_pass()

    print("\n[search behaviour — official run 7 regression]")
    test_search_is_bounded()
    test_gate_passed_clears_stale_track()

    print("\n[vertical dynamics — official run 6 regression]")
    test_vertical_overshoot()

    print("\n[track continuity — official run 6 regression]")
    test_run6_impostor_rejection()
    test_acquire_needs_confidence()

    print()
    if _fails:
        print(f"RESULT: FAIL — {len(_fails)}: {', '.join(_fails)}")
        return 1
    print("RESULT: PASS — servo converges onto the gate in both frames")
    return 0


def test_run6_impostor_rejection():
    """Official run 6: the servo span up because it chased impostor blobs.

    The three sightings below are the real ones from vq2_race5.log. No single
    gate can be 4.3 m away, then 1.0 m, then 12.6 m, while jumping from u=331
    to u=621 px. The servo must hold its track through them instead of
    saturating yaw toward the last one.
    """
    cfg = servo.ServoConfig()
    st = servo.ServoState()

    def az_of(u):
        return math.atan2((u - 320.0) / 320.0, 1.0)

    # Establish a track on a good, confident sighting.
    servo.step(st, cfg, 0.0, servo.Target(az=az_of(323.1), el=0.0,
                                          confidence=1.0, range_m=5.8))
    check("track acquired", st.have_fix)

    # Consistent follow-up: accepted.
    servo.step(st, cfg, 0.1, servo.Target(az=az_of(331.2), el=0.0,
                                          confidence=1.0, range_m=4.3))
    check("consistent sighting accepted", st.rejected == 0)

    # Impostor 1: range collapses to 1.0 m (a 480 px blob) in 0.1 s.
    servo.step(st, cfg, 0.2, servo.Target(az=az_of(403.5), el=0.0,
                                          confidence=1.0, range_m=1.0))
    # Impostor 2: teleports to the frame edge and back out to 12.6 m.
    out = servo.step(st, cfg, 0.3, servo.Target(az=az_of(621.3), el=0.0,
                                                confidence=0.44, range_m=12.6))
    check("impostors rejected", st.rejected >= 1, f"rejected={st.rejected}")
    check("yaw not saturated by impostor",
          abs(out.yaw_rate) < 0.9 * cfg.max_yaw_rate,
          f"yaw_rate={out.yaw_rate:.2f} (max {cfg.max_yaw_rate})")
    check("track survived", out.have_target)


def test_acquire_needs_confidence():
    """A marginal blob may sustain a track but must not start one."""
    cfg = servo.ServoConfig()
    st = servo.ServoState()
    servo.step(st, cfg, 0.0, servo.Target(az=0.0, el=0.0, confidence=0.30,
                                          range_m=8.0))
    check("weak sighting does not acquire", not st.have_fix)
    servo.step(st, cfg, 0.1, servo.Target(az=0.0, el=0.0, confidence=0.90,
                                          range_m=8.0))
    check("strong sighting acquires", st.have_fix)
    servo.step(st, cfg, 0.2, servo.Target(az=0.02, el=0.0, confidence=0.30,
                                          range_m=8.2))
    check("weak sighting sustains existing track", st.time_since_seen == 0.0)


def test_vertical_overshoot():
    """Arrive INSIDE the gate opening, not above it (official run 6).

    The convergence tests above are kinematic — they command velocity directly,
    so they can never exhibit overshoot, and they passed happily while the real
    aircraft climbed through gate 0's centre and clipped its top bar. This
    integrates the actual plant (thrust -> accel -> velocity -> altitude), so
    a P-only vertical loop fails it.

    Hard limit: the gate's inner opening is 1.5 m, so the vertical miss at the
    gate plane must stay well inside +/- 0.75 m.
    """
    G = 9.81
    VERT_AUTH_FRAC = 0.8          # must match controller_vq2
    HALF_OPENING = 0.75

    def approach(gate_up, closing, cfg):
        st = servo.ServoState()
        z = vz = t = 0.0
        dist = closing * 3.0
        while t < 8.0 and dist > 0.0:
            el = math.atan2(gate_up - z, max(dist, 0.05))
            obs = (servo.Target(az=0.0, el=el, confidence=1.0,
                                range_m=math.hypot(dist, gate_up - z))
                   if int(t / 0.02) % 3 == 0 else None)   # detector ~15 Hz
            out = servo.step(st, cfg, t, obs)
            vz += G * VERT_AUTH_FRAC * out.vertical * 0.02
            z += vz * 0.02
            dist -= closing * 0.02
            t += 0.02
        return z - gate_up

    cfg = servo.ServoConfig()
    worst = 0.0
    for gate_up in (-2.0, -1.0, 0.0, 1.2, 2.4, 3.5):
        for closing in (4.0, 6.0, 9.0):
            worst = max(worst, abs(approach(gate_up, closing, cfg)))
    check("vertical miss fits the gate opening", worst < 0.5 * HALF_OPENING,
          f"worst miss {worst:.2f} m (half-opening {HALF_OPENING} m)")

    # And prove the test has teeth: with the damping removed it must FAIL.
    from dataclasses import replace
    undamped = replace(cfg, kd_el=0.0)
    worst_p_only = max(abs(approach(u, c, undamped))
                       for u in (2.4, 3.5) for c in (6.0, 9.0))
    check("test detects a P-only loop", worst_p_only > HALF_OPENING,
          f"P-only misses by {worst_p_only:.2f} m — would hit the top bar")


def test_search_is_bounded():
    """Official run 7: search yawed one way forever and sank into the floor.

    After clearing gate 0 the aircraft turned ~147 deg off course chasing
    distant hangar gates, and sank from -1.1 to -9.4 m/s because hover thrust
    PRESERVES a descent when altitude is unobservable. A search must stay near
    the heading it started from and must not walk the aircraft down.
    """
    cfg = servo.ServoConfig()
    st = servo.ServoState()
    servo.gate_passed(st)          # as the controller does on index advance

    dt = 0.02
    heading = 0.0        # integrate the commanded yaw rate
    vert_sum = 0.0
    worst_heading = 0.0
    for i in range(int(12.0 / dt)):
        out = servo.step(st, cfg, i * dt, None)     # nothing ever seen
        heading += out.yaw_rate * dt
        vert_sum += out.vertical * dt
        worst_heading = max(worst_heading, abs(heading))

    check("search stays near its starting heading",
          worst_heading < math.radians(150.0),
          f"swept to {math.degrees(worst_heading):.0f} deg")
    check("search reverses rather than spinning one way",
          st.sweep_dir != 0.0 and abs(heading) < worst_heading * 0.9,
          f"ended {math.degrees(heading):.0f} deg vs peak "
          f"{math.degrees(worst_heading):.0f} deg")
    check("search does not sink on average", abs(vert_sum) < 0.15,
          f"integrated vertical effort {vert_sum:+.3f}")


def test_gate_passed_clears_stale_track():
    """A track describing a gate now behind us must not steer the next leg."""
    cfg = servo.ServoConfig()
    st = servo.ServoState()
    servo.step(st, cfg, 0.0, servo.Target(az=0.5, el=0.1, confidence=1.0,
                                          range_m=4.0))
    check("track exists before pass", st.have_fix)
    servo.gate_passed(st)
    check("track cleared by gate pass", not st.have_fix)
    check("smoothed bearing cleared", st.az_f == 0.0)
    # last_az is deliberately NOT zeroed: it (or a fresher next-gate hint) is
    # the only clue we have about which way the course turns. Runs 7-9 lost
    # gate 1 partly because this was thrown away.
    check("search prior retained for the next leg", st.last_az != 0.0)


def test_no_slam_after_gate_pass():
    """Official run 8: acquired gate 1 at 42 deg right and snapped over.

    The sighting was legitimate — the course turns hard right after gate 0, and
    u=609 with conf=0.74 is exactly the detector's edge-fade at that position.
    What killed it was commanding 1.47 rad/s in one step, 1 m past the gate
    plane with gate 0's posts still alongside. It clipped one and inverted.

    So: steer toward it, but not instantly, and not while still in the gate.
    """
    cfg = servo.ServoConfig()
    st = servo.ServoState()
    dt = 0.02
    t = 0.0
    servo.gate_passed(st, t, cfg)

    az_edge = math.atan2((609.6 - 320.0) / 320.0, 1.0)      # 42 deg
    tgt = servo.Target(az=az_edge, el=0.0, confidence=0.74, range_m=15.0)

    # First moments after the pass: tracking may start, steering must not.
    peak_during_clear = 0.0
    while t < cfg.clear_gate_s:
        out = servo.step(st, cfg, t, tgt)
        peak_during_clear = max(peak_during_clear, abs(out.yaw_rate))
        t += dt
    check("no steering while clearing the gate", peak_during_clear < 0.05,
          f"peak yaw {peak_during_clear:.3f} rad/s during clearance")

    # After clearance it should turn toward the gate, but smoothly.
    worst_jump = 0.0
    prev = out.yaw_rate
    for _ in range(60):
        out = servo.step(st, cfg, t, tgt)
        worst_jump = max(worst_jump, abs(out.yaw_rate - prev) / dt)
        prev = out.yaw_rate
        t += dt
    check("yaw slews rather than snapping",
          worst_jump <= cfg.max_yaw_accel * 1.05,
          f"peak yaw accel {worst_jump:.2f} rad/s^2 "
          f"(limit {cfg.max_yaw_accel})")
    check("it does eventually turn toward the gate", out.yaw_rate > 0.3,
          f"yaw settled at {out.yaw_rate:+.2f} rad/s")


if __name__ == "__main__":
    sys.exit(main())
