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

    print()
    if _fails:
        print(f"RESULT: FAIL — {len(_fails)}: {', '.join(_fails)}")
        return 1
    print("RESULT: PASS — servo converges onto the gate in both frames")
    return 0


if __name__ == "__main__":
    sys.exit(main())
