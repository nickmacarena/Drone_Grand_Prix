"""In-flight yaw polarity self-check (servo.check_polarity).

Runs 6-10 all failed the same way: YAW_SIGN was +1, which textbook FRD says is
correct for "turn right" and this sim says is not, so the azimuth loop sat in
positive feedback. Five official runs to notice. The relationship is directly
measurable in flight — commanding yaw toward a gate must SHRINK its azimuth —
so a wrong constant is now self-correcting rather than a lost run.

Asserts both directions: an inverted plant gets flipped, a correct one is left
alone. Pure Python, no simulator.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import servo

_fails = []


def check(name, ok, detail=""):
    print(f"    {'ok  ' if ok else 'FAIL'}  {name}" + (f"  — {detail}" if detail else ""))
    if not ok:
        _fails.append(name)


def fly(plant_inverted, yaw_sign=+1.0):
    """Close the azimuth loop against a plant that may steer the wrong way."""
    chk = servo.PolarityCheck()
    az = 0.5                              # gate 29 deg to the right
    flips = 0
    for _ in range(600):
        cmd = max(-1.6, min(1.6, 2.0 * az))          # servo: steer right
        applied = cmd * yaw_sign * (-1.0 if plant_inverted else 1.0)
        daz = -applied * 0.02
        az += daz
        verdict = servo.check_polarity(chk, cmd, daz / 0.02, 30.0)
        if verdict < 0:
            yaw_sign = -yaw_sign
            flips += 1
        if chk.decided:
            break
    return yaw_sign, chk, flips, abs(az)


def main():
    print("=== yaw polarity self-check ===\n")

    sign, chk, flips, _ = fly(plant_inverted=False)
    check("correct plant is left alone", flips == 0 and sign > 0,
          f"{chk.total - chk.wrong}/{chk.total} samples read correct")

    sign, chk, flips, _ = fly(plant_inverted=True)
    check("inverted plant is flipped", flips == 1 and sign < 0,
          f"{chk.wrong}/{chk.total} samples read wrong")

    # And it must converge once flipped, not oscillate.
    chk2 = servo.PolarityCheck()
    yaw_sign, az = -1.0, 0.5
    for _ in range(400):
        cmd = max(-1.6, min(1.6, 2.0 * az))
        applied = cmd * yaw_sign * -1.0          # inverted plant, corrected sign
        daz = -applied * 0.02
        az += daz
        if servo.check_polarity(chk2, cmd, daz / 0.02, 30.0) < 0:
            yaw_sign = -yaw_sign
    check("flipped sign then converges", abs(az) < 0.05,
          f"|az| settled at {math.degrees(abs(az)):.1f} deg")

    # Close-range evidence must be refused outright: that is what invalidated
    # run 11's mid-flight flip.
    near = servo.PolarityCheck()
    for _ in range(200):
        servo.check_polarity(near, 1.0, 1.0, 3.7)     # wrong sign, but 3.7 m out
    check("close-range evidence is refused", near.total == 0 and not near.decided,
          f"accumulated {near.total} samples at 3.7 m")
    unknown = servo.PolarityCheck()
    for _ in range(200):
        servo.check_polarity(unknown, 1.0, 1.0, None)
    check("missing range is refused", unknown.total == 0)

    print()
    if _fails:
        print(f"RESULT: FAIL — {', '.join(_fails)}")
        return 1
    print("RESULT: PASS — inverted steering is detected and corrected in flight")
    return 0


if __name__ == "__main__":
    sys.exit(main())
