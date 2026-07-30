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

    # ── servo.yaw_convention: the MISSION=yawtest measurement ──────────
    def synth(standard, n=40, span_deg=60.0, fx=320.0):
        """Gate fixed in the world, drone rotating: u = cx + fx*tan(bearing).

        Samples outside the 640 px frame are dropped, because the camera cannot
        report them — over a 100 deg rotation a gate leaves the 90 deg HFoV
        partway through, so only the visible part is available to regress.
        """
        pts = []
        for k in range(n):
            yaw = math.radians(span_deg) * k / (n - 1)
            bearing = -yaw if standard else +yaw   # right turn sweeps scene left
            u = 320.0 + fx * math.tan(bearing)
            if 0.0 <= u < 640.0:
                pts.append((yaw, u))
        return pts

    # Span-independence is the point of working in bearing space.
    for span_deg in (20.0, 60.0, 100.0):
        v, slope, _, note = servo.yaw_convention(synth(True, span_deg=span_deg))
        check(f"standard at {span_deg:.0f} deg span", v == "standard",
              f"slope={slope:+.2f} {note}")

    v, slope, span, _ = servo.yaw_convention(synth(True))
    check("standard rotation reads standard", v == "standard",
          f"verdict={v} slope={slope:+.2f} over {span:.0f} deg")
    v, slope, _, _ = servo.yaw_convention(synth(False))
    check("inverted rotation reads inverted", v == "inverted",
          f"verdict={v} slope={slope:+.2f}")

    # The failure that fooled the first offline attempt: a gate close enough to
    # fill the frame has its centroid pinned, so 37.7 deg of yaw moved u by
    # 2.9 px where pure rotation demands ~247. The SIGN was right; believing it
    # would still have been wrong.
    pinned = [(math.radians(60.0) * k / 39, 320.0 - 0.077 * k) for k in range(40)]
    v, slope, _, note = servo.yaw_convention(pinned)
    check("pinned-centroid data is refused", v == "inconclusive",
          f"slope {slope:+.2f} — {note}")

    v, _, _, note = servo.yaw_convention(
        [(math.radians(0.05) * k, 320.0 - 2.0 * k) for k in range(40)])
    check("no-rotation data is refused", v == "inconclusive", note)
    v, _, _, note = servo.yaw_convention([(0.0, 320.0)])
    check("too few sightings refused", v == "inconclusive", note)

    print()
    if _fails:
        print(f"RESULT: FAIL — {', '.join(_fails)}")
        return 1
    print("RESULT: PASS — inverted steering is detected and corrected in flight")
    return 0


if __name__ == "__main__":
    sys.exit(main())
