"""Geometry checks for the gate bearing model. No simulator needed.

Sign errors in camera projection are the same class of bug that cost us
several official-sim runs during VQ1. Catch them here for free, in both
frame conventions.

    python3 tests/test_bearing.py
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bearing import (  # noqa: E402
    AIGP_CAM,
    ELODIN_CAM,
    CAM_TILT_UP_DEG,
    GATE_INNER_W,
    observe_from_truth,
)

IDENT = [1.0, 0.0, 0.0, 0.0]
ORIGIN = (0.0, 0.0, 0.0)
_fails = []


def check(name, cond, detail=""):
    print(f"    {'ok  ' if cond else 'FAIL'}  {name}{('  — ' + detail) if detail else ''}")
    if not cond:
        _fails.append(name)


def main():
    print("=== gate bearing geometry ===\n")

    # ── Elodin: FLU body (x fwd, y LEFT, z up), identity attitude ────────
    print("[Elodin FLU / ENU]")
    ahead = observe_from_truth(ELODIN_CAM, IDENT, ORIGIN, (10.0, 0.0, 0.0))
    check("gate ahead is visible", ahead is not None)
    if ahead:
        check("ahead: az ~ 0", abs(ahead.az) < 1e-6, f"az={math.degrees(ahead.az):.2f} deg")
        # Camera pitches 20 deg UP, so a level gate sits BELOW the axis.
        check("ahead: el ~ -tilt (camera looks up)",
              abs(math.degrees(ahead.el) + CAM_TILT_UP_DEG) < 0.01,
              f"el={math.degrees(ahead.el):.2f} deg")
        check("range from apparent width ~ 10 m",
              abs(ahead.range_m - 10.0) < 0.05, f"range={ahead.range_m:.2f} m")

    # In FLU, +y is LEFT, so a gate at -y must read to the RIGHT.
    right = observe_from_truth(ELODIN_CAM, IDENT, ORIGIN, (10.0, -2.0, 0.0))
    left = observe_from_truth(ELODIN_CAM, IDENT, ORIGIN, (10.0, 2.0, 0.0))
    check("gate at -y reads RIGHT (az > 0)", right is not None and right.az > 0,
          f"az={math.degrees(right.az):.1f} deg" if right else "not visible")
    check("gate at +y reads LEFT (az < 0)", left is not None and left.az < 0,
          f"az={math.degrees(left.az):.1f} deg" if left else "not visible")

    # Raising the gate to the camera's tilt line should centre it vertically.
    on_axis_h = 10.0 * math.tan(math.radians(CAM_TILT_UP_DEG))
    on_axis = observe_from_truth(ELODIN_CAM, IDENT, ORIGIN, (10.0, 0.0, on_axis_h))
    check("gate on the tilt line: el ~ 0",
          on_axis is not None and abs(on_axis.el) < 1e-6,
          f"el={math.degrees(on_axis.el):.3f} deg" if on_axis else "not visible")
    check("higher gate reads UP (el increases)",
          on_axis is not None and ahead is not None and on_axis.el > ahead.el)

    check("gate behind is not visible",
          observe_from_truth(ELODIN_CAM, IDENT, ORIGIN, (-10.0, 0.0, 0.0)) is None)
    check("gate far off to the side is not visible",
          observe_from_truth(ELODIN_CAM, IDENT, ORIGIN, (2.0, -30.0, 0.0)) is None)

    # Confidence should fall off as the gate approaches the frame edge.
    centred = observe_from_truth(ELODIN_CAM, IDENT, ORIGIN, (10.0, 0.0, on_axis_h))
    # y=-10 puts the gate ~19 px from the frame edge, inside the 40 px
    # falloff band. (y=-9 lands 48 px out — outside it; y=-11 leaves frame.)
    edgeward = observe_from_truth(ELODIN_CAM, IDENT, ORIGIN, (10.0, -10.0, on_axis_h))
    check("confidence is high when centred", centred is not None and centred.confidence > 0.99)
    check("confidence drops near the edge",
          edgeward is not None and edgeward.confidence < centred.confidence,
          f"{edgeward.confidence:.2f} < {centred.confidence:.2f}" if edgeward else "not visible")

    # ── Official sim: FRD body (x fwd, y RIGHT, z DOWN) ─────────────────
    print("\n[AIGP FRD / NED]")
    a2 = observe_from_truth(AIGP_CAM, IDENT, ORIGIN, (10.0, 0.0, 0.0))
    check("gate ahead is visible", a2 is not None)
    if a2:
        check("ahead: az ~ 0", abs(a2.az) < 1e-6)
        check("ahead: el ~ -tilt", abs(math.degrees(a2.el) + CAM_TILT_UP_DEG) < 0.01,
              f"el={math.degrees(a2.el):.2f} deg")
    r2 = observe_from_truth(AIGP_CAM, IDENT, ORIGIN, (10.0, 2.0, 0.0))
    check("gate at +y reads RIGHT (az > 0)", r2 is not None and r2.az > 0,
          f"az={math.degrees(r2.az):.1f} deg" if r2 else "not visible")
    # In NED, UP is -z.
    u2 = observe_from_truth(AIGP_CAM, IDENT, ORIGIN, (10.0, 0.0, -5.0))
    check("gate at -z (above) reads UP (el > el_ahead)",
          u2 is not None and a2 is not None and u2.el > a2.el,
          f"el={math.degrees(u2.el):.1f} deg" if u2 else "not visible")

    print()
    if _fails:
        print(f"RESULT: FAIL — {len(_fails)} check(s): {', '.join(_fails)}")
        return 1
    print("RESULT: PASS — projection and sign conventions correct in both frames")
    return 0


if __name__ == "__main__":
    sys.exit(main())
