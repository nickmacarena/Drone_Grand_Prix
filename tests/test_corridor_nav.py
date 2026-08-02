"""Corridor steering law (corridor_nav). Pure Python, no simulator, no numpy.

The image processing lives in corridor.py and needs numpy; the control law is
kept separate precisely so it can be tested here.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import corridor_nav as CN

_fails = []


def check(name, ok, detail=""):
    print(f"    {'ok  ' if ok else 'FAIL'}  {name}" + (f"  — {detail}" if detail else ""))
    if not ok:
        _fails.append(name)


def run(lane_seq, look=0.0, coverage=1.0, cfg=None, dt=0.05):
    cfg = cfg or CN.CorridorConfig()
    st = CN.CorridorState()
    out = None
    for i, lane in enumerate(lane_seq):
        out = CN.step(st, cfg, i * dt, lane, look, coverage)
    return st, out


def main():
    print("=== corridor steering ===\n")
    cfg = CN.CorridorConfig()

    # Centred lane: fly straight.
    st, out = run([0.0] * 20)
    check("centred lane commands no turn",
          abs(out.yaw_rate) < 1e-6 and abs(out.tilt_right) < 1e-6,
          f"yaw={out.yaw_rate:+.3f} roll={out.tilt_right:+.3f}")
    check("centred lane drives forward", out.tilt_fwd > 0.3,
          f"fwd={out.tilt_fwd:.2f}")

    # Lane to the right: turn right AND roll right. Both, not just yaw — waiting
    # for the nose to come round is what made every gate-servo turn go wide.
    st, out = run([math.radians(12.0)] * 20)
    check("lane right -> yaw right", out.yaw_rate > 0.1,
          f"yaw={out.yaw_rate:+.2f} rad/s")
    check("lane right -> roll right", out.tilt_right > 0.1,
          f"roll={out.tilt_right:+.2f}")
    st, out = run([math.radians(-12.0)] * 20)
    check("lane left -> yaw left", out.yaw_rate < -0.1, f"yaw={out.yaw_rate:+.2f}")
    check("lane left -> roll left", out.tilt_right < -0.1, f"roll={out.tilt_right:+.2f}")

    # Lookahead is feed-forward: a centred lane that BENDS right still turns.
    # This is the entire advantage over gate-chasing — the turn is taken before
    # the next gate is reachable, not after it has left the frame.
    _, straight = run([0.0] * 20, look=0.0)
    _, bending = run([0.0] * 20, look=math.radians(20.0))
    check("a centred lane that bends right still turns right",
          bending.yaw_rate > straight.yaw_rate + 0.05,
          f"{straight.yaw_rate:+.2f} -> {bending.yaw_rate:+.2f} rad/s")

    # Poor coverage must hand back to the gate servo rather than steer on noise.
    _, out = run([math.radians(30.0)] * 20, coverage=0.10)
    check("low coverage yields no lane", not out.have_lane,
          f"coverage 0.10 < min {cfg.min_coverage}")
    _, out = run([math.radians(30.0)] * 20, coverage=0.90)
    check("good coverage yields a lane", out.have_lane)

    # Losing the corridor mid-flight must drop the lane immediately, not coast:
    # the caller falls back to the gate servo and needs to know at once.
    st = CN.CorridorState()
    for i in range(10):
        CN.step(st, cfg, i * 0.05, math.radians(5.0), 0.0, 1.0)
    out = CN.step(st, cfg, 0.55, None, None, 0.0)
    check("losing the corridor drops the lane at once",
          not out.have_lane and out.yaw_rate == 0.0)

    # Commands stay inside their limits for an extreme lane bearing.
    _, out = run([math.radians(60.0)] * 40, look=math.radians(40.0))
    check("yaw within limit", abs(out.yaw_rate) <= cfg.max_yaw_rate + 1e-9,
          f"{out.yaw_rate:+.2f} vs {cfg.max_yaw_rate}")
    check("roll within limit", abs(out.tilt_right) <= cfg.max_lat + 1e-9,
          f"{out.tilt_right:+.2f} vs {cfg.max_lat}")
    check("forward drive eased when badly misaligned",
          out.tilt_fwd < 0.5 * cfg.cruise_tilt,
          f"fwd={out.tilt_fwd:.3f} vs cruise {cfg.cruise_tilt}")

    # Closed loop: a lane offset should be driven to zero, not oscillate.
    st = CN.CorridorState()
    lane = math.radians(25.0)
    peak_after_settle = 0.0
    for i in range(300):
        out = CN.step(st, cfg, i * 0.02, lane, 0.0, 1.0)
        # Kinematic stand-in: yawing right reduces a right-hand lane bearing.
        lane -= out.yaw_rate * 0.02
        if i > 150:
            peak_after_settle = max(peak_after_settle, abs(lane))
    check("lane offset converges", peak_after_settle < math.radians(3.0),
          f"settled within {math.degrees(peak_after_settle):.2f} deg")

    print()
    if _fails:
        print(f"RESULT: FAIL — {', '.join(_fails)}")
        return 1
    print("RESULT: PASS — corridor steering law behaves")
    return 0


if __name__ == "__main__":
    sys.exit(main())
