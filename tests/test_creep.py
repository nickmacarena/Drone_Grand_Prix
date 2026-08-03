"""Gate-at-a-time state machine (creep). Pure Python, no simulator."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import creep

_fails = []


def check(name, ok, detail=""):
    print(f"    {'ok  ' if ok else 'FAIL'}  {name}" + (f"  — {detail}" if detail else ""))
    if not ok:
        _fails.append(name)


def drive(st, cfg, seq, dt=0.05, t0=0.0):
    """Feed a sequence of (az, el, conf, closure, lane, gate_idx)."""
    out = None
    t = t0
    for s in seq:
        out = creep.step(st, cfg, t, *s)
        t += dt
    return out, t


def main():
    print("=== creep state machine ===\n")
    cfg = creep.CreepConfig()

    # SEEK sweeps and does not translate — nothing to fly into while looking.
    st = creep.CreepState()
    out, _ = drive(st, cfg, [(None, None, 0.0, 0.0, None, 0)] * 20)
    check("SEEK sweeps without translating",
          st.phase == creep.SEEK and abs(out.yaw_rate) > 0.1
          and out.tilt_fwd == 0.0, f"yaw={out.yaw_rate:+.2f} fwd={out.tilt_fwd:.2f}")

    # The sweep must reverse, never circle: facing back down the course offers
    # the detector a gate we have already passed.
    st2 = creep.CreepState()
    peak = 0.0
    t = 0.0
    for _ in range(600):
        creep.step(st2, cfg, t, None, None, 0.0, 0.0, None, 0)
        peak = max(peak, abs(st2.sweep_pos))
        t += 0.05
    check("SEEK sweep is bounded", peak <= cfg.seek_sweep_rad + 0.05,
          f"peak {math.degrees(peak):.0f} deg")

    # A ceiling gate must not start an approach.
    st3 = creep.CreepState()
    drive(st3, cfg, [(0.0, math.radians(45.0), 1.0, 0.0, None, 0)] * 10)
    check("SEEK ignores a gate far above the course", st3.phase == creep.SEEK)

    # A real gate promotes to ALIGN.
    st4 = creep.CreepState()
    drive(st4, cfg, [(0.2, math.radians(8.0), 1.0, 3.0, None, 0)] * 4)
    check("a valid gate enters ALIGN", st4.phase == creep.ALIGN)

    # ALIGN brakes while still closing, and never drives forward.
    out, _ = drive(st4, cfg, [(0.2, math.radians(8.0), 1.0, 3.0, None, 0)] * 6)
    check("ALIGN brakes while closing", out.tilt_fwd < 0.0,
          f"fwd={out.tilt_fwd:+.2f} at closure 3.0 m/s")
    check("ALIGN steers with yaw and thrust", abs(out.yaw_rate) > 0.05
          and abs(out.vertical) > 0.01,
          f"yaw={out.yaw_rate:+.2f} vert={out.vertical:+.2f}")

    # Only a stopped, centred aircraft commits.
    st5 = creep.CreepState()
    drive(st5, cfg, [(0.01, 0.01, 1.0, 0.1, None, 0)] * 60)
    check("centred and stopped commits to TRANSIT", st5.phase == creep.TRANSIT)

    st6 = creep.CreepState()
    drive(st6, cfg, [(0.4, 0.01, 1.0, 0.1, None, 0)] * 60)
    check("badly aimed does NOT commit", st6.phase == creep.ALIGN,
          "az 23 deg off")

    st7 = creep.CreepState()
    drive(st7, cfg, [(0.01, 0.01, 1.0, 4.0, None, 0)] * 60)
    check("still moving does NOT commit", st7.phase == creep.ALIGN,
          "closure 4.0 m/s")

    # TRANSIT ignores vision entirely — the load-bearing property.
    st8 = creep.CreepState()
    drive(st8, cfg, [(0.01, 0.01, 1.0, 0.1, None, 0)] * 60)
    out, _ = drive(st8, cfg, [(0.9, math.radians(50.0), 1.0, 0.0, None, 0)] * 4)
    check("TRANSIT ignores vision", out.ignore_vision
          and out.tilt_right == 0.0 and out.yaw_rate == 0.0,
          f"az 52 deg / el 50 deg present, yaw={out.yaw_rate:+.2f}")

    # Motion is a pulse: forward, then reverse, so speed cannot accumulate.
    fwds = []
    st9 = creep.CreepState()
    drive(st9, cfg, [(0.01, 0.01, 1.0, 0.1, None, 0)] * 60)
    t = 3.0
    for _ in range(80):
        o = creep.step(st9, cfg, t, None, None, 0.0, 0.0, None, 0)
        fwds.append(o.tilt_fwd)
        t += 0.05
    check("TRANSIT pulses forward and back",
          max(fwds) > 0.1 and min(fwds) < -0.1,
          f"range {min(fwds):+.2f}..{max(fwds):+.2f}")
    # The two impulses must MATCH, or the aircraft drifts steadily in whichever
    # direction wins. A mean of -0.045 reversed it 75 m down the Elodin course:
    # a systematically signed mean is a direction, not "close to zero".
    net = sum(fwds) / len(fwds)
    check("pulse impulses cancel", abs(net) < 0.02, f"mean tilt {net:+.4f}")
    imp_on = cfg.transit_pulse_on_s * cfg.transit_tilt
    imp_off = cfg.transit_pulse_off_s * cfg.transit_brake_tilt
    check("forward and braking impulses are equal by construction",
          abs(imp_on - imp_off) < 1e-9,
          f"{imp_on:.4f} vs {imp_off:.4f}")

    # Passing the gate advances to PIVOT.
    out, t = drive(st9, cfg, [(None, None, 0.0, 0.0, None, 1)] * 2, t0=t)
    check("gate index advance enters PIVOT", st9.phase == creep.PIVOT)

    # PIVOT clears the structure before turning, then yaws onto the lane.
    out = creep.step(st9, cfg, t, None, None, 0.0, 0.0, math.radians(40.0), 1)
    check("PIVOT holds still briefly first", out.yaw_rate == 0.0,
          "inside pivot_min_s")
    out, t = drive(st9, cfg, [(None, None, 0.0, 0.0, math.radians(40.0), 1)] * 12,
                   t0=t + 0.5)
    check("PIVOT turns toward the lane", out.yaw_rate > 0.1,
          f"lane +40 deg -> yaw {out.yaw_rate:+.2f}")
    check("PIVOT does not translate", out.tilt_fwd == 0.0)

    # Lane centred -> back to SEEK for the next gate.
    _, t = drive(st9, cfg, [(None, None, 0.0, 0.0, 0.0, 1)] * 4, t0=t)
    check("centred lane returns to SEEK", st9.phase == creep.SEEK)

    print()
    if _fails:
        print(f"RESULT: FAIL — {', '.join(_fails)}")
        return 1
    print("RESULT: PASS — creep sequences align, commit, pass, pivot")
    return 0


if __name__ == "__main__":
    sys.exit(main())
