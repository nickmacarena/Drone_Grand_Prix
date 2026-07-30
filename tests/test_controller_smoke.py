"""Drive ControllerVQ2 end to end with a fake sim. Catches broken builds.

Twice now a controller with a MISSING METHOD reached the official simulator —
_calibrate/_vertical_angle once, and _sign_step/_calibrate/_integrate_vz/
_damped_hover/_vertical_accel/_check_yaw_polarity the second time, both from
index-based string surgery that swallowed the methods between two anchors. Each
cost a five-minute run to discover an AttributeError that any execution of the
control loop would have surfaced instantly.

Every other test here exercises servo/estimator/bearing in isolation. Nothing
ran the controller itself, so nothing checked that it is even callable. This
does: it feeds synthetic IMU and frames through update() for every mission and
asserts the loop runs and the commands stay in range.

Pure Python — numpy/PIL/pymavlink are stubbed, since none of their behaviour is
under test here.
"""

import math
import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


class _Any(types.ModuleType):
    """Stub module whose attributes are ints, so bit-ORing masks works."""
    def __getattr__(self, name):
        if name.isupper() or "TYPEMASK" in name:
            v = 1
        else:
            v = _Any(name)
        setattr(self, name, v)
        return v


for _m in ("numpy", "PIL", "PIL.Image", "pymavlink", "pymavlink.mavutil"):
    sys.modules.setdefault(_m, _Any(_m))
sys.modules["pymavlink"].mavutil = sys.modules["pymavlink.mavutil"]

import controller_vq2 as C          # noqa: E402
from state import HeartbeatStatus, ImuSample, RaceStatus, SharedState  # noqa: E402

_fails = []


def check(name, ok, detail=""):
    print(f"    {'ok  ' if ok else 'FAIL'}  {name}" + (f"  — {detail}" if detail else ""))
    if not ok:
        _fails.append(name)


class FakeConn:
    target_system = 1
    target_component = 1

    def __init__(self):
        self.sent = []
        self.mav = self

    def set_attitude_target_send(self, *a):
        self.sent.append(a)

    def command_long_send(self, *a):
        pass


def fly(mission, steps=1400, gate_visible=True, plant_standard=True,
        yaw_sign=None):
    """Run the control loop against a synthetic plant that actually RESPONDS.

    The commanded yaw rate is fed back as the gyro (perfect rate tracking) and a
    world-fixed gate is reprojected as the drone turns, so MISSION=yawtest
    produces a real verdict instead of merely executing. `plant_standard=False`
    inverts the gyro/image relationship, which is the case we cannot currently
    distinguish on the real sim.
    """
    C.MISSION = mission
    C.time.sleep = lambda _s: None          # no real-time pacing in tests
    clock = {"t": 1000.0}
    C.time.time = lambda: clock["t"]

    conn = FakeConn()
    shared = SharedState()
    shared.heartbeat = HeartbeatStatus(armed=True, base_mode=0, custom_mode=0,
                                       system_status=4)
    kw = {} if yaw_sign is None else {"yaw_sign": yaw_sign}
    ctl = C.ControllerVQ2(conn, shared, 0, **kw)

    from bearing import GateObservation

    thrusts = []
    yaw_world = 0.0          # integrated true heading of the fake drone
    gz = 0.0
    bearing0 = math.atan2((330.0 - 320.0) / 320.0, 1.0)
    for i in range(steps):
        dt = 1.0 / C.CONTROL_HZ
        clock["t"] += dt
        yaw_world += gz * dt
        # World-fixed gate reprojected as the drone rotates. Standard plant: a
        # right turn sweeps the scene left, so bearing decreases.
        bearing = bearing0 + (-yaw_world if plant_standard else yaw_world)
        u = 320.0 + 320.0 * math.tan(bearing)
        if gate_visible and 0.0 <= u < 640.0:
            C.detect_gate = lambda f, cam=None, _u=u: GateObservation(
                az=bearing, el=0.02, u=_u, v=170.0, width_px=90.0,
                confidence=1.0)
        else:
            C.detect_gate = lambda f, cam=None: None
        # Level-ish drone on a 17.8 deg nose-down ramp, as the sim reports.
        pitch = math.radians(-17.8) if i < 60 else 0.0
        shared.imu = ImuSample(
            t_us=int(clock["t"] * 1e6),
            ax=9.81 * math.sin(pitch), ay=0.0, az=-9.81 * math.cos(pitch),
            gx=0.0, gy=0.0, gz=gz)
        shared.race_status = RaceStatus(active_gate_index=0 if i < 400 else 1,
                                        race_started=True, race_finished=False,
                                        last_gate_race_time_s=0.0)
        shared.latest_frame = object()      # a new object each step -> "new frame"
        ctl.update()
        if conn.sent:
            thrusts.append(conn.sent[-1][8])
            gz = conn.sent[-1][7]          # perfect body-rate tracking
    return ctl, thrusts


def main():
    print("=== controller smoke (no simulator) ===\n")

    for mission in ("hover", "race", "yawtest"):
        try:
            ctl, thrusts = fly(mission)
            ok = len(thrusts) > 0
            detail = (f"{len(thrusts)} commands, thrust "
                      f"{min(thrusts):.2f}..{max(thrusts):.2f}" if ok else "no commands sent")
            check(f"MISSION={mission} runs", ok, detail)
            if ok:
                check(f"MISSION={mission} thrust in range",
                      all(0.0 <= x <= 1.0 for x in thrusts),
                      f"{min(thrusts):.2f}..{max(thrusts):.2f}")
        except Exception as e:
            check(f"MISSION={mission} runs", False, f"{type(e).__name__}: {e}")

    # The yawtest measurement must recover the plant's convention from a
    # responding plant — and must give the SAME answer whichever YAW_SIGN it
    # happens to be flying, since the slope is independent of it.
    import io, contextlib
    for plant_standard in (True, False):
        for start_sign in (+1.0, -1.0):
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                fly("yawtest", plant_standard=plant_standard,
                    yaw_sign=start_sign)
            out = buf.getvalue()
            expect = "STANDARD" if plant_standard else "INVERTED"
            want_sign = "+1" if plant_standard else "-1"
            got = expect in out
            said = f"should be {want_sign}" in out
            check(f"yawtest reads {expect.lower()} plant (flying "
                  f"{start_sign:+.0f})", got and said,
                  [l.strip() for l in out.splitlines()
                   if "convention is" in l or "INCONCLUSIVE" in l][:1])

    # Losing the gate must not break the loop either — that path is where the
    # search/sweep code lives and it only runs when nothing is visible.
    try:
        _, thrusts = fly("race", gate_visible=False)
        check("race with no gate visible runs", len(thrusts) > 0,
              f"{len(thrusts)} commands")
    except Exception as e:
        check("race with no gate visible runs", False, f"{type(e).__name__}: {e}")

    # Every method referenced on self must exist. Cheap guard against the exact
    # string-surgery accident that caused this file to be written.
    import re
    src = open(os.path.join(os.path.dirname(__file__), "..",
                            "controller_vq2.py")).read()
    referenced = set(re.findall(r"self\.(_\w+)\(", src))
    defined = set(re.findall(r"^    def (\w+)", src, re.M))
    missing = sorted(referenced - defined)
    check("no method is called but undefined", not missing,
          f"missing: {missing}" if missing else f"{len(referenced)} checked")

    print()
    if _fails:
        print(f"RESULT: FAIL — {', '.join(_fails)}")
        return 1
    print("RESULT: PASS — controller runs in every mission")
    return 0


if __name__ == "__main__":
    sys.exit(main())
