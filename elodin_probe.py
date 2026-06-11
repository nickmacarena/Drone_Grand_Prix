"""Calibration probe for the Elodin/Betaflight ANGLE-mode plant.

Run instead of the race solver:

    tools/run_elodin.sh probe /tmp/elodin_probe.log 120 elodin_probe

Phases:
    A  takeoff  — fixed high throttle until z > 4 m
    B  trim     — PI on vertical speed finds the true hover PWM
    C  v-steps  — throttle steps ± around hover; measure vertical accel/PWM
    D  tilt     — pitch steps; measure horizontal accel/PWM and achieved angle
    E  report   — print [PROBE] lines with fitted constants, cut throttle

All output goes to stdout with a [PROBE] prefix (lands in the run log).
"""

import math

from solver.api import RCCommand, SensorUpdate  # elodin repo package

T_DISARMED_END = 0.50
T_ARM_IDLE_END = 0.75

# Racing-quad preset: T/W ~8.8, theoretical hover stick ~1114. Keep takeoff
# gentle or the drone catapults and the trim phase can't recover.
TAKEOFF_PWM = 1250
TAKEOFF_ALT_M = 3.0

TRIM_S = 8.0       # phase B duration
TRIM_KP = 25.0     # PWM per m/s of vz error (gentle — avoid sat-tumbling)
TRIM_KI = 50.0     # PWM per m/s per s
TRIM_START = 1120.0
TRIM_MIN, TRIM_MAX = 1050, 1400
TRIM_SETTLED_VZ = 0.6   # only sample hover PWM when |vz| below this
VSTEP_PWM = 80
VSTEP_S = 1.5
SETTLE_S = 1.0

# ANGLE-mode verdict test: pulse pitch stick, release, watch attitude.
# ANGLE mode → attitude snaps back level after release.
# Acro       → attitude stays tipped (rate cmd 0 just holds current angle).
PULSE_PWM = 120
PULSE_S = 0.8
RELEASE_S = 1.5


class Probe:
    def __init__(self):
        self.phase = "takeoff"
        self.phase_t0 = None
        self.trim_i = TRIM_START
        self.hover_pwm = None
        self.last_t = None
        self.samples = []          # (t, vz, vx, pitch_rad) within a step
        self.results = {}
        self.step_queue = []
        self.current_step = None
        self.reported = False

    def log(self, msg):
        print(f"[PROBE] {msg}", flush=True)

    def pitch_rad(self, u: SensorUpdate) -> float:
        qx, qy, qz, qw = (float(u.world_pos[i]) for i in range(4))
        return math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))

    def fit_slope(self, ts, vs):
        """Least-squares slope of v over t."""
        n = len(ts)
        if n < 5:
            return 0.0
        tm = sum(ts) / n
        vm = sum(vs) / n
        num = sum((t - tm) * (v - vm) for t, v in zip(ts, vs))
        den = sum((t - tm) ** 2 for t in ts)
        return num / den if den > 0 else 0.0

    def update(self, u: SensorUpdate) -> RCCommand:
        t = u.t
        z = float(u.world_pos[6])
        vx = float(u.world_vel[3])
        vz = float(u.world_vel[5])
        dt = 0.001 if self.last_t is None else max(1e-4, t - self.last_t)
        self.last_t = t

        thr, pitch = 1000, 1500

        if self.phase == "takeoff":
            thr = TAKEOFF_PWM
            if z > TAKEOFF_ALT_M:
                self.phase, self.phase_t0 = "trim", t
                self.log(f"t={t:.2f} takeoff done z={z:.2f}, trimming hover...")

        elif self.phase == "trim":
            self.trim_i += TRIM_KI * (0.0 - vz) * dt
            self.trim_i = max(TRIM_MIN, min(TRIM_MAX, self.trim_i))
            thr = self.trim_i + TRIM_KP * (0.0 - vz)
            if t - self.phase_t0 > TRIM_S * 0.5 and abs(vz) < TRIM_SETTLED_VZ:
                self.samples.append((t, thr))
            if t - self.phase_t0 > TRIM_S:
                if not self.samples:
                    self.log(f"t={t:.2f} trim never settled (vz={vz:+.2f}); using trim_i={self.trim_i:.0f}")
                    self.samples = [(t, self.trim_i)]
                pwms = [p for _, p in self.samples]
                self.hover_pwm = sum(pwms) / len(pwms)
                pitch_now = math.degrees(self.pitch_rad(u))
                self.results["HOVER_PWM"] = self.hover_pwm
                self.log(f"t={t:.2f} HOVER_PWM = {self.hover_pwm:.0f} (pitch at trim end: {pitch_now:+.1f} deg)")
                self.samples = []
                self.step_queue = [
                    ("vstep", +VSTEP_PWM), ("settle", 0),
                    ("vstep", -VSTEP_PWM), ("settle", 0),
                    ("pulse", +PULSE_PWM), ("release", 0),
                ]
                self.phase = "steps"
                self.current_step = None

        elif self.phase == "steps":
            if self.current_step is None:
                if not self.step_queue:
                    self.phase = "report"
                    return RCCommand(arm=1000, throttle=1000, aux2=1000)
                self.current_step = self.step_queue.pop(0)
                self.phase_t0 = t
                self.samples = []

            kind, mag = self.current_step
            thr = self.hover_pwm
            dur = {"settle": SETTLE_S, "vstep": VSTEP_S,
                   "pulse": PULSE_S, "release": RELEASE_S}[kind]

            if kind == "vstep":
                thr = self.hover_pwm + mag
                self.samples.append((t, vz, vx, self.pitch_rad(u)))
            elif kind == "pulse":
                pitch = 1500 + mag
                thr = self.hover_pwm + TRIM_KP * (0.0 - vz)
                self.samples.append((t, vz, vx, self.pitch_rad(u)))
            elif kind == "release":
                # sticks centered; just hold gentle vz trim and observe attitude
                thr = self.hover_pwm + TRIM_KP * (0.0 - vz)
                self.samples.append((t, vz, vx, self.pitch_rad(u)))
            else:  # settle
                thr = self.hover_pwm + TRIM_KP * (0.0 - vz)

            if t - self.phase_t0 > dur:
                if kind == "vstep" and self.samples:
                    ts = [s[0] for s in self.samples]
                    vzs = [s[1] for s in self.samples]
                    a = self.fit_slope(ts, vzs)
                    pitch_avg = math.degrees(sum(s[3] for s in self.samples) / len(self.samples))
                    self.results[f"VACCEL@{mag:+d}"] = a
                    self.log(f"vstep {mag:+d} PWM -> vert accel {a:+.2f} m/s^2 (pitch {pitch_avg:+.1f} deg)")
                elif kind == "pulse" and self.samples:
                    end_pitch = math.degrees(self.samples[-1][3])
                    ts = [s[0] for s in self.samples]
                    ps = [s[3] for s in self.samples]
                    rate = math.degrees(self.fit_slope(ts, ps))
                    self.results["PULSE_END_PITCH_DEG"] = end_pitch
                    self.results["PITCH_RATE_DEG_S"] = rate
                    self.log(f"pulse {mag:+d} PWM for {PULSE_S}s -> pitch {end_pitch:+.1f} deg (rate {rate:+.1f} deg/s)")
                elif kind == "release" and self.samples:
                    end_pitch = math.degrees(self.samples[-1][3])
                    self.results["RELEASE_END_PITCH_DEG"] = end_pitch
                    verdict = "ANGLE (self-levels)" if abs(end_pitch) < 6.0 else "ACRO (stays tipped)"
                    self.results["MODE_VERDICT"] = verdict
                    self.log(f"release: pitch settled at {end_pitch:+.1f} deg -> {verdict}")
                self.current_step = None

        elif self.phase == "report":
            if not self.reported:
                self.reported = True
                r = self.results
                self.log("========== PROBE RESULTS ==========")
                for k, v in r.items():
                    self.log(f"  {k} = {v:.3f}" if isinstance(v, float) else f"  {k} = {v}")
                up = r.get("VACCEL@+80", 0.0)
                dn = r.get("VACCEL@-80", 0.0)
                if up or dn:
                    self.log(f"  VERT_ACCEL_PER_PWM ~ {(up - dn) / (2 * VSTEP_PWM):.4f} m/s^2/PWM")
                ax_f = r.get("XACCEL@-150", 0.0)
                ax_b = r.get("XACCEL@+150", 0.0)
                if ax_f or ax_b:
                    self.log(f"  HORIZ_ACCEL_PER_PWM ~ {(ax_f - ax_b) / (2 * TILT_PWM):.4f} m/s^2/PWM")
            return RCCommand(arm=1000, throttle=1000, aux2=1000)

        thr = int(round(max(1000, min(1700, thr))))
        pitch = int(round(max(1000, min(2000, pitch))))
        return RCCommand(arm=1800, throttle=thr, pitch=pitch, aux2=1000)


_probe = Probe()


def autopilot(update: SensorUpdate) -> RCCommand:
    t = update.t
    if t < T_DISARMED_END:
        return RCCommand(arm=1000, throttle=1000, aux2=1000)
    if t < T_ARM_IDLE_END:
        return RCCommand(arm=1800, throttle=1000, aux2=1000)
    return _probe.update(update)
