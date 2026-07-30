"""VQ2 controller for the OFFICIAL sim. Staged bring-up, IMU-only state.

    $env:MISSION="hover"   # calibrate + hold level on IMU attitude alone
    $env:MISSION="frames"  # same, plus dump camera frames (set FRAME_DIR)
    $env:MISSION="race"    # full servo — needs a gate detector (Stage 3b)

VQ2 (sim v1.0.3391) disables ATTITUDE, LOCAL_POSITION_NED and ODOMETRY and
nulls track data, so everything here runs on HIGHRES_IMU plus the camera.
The estimator, bearing and servo modules are the same ones validated in
Elodin — only this adapter is sim-specific.

Two VQ2-forced changes worth knowing:

*Calibration.* VQ1 measured hover thrust by differentiating vertical
VELOCITY, which no longer exists. The IMU gives vertical acceleration
directly, which is both simpler and less noisy: rotate the accelerometer's
specific force into the world frame, add gravity back, and you have kinematic
acceleration. Hover is where that is zero, so with a test thrust T and the
measured upward acceleration a:  hover = T * g / (a + g).

*Frames.* Official sim is FRD body / NED world, so up_world = (0, 0, -1) and
the servo's "turn right" maps to +body-z (the opposite of Elodin's FLU).
Both are validated in tests/test_servo.py.
"""

import math
import os
import time

from attitude import quat_rotate
from bearing import AIGP_CAM, stabilized_bearing
from detector import detect_gate
from mavlink_tx import send_arm, send_attitude_rates
from state import SharedState

import estimator
import servo

MISSION = os.environ.get("MISSION", "hover").lower()

# ── Measured plant constants ─────────────────────────────────────────
# In-race calibration did its job: it DISCOVERED these, and they have now
# reproduced across four official-sim runs (hover 0.244 / 0.251 / 0.238 /
# 0.251; gyro sign test [+1,+1,+1] both times it ran uncorrupted).
#
# Continuing to re-derive them costs ~7 s of bring-up during which we command
# zero roll/pitch and have NO horizontal control — starting on a 17.8 deg ramp
# roughly 12 m from gate 0, the drone simply drifts into it. That, not the
# control law, is what ended runs 4 and 5.
#
# So: fly on the measured constants by default, and keep the calibration path
# behind CALIBRATE=1 for re-measurement if the sim is updated.
CALIBRATE = os.environ.get("CALIBRATE", "0") == "1"
HOVER_DEFAULT = 0.248          # mean of four measurements
SIGNS_DEFAULT = [1.0, 1.0, 1.0]

CONTROL_HZ = 100
REARM_PERIOD_S = 0.5
LOG_PERIOD_S = 1.0
G = 9.80665

# NED: down is +z, so "up" is -z.
UP_NED = (0.0, 0.0, -1.0)
# Servo says "turn right". Textbook FRD says +body-z does that, which is why
# this was +1 for runs 1-10 — and it is empirically WRONG for this sim.
#
# Run 10, post-gate-0: we commanded right, the yaw estimate advanced 58 deg, and
# the target moved from 32 deg right (u=521) to 44 deg right (u=630). A real
# right turn sweeps a right-hand target toward centre and out the left side; it
# can only move further right if the aircraft turned LEFT. Nick saw exactly that
# ("turn and face left"). Runs 6-10 all show the same monotone yaw runaway: the
# azimuth loop was in POSITIVE feedback the whole time.
#
# The in-race gyro test cannot catch this. It checks that the commanded rate and
# the gyro agree in sign, which they do; it never checks whether the resulting
# rotation moves the IMAGE the expected way.
YAW_SIGN = -1.0

# ...and because that was a one-line inference costing a five-minute run to
# test, the aircraft now checks it in flight. Commanding yaw toward a tracked
# gate must make its azimuth shrink. If commanded yaw and measured d(az)/dt
# agree in sign often enough, steering is inverted and we flip once.
# Thresholds live in servo.PolarityCheck.

# MISSION=yawtest: isolate rotation from translation (see _yaw_test_step).
YAW_AUTOFLIP = os.environ.get("YAW_AUTOFLIP", "0") == "1"

YAWTEST_RATE = 0.35        # rad/s — slow, so the gate stays in frame
YAWTEST_S = 6.0            # rotate for this long
YAWTEST_SETTLE_S = 3.0     # hold still first, to fix the starting bearing

# ── Calibration ──────────────────────────────────────────────────────
CAL_THRUST = 0.30         # gentler than VQ1's 0.35: every m/s of climb
CAL_DURATION_S = 1.0      # bought here has to be paid back before flying
CAL_SKIP_S = 0.3          # ignore spool-up
HOVER_MIN, HOVER_MAX = 0.10, 0.80
SETTLE_S = 2.5            # ACTIVELY arrest the calibration climb

# Vertical velocity, integrated from the IMU. VQ2 removed LOCAL_POSITION_NED,
# so nothing reports how fast we are climbing — and holding hover thrust means
# zero ACCELERATION, not zero velocity. Run 3 (2026-07-28) calibrated at 0.35
# against a 0.238 hover, left the settle phase still climbing at ~5.5 m/s,
# coasted ~20 m up inside a roofed hangar and hit the ceiling (a_up spike of
# +13.9 m/s^2, then tumbling). Integration drifts over minutes but is accurate
# over the seconds needed to stop a climb; the leak keeps bias bounded, and
# this is only ever used as a DAMPING term, never as absolute altitude.
VZ_LEAK_PER_S = 0.15      # bleed the estimate toward zero (bias guard)
VZ_DAMP = 0.06            # thrust per (m/s) of unwanted vertical speed
VZ_SETTLE_TOL = 0.5       # m/s; settle finishes early once below this

# vz_est is ONLY trustworthy as a transient, measured from a known-zero start.
# A steady descent produces zero net acceleration, so the accelerometer reads
# exactly 1 g — indistinguishable from a hover. Integration therefore cannot
# see constant-velocity motion at all, and the leak turns any leftover into a
# standing bias. Elodin ground truth caught this cleanly (2026-07-28): true
# vz -2.3 m/s while the estimate read +0.6, so the damper trimmed thrust BELOW
# hover and drove the very descent it could not observe.
#
# So: damp only while settling the calibration climb, then stop and let VISION
# own altitude — the gate's elevation in frame is an absolute reference, which
# is exactly what an IMU cannot provide.
VZ_DAMP_AFTER_SETTLE = False

# ── Body-rate sign detection ─────────────────────────────────────────
# VQ1 measured this sim's rate conventions as [roll +1, pitch -1, yaw -1] and
# re-derived them every run. The rev-3390 rad/s type_mask bit does NOT
# normalise them: assuming it did put the pitch loop in positive feedback and
# the drone tumbled seconds after the attitude loop engaged (run 1, 2026-07-27
# attitude went -17.8 -> +73.5 -> -137.8). Detect, never assume. The estimator
# is the observer now, since ATTITUDE telemetry is gone.
# Measured from the GYRO, not from integrated attitude. Run 2 (2026-07-28)
# detected signs by pulsing an axis and watching the estimator's Euler angles:
# with no levelling between pulses the roll pulse left the drone at 39 deg,
# the pitch pulse then acted on an already-banked airframe, and by the yaw
# pulse it was inverted at 120 deg — the measurement destroyed what it was
# measuring, and axis 2 returned garbage (+97.6 deg).
#
# Comparing commanded rate against the gyro is direct, instantaneous, and
# needs no attitude integration, so the pulse can be small and brief enough
# to barely disturb the aircraft. It is also exactly the mapping we want: the
# estimator integrates this same gyro, so its attitude — and therefore the
# attitude loop's output — lives in the gyro's convention.
SIGN_PULSE_RATE = 0.5     # rad/s
SIGN_PULSE_S = 0.25
SIGN_SETTLE_S = 0.35      # zero rates between axes, so each starts from calm
SIGN_MIN_GYRO = 0.05      # rad/s; below this the axis did not respond

# After detection, level out before flying: the pulses leave some attitude.
LEVEL_S = 1.5

# ── Attitude control (same shape as the VQ1 stack, IMU-driven) ───────
ATT_P = 5.0               # rad/s of body rate per rad of attitude error
MAX_RATE = 3.0
VERT_AUTH_FRAC = 0.8      # thrust = hover * (1 + frac * vertical_effort)
THRUST_MIN, THRUST_MAX = 0.02, 0.90
MAX_TILT_RAD = math.radians(20)


class ControllerVQ2:
    def __init__(self, mavlink_conn, shared: SharedState, system_boot_ms: int,
                 up_world=UP_NED, yaw_sign=YAW_SIGN, cam=AIGP_CAM):
        self.conn = mavlink_conn
        self.shared = shared
        self.boot_ms = system_boot_ms
        # Frame conventions are injected so this exact bring-up sequence can be
        # debugged in Elodin (FLU/ENU) at ~90 s per run instead of 5 minutes.
        self.up_world = up_world
        self.yaw_sign = yaw_sign
        self.cam = cam

        self.est = estimator.EstimatorState()
        self.est_cfg = estimator.EstimatorConfig(up_world=up_world)
        self.servo_state = servo.ServoState()
        self._last_gate_index = None
        self._yaw_chk = servo.PolarityCheck()
        self._yawtest_t0 = None
        self._yawtest_u0 = None
        self._yawtest_yaw0 = 0.0
        self._yawtest_last_u = None
        self._yawtest_last_yaw = 0.0
        self._yawtest_done = False
        self.servo_cfg = servo.ServoConfig()

        self.hover = None if CALIBRATE else HOVER_DEFAULT
        self.vz_est = 0.0         # m/s, +up. IMU-integrated; damping only.
        self._vz_last_t = None
        self.rate_sign = list(SIGNS_DEFAULT)
        self._sign_axis = 0
        self._sign_t0 = None
        self._sign_att0 = None
        self._signs_done = not CALIBRATE
        self._sign_samples = []
        self._sign_resting = False
        self._level_t0 = None
        self._leveled = not CALIBRATE
        self._direct_rates = None
        self._cal_t0 = None
        self._cal_samples = []
        self._settle_t0 = None
        self._settled = not CALIBRATE
        self._leveled_init = not CALIBRATE
        self._last_arm_t = 0.0
        self._last_log_t = 0.0
        self._hold_yaw = None
        self._last_frame_seen = None
        self._last_obs = None
        self._detect_err_logged = False

    # ── main loop ────────────────────────────────────────────────────
    def arm(self):
        send_arm(self.conn)
        self._last_arm_t = time.time()
        if not CALIBRATE:
            print(f"  [PLANT] using measured constants: hover={HOVER_DEFAULT} "
                  f"signs={SIGNS_DEFAULT} (CALIBRATE=1 to re-measure)",
                  flush=True)

    def update(self):
        self._maybe_rearm()
        imu = self.shared.imu
        t = time.time()

        if imu is not None:
            estimator.update(self.est, self.est_cfg, t,
                             (imu.gx, imu.gy, imu.gz), (imu.ax, imu.ay, imu.az))

        if imu is not None and self.est.initialized:
            self._integrate_vz(t, imu)

        roll_c, pitch_c, yaw_rate_c, thrust = self._step(t, imu)

        # Attitude P -> body rates. Roll/pitch are absolute targets; yaw is a
        # rate (heading is unobservable from gravity and we never need it).
        if self._direct_rates is not None:
            roll_rate, pitch_rate, yaw_rate_out = self._direct_rates
        elif self.est.initialized:
            r, p, _ = self._est_euler()
            roll_rate = self.rate_sign[0] * _clamp(
                ATT_P * _wrap(roll_c - r), -MAX_RATE, MAX_RATE)
            pitch_rate = self.rate_sign[1] * _clamp(
                ATT_P * _wrap(pitch_c - p), -MAX_RATE, MAX_RATE)
            self._check_yaw_polarity(yaw_rate_c)
            yaw_rate_out = self.rate_sign[2] * self.yaw_sign * yaw_rate_c
        else:
            roll_rate = pitch_rate = yaw_rate_out = 0.0

        send_attitude_rates(self.conn, self.boot_ms,
                            roll_rate, pitch_rate, yaw_rate_out, thrust)
        self._maybe_log(t, imu, thrust)
        time.sleep(1.0 / CONTROL_HZ)

    def _maybe_rearm(self):
        hb = self.shared.heartbeat
        if hb is None or hb.armed:
            return
        now = time.time()
        if now - self._last_arm_t >= REARM_PERIOD_S:
            send_arm(self.conn)
            self._last_arm_t = now

    # ── phases ───────────────────────────────────────────────────────
    def _step(self, t, imu):
        rs = self.shared.race_status

        # Nothing before the gun: any motion is an early-start DQ (VQ1 lesson).
        if imu is None or rs is None or not rs.race_started or not self.est.initialized:
            return 0.0, 0.0, 0.0, 0.0

        if self.hover is None:
            return self._calibrate(t, imu)

        if not self._settled:
            if self._settle_t0 is None:
                self._settle_t0 = t
            done = (t - self._settle_t0 >= SETTLE_S
                    or abs(self.vz_est) < VZ_SETTLE_TOL)
            if done:
                self._settled = True
                print(f"  [CAL] settled; hover={self.hover:.3f} "
                      f"vz={self.vz_est:+.2f} m/s (zeroing: residual becomes "
                      f"bias otherwise)", flush=True)
                self.vz_est = 0.0
            return 0.0, 0.0, 0.0, self._damped_hover()

        if not self._signs_done:
            return self._sign_step(t, imu)

        # Recover whatever attitude the pulses left behind before flying.
        if not self._leveled:
            if self._level_t0 is None:
                self._level_t0 = t
            if t - self._level_t0 >= LEVEL_S:
                self._leveled = True
                print(f"  [LEVEL] recovered; vz={self.vz_est:+.2f} m/s; "
                      f"starting mission", flush=True)
            return 0.0, 0.0, 0.0, self._damped_hover()

        if MISSION == "race":
            return self._race_step(t)
        if MISSION == "yawtest":
            return self._yaw_test_step(t)
        return 0.0, 0.0, 0.0, self._damped_hover()   # hover/frames: hold still

    def _yaw_test_step(self, t):
        """Measure the yaw convention directly, with translation removed.

        Six runs were spent inferring this from race logs and every inference was
        confounded the same way. At the ranges where it mattered (rng 1-4 m,
        gate 30-40 deg off axis, ~8 m/s), the bearing rate caused by TRANSLATION
        is v*sin(az)/r ~ 1.3 rad/s — triple the commanded yaw rate. Azimuth was
        moving because we were flying past the gate, not because we were
        rotating, so it carried almost no information about the sign. Runs 10 and
        11 flew opposite signs and BOTH showed azimuth increasing, which is the
        proof that the measurement was meaningless.

        Rotation and translation separate only when translation is zero. So:
        hover in place, no forward tilt, no roll, and rotate slowly. Then du/dt
        is purely rotational and the convention is unambiguous.

        Run with MISSION=yawtest. Takes about 12 s and does not need a gate to be
        passed, only seen.
        """
        if self._yawtest_t0 is None:
            self._yawtest_t0 = t
            print(f"  [YAWTEST] hovering, commanding yaw {YAWTEST_RATE:+.2f} "
                  f"rad/s for {YAWTEST_S}s. No forward tilt: any u movement is "
                  f"rotation only.", flush=True)
        elapsed = t - self._yawtest_t0

        # Observe the gate, but do not act on it.
        frame = self.shared.latest_frame
        if frame is not None and frame is not self._last_frame_seen:
            self._last_frame_seen = frame
            try:
                self._last_obs = detect_gate(frame, self.cam)
            except Exception:
                self._last_obs = None
        obs = self._last_obs
        _, _, yaw_est = self._est_euler()

        if elapsed < YAWTEST_SETTLE_S:
            if obs is not None:                  # record the starting point
                self._yawtest_u0 = obs.u
                self._yawtest_yaw0 = yaw_est
            return 0.0, 0.0, 0.0, self._damped_hover()

        if elapsed < YAWTEST_SETTLE_S + YAWTEST_S:
            if obs is not None and self._yawtest_u0 is not None:
                self._yawtest_last_u = obs.u
                self._yawtest_last_yaw = yaw_est
            # Pure rotation: hover thrust, level, commanded yaw only.
            return 0.0, 0.0, YAWTEST_RATE, self._damped_hover()

        if not self._yawtest_done:
            self._yawtest_done = True
            self._report_yaw_test()
        return 0.0, 0.0, 0.0, self._damped_hover()

    def _report_yaw_test(self):
        """State the convention plainly, with the numbers behind it."""
        u0, u1 = self._yawtest_u0, self._yawtest_last_u
        if u0 is None or u1 is None:
            print("  [YAWTEST] no gate held in view — point the drone at a gate "
                  "and rerun", flush=True)
            return
        du = u1 - u0
        dyaw = math.degrees(self._yawtest_last_yaw - self._yawtest_yaw0)
        # Commanding +yaw_rate through the current sign chain produced this
        # rotation. Correct convention: a RIGHT turn sweeps the scene LEFT, so
        # the gate's u must DECREASE.
        applied = self.rate_sign[2] * self.yaw_sign * YAWTEST_RATE
        want = "decrease" if applied > 0 else "increase"
        got = "decrease" if du < 0 else "increase"
        ok = (want == got)
        print(f"  [YAWTEST] commanded {YAWTEST_RATE:+.2f} rad/s "
              f"(applied {applied:+.2f} after rate_sign*yaw_sign)", flush=True)
        print(f"  [YAWTEST] gate u {u0:.1f} -> {u1:.1f}  (du={du:+.1f} px), "
              f"gyro yaw moved {dyaw:+.1f} deg", flush=True)
        print(f"  [YAWTEST] expected u to {want}, saw it {got} -> "
              f"{'CORRECT' if ok else 'INVERTED'}: "
              f"YAW_SIGN should be {self.yaw_sign if ok else -self.yaw_sign:+.0f}",
              flush=True)

    def _sign_step(self, t, imu):
        """Pulse an axis and compare the GYRO against the command.

        Alternates pulse / rest so each axis is measured from a calm start and
        the aircraft is never left rotating.
        """
        if self._sign_t0 is None:
            self._sign_t0 = t
            self._sign_samples = []
            self._sign_resting = False
            pulse = [0.0, 0.0, 0.0]
            pulse[self._sign_axis] = SIGN_PULSE_RATE
            self._direct_rates = tuple(pulse)
            print(f"  [SIGN] pulsing axis {self._sign_axis}", flush=True)

        elapsed = t - self._sign_t0

        if not self._sign_resting:
            if imu is not None:
                self._sign_samples.append(
                    (imu.gx, imu.gy, imu.gz)[self._sign_axis])
            if elapsed >= SIGN_PULSE_S:
                n = len(self._sign_samples)
                measured = sum(self._sign_samples) / n if n else 0.0
                if abs(measured) >= SIGN_MIN_GYRO:
                    self.rate_sign[self._sign_axis] = 1.0 if measured > 0 else -1.0
                    verdict = f"{self.rate_sign[self._sign_axis]:+.0f}"
                else:
                    verdict = "no response, keeping default"
                print(f"  [SIGN] axis {self._sign_axis}: commanded "
                      f"{SIGN_PULSE_RATE:+.2f} -> gyro {measured:+.3f} rad/s "
                      f"({n} samples) -> {verdict}", flush=True)
                # Rest: zero rates, let the axis stop before the next test.
                self._sign_resting = True
                self._direct_rates = (0.0, 0.0, 0.0)
            return 0.0, 0.0, 0.0, self._damped_hover()

        if elapsed >= SIGN_PULSE_S + SIGN_SETTLE_S:
            self._sign_axis += 1
            self._sign_t0 = None
            if self._sign_axis >= 3:
                self._signs_done = True
                self._direct_rates = None
                print(f"  [SIGN] done: {self.rate_sign}", flush=True)
        return 0.0, 0.0, 0.0, self._damped_hover()

    def _calibrate(self, t, imu):
        if self._cal_t0 is None:
            self._cal_t0 = t
            self.vz_est = 0.0          # at rest on the pad: a known zero
            print(f"  [CAL] thrust={CAL_THRUST} for {CAL_DURATION_S}s "
                  f"(vertical accel from IMU - VQ2 has no velocity)", flush=True)

        elapsed = t - self._cal_t0
        if elapsed > CAL_SKIP_S:
            self._cal_samples.append(self._vertical_accel(imu))

        if elapsed >= CAL_DURATION_S:
            n = len(self._cal_samples)
            a_up = sum(self._cal_samples) / n if n else 0.0
            self.hover = _clamp(CAL_THRUST * G / max(a_up + G, 1.0),
                                HOVER_MIN, HOVER_MAX)
            print(f"  [CAL] a_up={a_up:+.2f} m/s^2 over {n} samples "
                  f"-> hover={self.hover:.3f}", flush=True)
        return 0.0, 0.0, 0.0, CAL_THRUST

    def _integrate_vz(self, t, imu):
        if self._vz_last_t is None:
            self._vz_last_t = t
            return
        dt = t - self._vz_last_t
        self._vz_last_t = t
        if dt <= 0.0 or dt > 0.2:
            return
        self.vz_est += self._vertical_accel(imu) * dt
        self.vz_est *= max(0.0, 1.0 - VZ_LEAK_PER_S * dt)

    def _damped_hover(self) -> float:
        """Hover thrust, minus whatever it takes to kill vertical speed.

        Only valid while `vz_est` is a fresh transient (see VZ_DAMP_AFTER_SETTLE).
        """
        if self._settled and not VZ_DAMP_AFTER_SETTLE:
            return _clamp(self.hover, THRUST_MIN, THRUST_MAX)
        return _clamp(self.hover - VZ_DAMP * self.vz_est, THRUST_MIN, THRUST_MAX)

    def _vertical_accel(self, imu) -> float:
        """Kinematic acceleration along world UP, from the IMU alone.

        The accelerometer reads specific force f = a - g in the body frame.
        Rotating into the world and adding gravity back recovers a; its
        component along UP is what hover calibration needs.
        """
        f_world = quat_rotate(self.est.q, (imu.ax, imu.ay, imu.az))
        g_world = (-G * self.up_world[0], -G * self.up_world[1], -G * self.up_world[2])
        a_world = (f_world[0] + g_world[0], f_world[1] + g_world[1],
                   f_world[2] + g_world[2])
        return (a_world[0] * self.up_world[0] + a_world[1] * self.up_world[1]
                + a_world[2] * self.up_world[2])

    def _check_yaw_polarity(self, yaw_rate_c: float) -> None:
        """Flip yaw_sign if flight data says steering is inverted (see servo)."""
        if not self.servo_state.have_fix:
            return
        verdict = servo.check_polarity(self._yaw_chk, yaw_rate_c,
                                       self.servo_state.az_rate,
                                       self.servo_state.rng_f)
        if verdict == 0:
            return
        n, wrong = self._yaw_chk.total, self._yaw_chk.wrong
        if verdict > 0:
            print(f"  [YAW] polarity confirmed ({n - wrong}/{n} samples)",
                  flush=True)
        elif YAW_AUTOFLIP:
            self.yaw_sign = -self.yaw_sign
            print(f"  [YAW] steering inverted ({wrong}/{n} samples) -> "
                  f"yaw_sign={self.yaw_sign:+.0f}", flush=True)
        else:
            # Run 11 flipped mid-flight on confounded close-range data and made
            # things worse. Report, do not act, until MISSION=yawtest has
            # settled the convention from a hover.
            print(f"  [YAW] steering looks INVERTED ({wrong}/{n} samples); "
                  f"not flipping (YAW_AUTOFLIP=1 to enable)", flush=True)

    def _race_step(self, t):
        """Full visual servoing: detect the gate, de-rotate the sighting into
        the level frame with the IMU attitude, servo onto it."""
        # The sim advancing active_gate_index is the only unambiguous progress
        # signal VQ2 gives us, and run 7 ignored it: after clearing gate 0 the
        # servo kept steering on a track that described a gate now behind it,
        # turned ~147 deg away, and never recovered.
        rs = self.shared.race_status
        if rs is not None and rs.active_gate_index != self._last_gate_index:
            if self._last_gate_index is not None:
                print(f"  [GATE] passed {self._last_gate_index} -> "
                      f"{rs.active_gate_index}; re-centring search", flush=True)
                servo.gate_passed(self.servo_state, t, self.servo_cfg)
                self._last_obs = None
            self._last_gate_index = rs.active_gate_index

        target = None
        frame = self.shared.latest_frame
        if frame is not None and frame is not self._last_frame_seen:
            self._last_frame_seen = frame
            try:
                obs = detect_gate(frame, self.cam)
            except Exception as e:                    # never let vision kill the loop
                obs = None
                if not self._detect_err_logged:
                    self._detect_err_logged = True
                    print(f"  [DETECT] error: {e}", flush=True)
            self._last_obs = obs
        obs = self._last_obs
        if obs is not None:
            az, el = stabilized_bearing(self.cam, obs, self.est.q, self.up_world)
            target = servo.Target(az=az, el=el,
                                 confidence=obs.confidence,
                                 range_m=obs.range_m)
        out = servo.step(self.servo_state, self.servo_cfg, t, target)
        if not out.have_target:
            return 0.0, 0.0, 0.0, self.hover
        pitch = -MAX_TILT_RAD * out.tilt_fwd     # nose down = negative FRD pitch
        roll = MAX_TILT_RAD * out.tilt_right
        thrust = _clamp(self.hover * (1.0 + VERT_AUTH_FRAC * out.vertical),
                        THRUST_MIN, THRUST_MAX)
        return roll, pitch, out.yaw_rate, thrust

    # ── helpers ──────────────────────────────────────────────────────
    def _est_euler(self):
        from attitude import quat_to_euler_zyx
        return quat_to_euler_zyx(self.est.q)

    def _maybe_log(self, t, imu, thrust):
        if t - self._last_log_t < LOG_PERIOD_S:
            return
        self._last_log_t = t
        rs = self.shared.race_status
        hb = self.shared.heartbeat
        if self.est.initialized:
            r, p, y = self._est_euler()
            att = (f"est_att=({math.degrees(r):+5.1f},{math.degrees(p):+5.1f},"
                   f"{math.degrees(y):+6.1f})")
        else:
            att = "est_att=uncal"
        hov = f"{self.hover:.3f}" if self.hover else "uncal"
        a_up = self._vertical_accel(imu) if (imu and self.est.initialized) else 0.0
        frames = self.shared.latest_frame is not None
        o = self._last_obs
        det = (f"DET u={o.u:5.1f} v={o.v:5.1f} rng={o.range_m:5.1f} conf={o.confidence:.2f}"
               if o is not None else "no-gate")
        print(f"  [VQ2] {MISSION} {att} a_up={a_up:+5.2f} thr={thrust:.2f} "
              f"vz={self.vz_est:+5.2f} hov={hov} armed={hb.armed if hb else '?'} "
              f"gate={rs.active_gate_index if rs else '?'} "
              f"started={rs.race_started if rs else '?'} frame={frames} {det}",
              flush=True)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a
