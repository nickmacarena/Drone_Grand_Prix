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
from bearing import AIGP_CAM
from mavlink_tx import send_arm, send_attitude_rates
from state import SharedState

import estimator
import servo

MISSION = os.environ.get("MISSION", "hover").lower()

CONTROL_HZ = 100
REARM_PERIOD_S = 0.5
LOG_PERIOD_S = 1.0
G = 9.80665

# NED: down is +z, so "up" is -z.
UP_NED = (0.0, 0.0, -1.0)
# Servo says "turn right"; in FRD +body-z already turns right. (Elodin FLU
# needs -1; see tests/test_servo.py.)
YAW_SIGN = +1.0

# ── Calibration ──────────────────────────────────────────────────────
CAL_THRUST = 0.35
CAL_DURATION_S = 1.2
CAL_SKIP_S = 0.3          # ignore spool-up
HOVER_MIN, HOVER_MAX = 0.10, 0.80
SETTLE_S = 2.0            # let the calibration climb bleed off

# ── Body-rate sign detection ─────────────────────────────────────────
# VQ1 measured this sim's rate conventions as [roll +1, pitch -1, yaw -1] and
# re-derived them every run. The rev-3390 rad/s type_mask bit does NOT
# normalise them: assuming it did put the pitch loop in positive feedback and
# the drone tumbled seconds after the attitude loop engaged (run 1, 2026-07-27
# attitude went -17.8 -> +73.5 -> -137.8). Detect, never assume. The estimator
# is the observer now, since ATTITUDE telemetry is gone.
SIGN_PULSE_RATE = 0.6     # rad/s
SIGN_PULSE_S = 0.4
SIGN_MIN_DELTA_RAD = 0.02

# ── Attitude control (same shape as the VQ1 stack, IMU-driven) ───────
ATT_P = 5.0               # rad/s of body rate per rad of attitude error
MAX_RATE = 3.0
VERT_AUTH_FRAC = 0.8      # thrust = hover * (1 + frac * vertical_effort)
THRUST_MIN, THRUST_MAX = 0.02, 0.90
MAX_TILT_RAD = math.radians(20)


class ControllerVQ2:
    def __init__(self, mavlink_conn, shared: SharedState, system_boot_ms: int):
        self.conn = mavlink_conn
        self.shared = shared
        self.boot_ms = system_boot_ms

        self.est = estimator.EstimatorState()
        self.est_cfg = estimator.EstimatorConfig(up_world=UP_NED)
        self.servo_state = servo.ServoState()
        self.servo_cfg = servo.ServoConfig()

        self.hover = None
        self.rate_sign = [1.0, 1.0, 1.0]
        self._sign_axis = 0
        self._sign_t0 = None
        self._sign_att0 = None
        self._signs_done = False
        self._direct_rates = None
        self._cal_t0 = None
        self._cal_samples = []
        self._settle_t0 = None
        self._settled = False
        self._last_arm_t = 0.0
        self._last_log_t = 0.0
        self._hold_yaw = None

    # ── main loop ────────────────────────────────────────────────────
    def arm(self):
        send_arm(self.conn)
        self._last_arm_t = time.time()

    def update(self):
        self._maybe_rearm()
        imu = self.shared.imu
        t = time.time()

        if imu is not None:
            estimator.update(self.est, self.est_cfg, t,
                             (imu.gx, imu.gy, imu.gz), (imu.ax, imu.ay, imu.az))

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
            yaw_rate_out = self.rate_sign[2] * YAW_SIGN * yaw_rate_c
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
            if t - self._settle_t0 >= SETTLE_S:
                self._settled = True
                print(f"  [CAL] settled; hover={self.hover:.3f}", flush=True)
            return 0.0, 0.0, 0.0, self.hover

        if not self._signs_done:
            return self._sign_step(t)

        if MISSION == "race":
            return self._race_step(t)
        return 0.0, 0.0, 0.0, self.hover     # hover / frames: hold level

    def _sign_step(self, t):
        """Pulse each body axis; the estimator reports which way it moved."""
        att = self._est_euler()
        if self._sign_t0 is None:
            self._sign_t0 = t
            self._sign_att0 = att
            pulse = [0.0, 0.0, 0.0]
            pulse[self._sign_axis] = SIGN_PULSE_RATE
            self._direct_rates = tuple(pulse)
            print(f"  [SIGN] pulsing axis {self._sign_axis}", flush=True)

        if t - self._sign_t0 >= SIGN_PULSE_S:
            delta = _wrap(att[self._sign_axis] - self._sign_att0[self._sign_axis])
            if abs(delta) >= SIGN_MIN_DELTA_RAD:
                self.rate_sign[self._sign_axis] = 1.0 if delta > 0 else -1.0
            print(f"  [SIGN] axis {self._sign_axis}: "
                  f"delta={math.degrees(delta):+.1f} deg -> "
                  f"{self.rate_sign[self._sign_axis]:+.0f}", flush=True)
            self._sign_axis += 1
            self._sign_t0 = None
            if self._sign_axis >= 3:
                self._signs_done = True
                self._direct_rates = None
                print(f"  [SIGN] done: {self.rate_sign}", flush=True)

        return 0.0, 0.0, 0.0, self.hover

    def _calibrate(self, t, imu):
        if self._cal_t0 is None:
            self._cal_t0 = t
            print(f"  [CAL] thrust={CAL_THRUST} for {CAL_DURATION_S}s "
                  f"(vertical accel from IMU — VQ2 has no velocity)", flush=True)

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

    def _vertical_accel(self, imu) -> float:
        """Kinematic acceleration along world UP, from the IMU alone.

        The accelerometer reads specific force f = a - g in the body frame.
        Rotating into the world and adding gravity back recovers a; its
        component along UP is what hover calibration needs.
        """
        f_world = quat_rotate(self.est.q, (imu.ax, imu.ay, imu.az))
        a_world = (f_world[0], f_world[1], f_world[2] + G)   # NED: +z is down
        return -(a_world[2])                                  # up-positive

    def _race_step(self, t):
        """Full visual servoing. Needs a detector to fill shared.latest_frame
        with a gate sighting — Stage 3b. Until then, hold level rather than
        fly blind."""
        target = None   # TODO(Stage 3b): detector -> bearing -> Target
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
        print(f"  [VQ2] {MISSION} {att} a_up={a_up:+5.2f} thr={thrust:.2f} "
              f"hov={hov} armed={hb.armed if hb else '?'} "
              f"gate={rs.active_gate_index if rs else '?'} "
              f"started={rs.race_started if rs else '?'} frame={frames}",
              flush=True)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a
