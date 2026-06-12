"""AIGP-sim adapter: planner → SET_ATTITUDE_TARGET (quaternion + thrust).

The official sim's attitude stabilizer plays the role flight_stack.py plays
in Elodin: we send a target attitude + collective thrust, it tracks it.

Frames: planner works in (x=north, y=east, alt=-down). Yaw is held at 0
(facing north); tilt_x maps to pitch (nose down = accelerate north),
tilt_y to roll (right = accelerate east).

Phases:
    PRE-RACE   level attitude, zero thrust (any motion before race_started
               is an early-start DQ; see JOURNAL 2026-06-06).
    CALIBRATE  first ~1.2 s after race start: hold level at a fixed test
               thrust, measure vertical acceleration, solve
               hover = T_test * g / (accel_up + g). Sim-version-proof.
    FLY        planner efforts → attitude + thrust toward the active gate.

Protocol quirks handled: sim disarms at race transition (re-arm loop),
race_started means sim time >= scheduled start (mavlink_rx handles).
"""

import math
import os
import time

from mavlink_tx import send_arm, send_attitude_rates
from planner import PlannerConfig, PlannerState, plan
from state import SharedState


CONTROL_HZ = 100
REARM_PERIOD_S = 0.5
LOG_PERIOD_S = 1.0

# Staged bring-up: hover -> waypoint -> race. Set via env var, e.g.
#   $env:MISSION="hover"; python main.py
MISSION = os.environ.get("MISSION", "hover").lower()
WAYPOINT_OFFSET = (-10.0, 0.0)   # 10 m south of the pad, same altitude

G = 9.81

# Calibration phase
CAL_THRUST = 0.35      # safely above any plausible hover; drone climbs gently
CAL_DURATION_S = 0.9
CAL_SKIP_S = 0.3       # ignore the first samples (arming/thrust spool)
HOVER_MIN, HOVER_MAX = 0.12, 0.8   # sanity clamp; run 4's trim hit a 0.05 floor and the drone was powerless

# Post-calibration settle: trim hover against vertical speed (integrator —
# the slope fit lands above true hover because drag confounds it; run 3
# "settled" at -7.5 m/s on timeout and handed the planner a climbing drone).
SETTLE_VD_MPS = 0.8
SETTLE_MAX_S = 8.0
SETTLE_TRIM_GAIN = 0.012   # hover -= gain * (-vd) * dt
SETTLE_DAMP = 0.4          # vertical effort = clamp(damp * vd, ...)

# Command shaping: a ballistic, tumbling drone came from near-zero thrust +
# rapidly alternating ±20° targets (run 3 flew inverted, att roll ~180°).
EFFORT_FLOOR = -0.6        # vertical effort floor → thrust ≥ ~0.5 * hover
ATT_SLEW_RAD_S = math.radians(90)   # max attitude-target change rate

# Attitude → body-rate P loop. Quaternion attitude mode IGNORES our thrust
# field (runs 1-4: vd unresponsive to thrust in both directions); body-rates
# mode demonstrably honors it (2026-06-08 diagnostic). So we close the
# attitude loop ourselves — same approach as flight_stack.py in Elodin.
ATT_P = 5.0                    # rad/s of rate per rad of attitude error
MAX_RATE_RAD_S = 3.0
YAW_P = 2.0

# Rate-sign auto-detection: run 5's roll axis hit positive feedback (±150°
# in under a second), so the sim's body-rate sign convention differs from
# ours on at least one axis. Pulse each axis briefly and measure which way
# the attitude actually moves. Convention-proof.
SIGN_PULSE_RATE = 0.6      # rad/s
SIGN_PULSE_S = 0.35
SIGN_MIN_DELTA_RAD = 0.02  # below this, keep default sign

# Tilt-response matrix: the sim's attitude telemetry is mirrored on some
# axes, offset on others (level pad reads pitch +18°), and yaw drifts after
# the sign pulses — per-axis sign flips can't untangle that (runs 6-7 each
# fixed one axis and revealed another). Instead, measure the full 2x2 map
# from attitude deflection (around the resting attitude) to world-frame
# acceleration, and invert it. Mirrors, rotations, and offsets all reduce
# to numbers in the matrix.
TILT_TEST_RAD = math.radians(12)
TILT_TEST_S = 0.8
M_DET_MIN = 0.5            # (m/s²/rad)² — below this the matrix is garbage

# Flight mapping
MAX_TILT_RAD = math.radians(20)
VERT_AUTH_FRAC = 0.8   # thrust = hover * (1 + frac * vertical_effort)
THRUST_MIN, THRUST_MAX = 0.02, 0.9

# Planner gains for this authority:
#   horizontal accel ≈ g·tan(20°) ≈ 3.6 m/s² at |tilt|=1  → ωn≈1.0, ζ≈1
#   vertical accel  ≈ g·VERT_AUTH ≈ 7.8 m/s² at |vert|=1  → ωn≈1.2, ζ≈1
PLANNER_CFG = PlannerConfig(
    # Overdamped vs Elodin: authority here is ~3.5 m/s² and run 8 overshot
    # gate 0 by 22 m carrying self-test lateral error into the approach.
    kp_x=0.20, kd_x=0.80,
    kp_y=0.20, kd_y=0.80,
    kp_alt=0.18, kd_alt=0.31, ki_alt=0.005, i_clamp=0.06,
    takeoff_clear_m=1.0,
    takeoff_goal_band_m=1.0,
    takeoff_climb_rate=1.0,
    takeoff_vz_gain=0.35,
    pass_through_m=2.5,
)


class Controller:
    def __init__(self, mavlink_conn, shared: SharedState, system_boot_ms: int):
        self.mavlink_conn = mavlink_conn
        self.shared = shared
        self.system_boot_ms = system_boot_ms
        self.planner_state = PlannerState()
        # LOCAL_POSITION_NED's origin IS the start pad: anchor the course
        # direction for gate 0 there, not wherever the self-tests drifted us.
        self.planner_state.spawn = (0.0, 0.0, 0.0)
        self.hover_thrust = None        # set by calibration
        self._cal_t0 = None
        self._cal_samples = []          # (t, vd)
        self._settle_t0 = None
        self._settle_last_t = None
        self._settled = False
        self._last_roll_cmd = 0.0
        self._last_pitch_cmd = 0.0
        self._last_cmd_t = None
        # Rate-sign detection state
        self.rate_sign = [1.0, 1.0, 1.0]       # roll, pitch, yaw multipliers
        self._sign_axis = 0                     # which axis is being tested
        self._sign_t0 = None
        self._sign_att0 = None
        self._signs_done = False
        self._direct_rates = None               # when set, update() sends these raw
        # Tilt-response matrix state
        self.resp = None                        # 2x2: d(accel_n, accel_e)/d(pitch, roll)
        self.resp_inv = None
        self.yaw_ref = None                     # yaw pinned for the whole flight
        self._map_phase = "pitch"               # "pitch" → "roll" → done
        self._map_t0 = None
        self._map_v0 = None
        self._map_cols = {}
        self._maps_done = False
        self._last_arm_t = 0.0
        self._last_log_t = 0.0
        self._logged_gates = False

    # ── main loop ─────────────────────────────────────────────────────
    def arm(self):
        send_arm(self.mavlink_conn)
        self._last_arm_t = time.time()

    def update(self):
        self._maybe_rearm()
        roll, pitch, yaw, thrust = self._step()

        ds = self.shared.drone_state
        if self._direct_rates is not None:
            # Sign-detection pulse in progress: send raw rates.
            roll_rate, pitch_rate, yaw_rate = self._direct_rates
        elif ds is not None:
            # Attitude P → body rates with detected signs (the sim honors
            # rates+thrust; it ignores the thrust field in quaternion mode).
            roll_rate = self.rate_sign[0] * self._clamp_rate(ATT_P * self._wrap(roll - ds.roll_rad))
            pitch_rate = self.rate_sign[1] * self._clamp_rate(ATT_P * self._wrap(pitch - ds.pitch_rad))
            yaw_rate = self.rate_sign[2] * self._clamp_rate(YAW_P * self._wrap(yaw - ds.yaw_rad))
        else:
            roll_rate = pitch_rate = yaw_rate = 0.0

        send_attitude_rates(
            self.mavlink_conn, self.system_boot_ms,
            roll_rate, pitch_rate, yaw_rate, thrust,
        )
        self._maybe_log(roll, pitch, thrust)
        time.sleep(1.0 / CONTROL_HZ)

    @staticmethod
    def _wrap(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    @staticmethod
    def _clamp_rate(r: float) -> float:
        return max(-MAX_RATE_RAD_S, min(MAX_RATE_RAD_S, r))

    def _maybe_rearm(self):
        hb = self.shared.heartbeat
        if hb is None or hb.armed:
            return
        now = time.time()
        if now - self._last_arm_t < REARM_PERIOD_S:
            return
        send_arm(self.mavlink_conn)
        self._last_arm_t = now

    # ── control phases ───────────────────────────────────────────────
    def _step(self):
        """Return (roll, pitch, yaw, thrust)."""
        ds = self.shared.drone_state
        td = self.shared.track_data
        rs = self.shared.race_status

        # PRE-RACE: hold level at current yaw, zero thrust. No motion before the gun.
        if ds is None or td is None or rs is None or not rs.race_started:
            return 0.0, 0.0, (ds.yaw_rad if ds else 0.0), 0.0

        # CALIBRATE: fixed thrust, ZERO body rates (open loop — rate signs
        # are unknown until the SIGN phase; closing the attitude loop here
        # tumbled the drone in run 12).
        if self.hover_thrust is None:
            return self._calibrate_step(ds)

        # SIGN DETECTION: pulse each body axis, watch the attitude response.
        if not self._signs_done:
            return self._sign_step(ds)

        # SETTLE: level out (signs now known), trim the hover estimate.
        if not self._settled:
            now = time.time()
            if self._settle_t0 is None:
                self._settle_t0 = now
                self._settle_last_t = now
            dt = max(1e-3, now - self._settle_last_t)
            self._settle_last_t = now

            # Climbing (vd < 0) → fitted hover is high → trim it down.
            # Rate at vd = -7: about -0.08/s on the hover estimate.
            self.hover_thrust += SETTLE_TRIM_GAIN * ds.vd_mps * dt
            self.hover_thrust = max(HOVER_MIN, min(HOVER_MAX, self.hover_thrust))

            if abs(ds.vd_mps) < SETTLE_VD_MPS or now - self._settle_t0 > SETTLE_MAX_S:
                self._settled = True
                # The drone is demonstrably flying; never let the planner's
                # takeoff latch suppress horizontal control (run 5).
                self.planner_state.airborne = True
                print(f"  [CAL] settled: vd={ds.vd_mps:+.2f}, trimmed hover={self.hover_thrust:.3f}", flush=True)
            else:
                effort = max(EFFORT_FLOOR, min(1.0, SETTLE_DAMP * ds.vd_mps))
                thr = self.hover_thrust * (1.0 + VERT_AUTH_FRAC * effort)
                return 0.0, 0.0, ds.yaw_rad, max(THRUST_MIN, min(THRUST_MAX, thr))

        # TILT-MAP DETECTION: hold small tilts, watch which way velocity builds.
        if not self._maps_done:
            return self._map_step(ds)

        # MISSION
        if MISSION == "race":
            return self._fly_step(ds, td, rs)
        return self._mission_step(ds)

    def _calibrate_step(self, ds):
        now = time.time()
        if self._cal_t0 is None:
            self._cal_t0 = now
            self._cal_att0 = (ds.roll_rad, ds.pitch_rad)
            self._direct_rates = (0.0, 0.0, 0.0)   # open loop: zero rates
            print(f"  [CAL] start: thrust={CAL_THRUST} for {CAL_DURATION_S}s (zero rates)", flush=True)

        elapsed = now - self._cal_t0
        if elapsed > CAL_SKIP_S:
            self._cal_samples.append((now, ds.vd_mps))

        if elapsed >= CAL_DURATION_S:
            self.hover_thrust = self._fit_hover()
            self._direct_rates = None
            print(f"  [CAL] hover_thrust = {self.hover_thrust:.3f}", flush=True)

        # Hold CURRENT yaw — drone spawns facing ~south; commanding yaw 0
        # made the stabilizer whip through a 180° flip at the gun (run 1).
        # Hold LEVEL: reported attitude is physical attitude; the pad's +18
        # reading is a real ramp tilt, not an offset (run 11).
        return 0.0, 0.0, ds.yaw_rad, CAL_THRUST

    def _fit_hover(self):
        n = len(self._cal_samples)
        if n < 8:
            print("  [CAL] too few samples; fallback hover=0.27", flush=True)
            return 0.27
        ts = [s[0] for s in self._cal_samples]
        vds = [s[1] for s in self._cal_samples]
        tm = sum(ts) / n
        vm = sum(vds) / n
        num = sum((t - tm) * (v - vm) for t, v in zip(ts, vds))
        den = sum((t - tm) ** 2 for t in ts)
        accel_down = num / den if den > 0 else 0.0
        accel_up = -accel_down
        hover = CAL_THRUST * G / max(accel_up + G, 1.0)
        return max(HOVER_MIN, min(HOVER_MAX, hover))

    def _sign_step(self, ds):
        """Pulse one axis at a time; set rate_sign from the attitude response."""
        now = time.time()
        att = (ds.roll_rad, ds.pitch_rad, ds.yaw_rad)

        if self._sign_t0 is None:
            self._sign_t0 = now
            self._sign_att0 = att
            pulse = [0.0, 0.0, 0.0]
            pulse[self._sign_axis] = SIGN_PULSE_RATE
            self._direct_rates = tuple(pulse)
            print(f"  [SIGN] pulsing axis {self._sign_axis}", flush=True)

        if now - self._sign_t0 >= SIGN_PULSE_S:
            delta = self._wrap(att[self._sign_axis] - self._sign_att0[self._sign_axis])
            if abs(delta) >= SIGN_MIN_DELTA_RAD:
                self.rate_sign[self._sign_axis] = 1.0 if delta > 0 else -1.0
            print(f"  [SIGN] axis {self._sign_axis}: delta={math.degrees(delta):+.1f} deg "
                  f"-> sign {self.rate_sign[self._sign_axis]:+.0f}", flush=True)
            self._sign_axis += 1
            self._sign_t0 = None
            if self._sign_axis >= 3:
                self._signs_done = True
                self._direct_rates = None
                print(f"  [SIGN] done: {self.rate_sign}", flush=True)

        # Hover thrust during the test; attitude args unused while pulsing.
        return 0.0, 0.0, ds.yaw_rad, self.hover_thrust

    def _map_step(self, ds):
        """Measure the world-accel response to each attitude axis.

        Deflections are around the resting attitude (which is physically
        level even though it reads pitch +18). Yaw is pinned to yaw_ref
        during and after the tests so the measured matrix stays valid.
        """
        now = time.time()
        if self.yaw_ref is None:
            self.yaw_ref = ds.yaw_rad
            print(f"  [MAP] yaw pinned at {math.degrees(self.yaw_ref):+.0f} deg", flush=True)

        r0, p0 = 0.0, 0.0

        if self._map_t0 is None:
            self._map_t0 = now
            self._map_v0 = (ds.vn_mps, ds.ve_mps)
            print(f"  [MAP] testing {self._map_phase} deflection", flush=True)

        if now - self._map_t0 >= TILT_TEST_S:
            dt = now - self._map_t0
            col = (
                (ds.vn_mps - self._map_v0[0]) / dt / TILT_TEST_RAD,
                (ds.ve_mps - self._map_v0[1]) / dt / TILT_TEST_RAD,
            )
            self._map_cols[self._map_phase] = col
            print(f"  [MAP] {self._map_phase}: accel response "
                  f"(n={col[0]:+.2f}, e={col[1]:+.2f}) m/s^2 per rad", flush=True)
            if self._map_phase == "pitch":
                self._map_phase = "roll"
                self._map_t0 = None
            else:
                self._finish_matrix()
            return r0, p0, self.yaw_ref, self.hover_thrust

        if self._map_phase == "pitch":
            return r0, p0 + TILT_TEST_RAD, self.yaw_ref, self.hover_thrust
        return r0 + TILT_TEST_RAD, p0, self.yaw_ref, self.hover_thrust

    def _finish_matrix(self):
        a, c = self._map_cols["pitch"]   # accel_n, accel_e per rad of pitch
        b, d = self._map_cols["roll"]    # accel_n, accel_e per rad of roll
        det = a * d - b * c
        self._maps_done = True
        if abs(det) < M_DET_MIN:
            # Degenerate measurement; fall back to identity-ish guess.
            print(f"  [MAP] WARNING det={det:.2f} too small; using fallback", flush=True)
            self.resp_inv = ((1.0 / 5.0, 0.0), (0.0, 1.0 / 5.0))
        else:
            self.resp_inv = ((d / det, -b / det), (-c / det, a / det))
        print(f"  [MAP] matrix done: det={det:+.2f}", flush=True)

    def _mission_step(self, ds):
        """Stage A/B missions: hold position, or fly to one waypoint and hold."""
        if not hasattr(self, "_hold_target") or self._hold_target is None:
            alt = -ds.down_m
            if MISSION == "waypoint":
                tgt = (ds.north_m + WAYPOINT_OFFSET[0], ds.east_m + WAYPOINT_OFFSET[1], alt)
            else:
                tgt = (ds.north_m, ds.east_m, alt)
            self._hold_target = tgt
            print(f"  [MISSION] {MISSION}: target=({tgt[0]:.1f},{tgt[1]:.1f},alt {tgt[2]:.1f})", flush=True)

        gx, gy, galt = self._hold_target
        # Same PD law and gains as the planner, applied to a fixed target.
        cfg = PLANNER_CFG
        tilt_x = max(-1.0, min(1.0, cfg.kp_x * (gx - ds.north_m) - cfg.kd_x * ds.vn_mps))
        tilt_y = max(-1.0, min(1.0, cfg.kp_y * (gy - ds.east_m) - cfg.kd_y * ds.ve_mps))
        alt = -ds.down_m
        v_alt = -ds.vd_mps
        vertical = max(EFFORT_FLOOR, min(1.0, cfg.kp_alt * (galt - alt) - cfg.kd_alt * v_alt))

        self._last_goal = self._hold_target
        self._last_tilt = (tilt_x, tilt_y)
        return self._efforts_to_attitude(tilt_x, tilt_y, vertical)

    def _fly_step(self, ds, td, rs):
        # NED → planner frame (x=north, y=east, alt up)
        pos = (ds.north_m, ds.east_m, -ds.down_m)
        vel = (ds.vn_mps, ds.ve_mps, -ds.vd_mps)
        gates = tuple((g.north_m, g.east_m, -g.down_m) for g in td.gates)

        idx = rs.active_gate_index
        if rs.race_finished:
            idx = len(gates)  # out of range → planner aims past last gate

        out = plan(
            self.planner_state, PLANNER_CFG,
            time.time(), pos, vel, gates, idx,
        )
        # For telemetry: the goal the planner is actually steering toward.
        from planner import select_goal
        self._last_goal = select_goal(
            gates, idx, self.planner_state.spawn, PLANNER_CFG.pass_through_m, pos=pos
        )
        self._last_tilt = (out.tilt_x, out.tilt_y)

        return self._efforts_to_attitude(out.tilt_x, out.tilt_y, out.vertical)

    def _efforts_to_attitude(self, tilt_x, tilt_y, vertical):
        # Efforts → attitude via the measured response matrix; yaw pinned.
        accel_auth = 9.81 * math.tan(MAX_TILT_RAD)
        acc_n = accel_auth * tilt_x
        acc_e = accel_auth * tilt_y
        dp = self.resp_inv[0][0] * acc_n + self.resp_inv[0][1] * acc_e
        dr = self.resp_inv[1][0] * acc_n + self.resp_inv[1][1] * acc_e
        pitch = max(-MAX_TILT_RAD, min(MAX_TILT_RAD, dp))
        roll = max(-MAX_TILT_RAD, min(MAX_TILT_RAD, dr))
        yaw = self.yaw_ref

        # Slew-limit attitude targets (run 3 tumbled on square waves).
        now = time.time()
        dt = max(1e-3, now - self._last_cmd_t) if self._last_cmd_t else 1.0 / CONTROL_HZ
        self._last_cmd_t = now
        max_step = ATT_SLEW_RAD_S * dt
        roll = self._last_roll_cmd + max(-max_step, min(max_step, roll - self._last_roll_cmd))
        pitch = self._last_pitch_cmd + max(-max_step, min(max_step, pitch - self._last_pitch_cmd))
        self._last_roll_cmd = roll
        self._last_pitch_cmd = pitch

        vertical = max(EFFORT_FLOOR, min(1.0, vertical))
        thrust = self.hover_thrust * (1.0 + VERT_AUTH_FRAC * vertical)
        thrust = max(THRUST_MIN, min(THRUST_MAX, thrust))

        return roll, pitch, yaw, thrust

    # ── telemetry ─────────────────────────────────────────────────────
    def _maybe_log(self, roll, pitch, thrust):
        now = time.time()
        if now - self._last_log_t < LOG_PERIOD_S:
            return
        self._last_log_t = now

        ds = self.shared.drone_state
        td = self.shared.track_data
        rs = self.shared.race_status
        hb = self.shared.heartbeat

        if td and not self._logged_gates:
            print(f"  Track has {len(td.gates)} gates:", flush=True)
            for g in td.gates:
                print(f"    gate {g.gate_id}: ({g.north_m:.1f}, {g.east_m:.1f}, {g.down_m:.1f})", flush=True)
            self._logged_gates = True

        ds_str = (
            f"pos=({ds.north_m:.1f},{ds.east_m:.1f},{ds.down_m:.1f}) "
            f"vd={ds.vd_mps:+.1f} att=({math.degrees(ds.roll_rad):+.0f},{math.degrees(ds.pitch_rad):+.0f})"
            if ds else "pos=None"
        )
        rs_str = (
            f"active={rs.active_gate_index} started={rs.race_started} fin={rs.race_finished}"
            if rs else "race=None"
        )
        hb_str = f"armed={hb.armed}" if hb else "hb=None"
        hov = f"{self.hover_thrust:.3f}" if self.hover_thrust is not None else "uncal"
        att = f"r={math.degrees(roll):+.0f} p={math.degrees(pitch):+.0f} thr={thrust:.2f} hov={hov}"
        goal = getattr(self, "_last_goal", None)
        tilt = getattr(self, "_last_tilt", None)
        extra = ""
        if goal is not None and tilt is not None:
            extra = (
                f"  goal=({goal[0]:.1f},{goal[1]:.1f},{goal[2]:.1f})"
                f" tilt=({tilt[0]:+.2f},{tilt[1]:+.2f})"
            )
        print(f"  {ds_str}  {rs_str}  {hb_str}  {att}{extra}", flush=True)
