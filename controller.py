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
import time

from attitude import euler_to_quaternion
from mavlink_tx import send_arm, send_attitude_quaternion
from planner import PlannerConfig, PlannerState, plan
from state import SharedState


CONTROL_HZ = 100
REARM_PERIOD_S = 0.5
LOG_PERIOD_S = 1.0

G = 9.81

# Calibration phase
CAL_THRUST = 0.35      # safely above any plausible hover; drone climbs gently
CAL_DURATION_S = 0.9
CAL_SKIP_S = 0.3       # ignore the first samples (arming/thrust spool)
HOVER_MIN, HOVER_MAX = 0.05, 0.8   # sanity clamp on the fitted value

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

# Flight mapping
MAX_TILT_RAD = math.radians(20)
VERT_AUTH_FRAC = 0.8   # thrust = hover * (1 + frac * vertical_effort)
THRUST_MIN, THRUST_MAX = 0.02, 0.9

# Planner gains for this authority:
#   horizontal accel ≈ g·tan(20°) ≈ 3.6 m/s² at |tilt|=1  → ωn≈1.0, ζ≈1
#   vertical accel  ≈ g·VERT_AUTH ≈ 7.8 m/s² at |vert|=1  → ωn≈1.2, ζ≈1
PLANNER_CFG = PlannerConfig(
    kp_x=0.28, kd_x=0.56,
    kp_y=0.28, kd_y=0.56,
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
        self.hover_thrust = None        # set by calibration
        self._cal_t0 = None
        self._cal_samples = []          # (t, vd)
        self._settle_t0 = None
        self._settle_last_t = None
        self._settled = False
        self._last_roll_cmd = 0.0
        self._last_pitch_cmd = 0.0
        self._last_cmd_t = None
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
        q = euler_to_quaternion(roll, pitch, yaw)
        send_attitude_quaternion(self.mavlink_conn, self.system_boot_ms, q, thrust)
        self._maybe_log(roll, pitch, thrust)
        time.sleep(1.0 / CONTROL_HZ)

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

        # CALIBRATE: fixed thrust, level, fit hover from vertical accel.
        if self.hover_thrust is None:
            return self._calibrate_step(ds)

        # SETTLE: trim the hover estimate against vertical speed and bleed
        # off the calibration climb before handing to the planner.
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
                print(f"  [CAL] settled: vd={ds.vd_mps:+.2f}, trimmed hover={self.hover_thrust:.3f}", flush=True)
            else:
                effort = max(EFFORT_FLOOR, min(1.0, SETTLE_DAMP * ds.vd_mps))
                thr = self.hover_thrust * (1.0 + VERT_AUTH_FRAC * effort)
                return 0.0, 0.0, ds.yaw_rad, max(THRUST_MIN, min(THRUST_MAX, thr))

        # FLY
        return self._fly_step(ds, td, rs)

    def _calibrate_step(self, ds):
        now = time.time()
        if self._cal_t0 is None:
            self._cal_t0 = now
            print(f"  [CAL] start: thrust={CAL_THRUST} for {CAL_DURATION_S}s", flush=True)

        elapsed = now - self._cal_t0
        if elapsed > CAL_SKIP_S:
            self._cal_samples.append((now, ds.vd_mps))

        if elapsed >= CAL_DURATION_S:
            self.hover_thrust = self._fit_hover()
            print(f"  [CAL] hover_thrust = {self.hover_thrust:.3f}", flush=True)

        # Hold CURRENT yaw — drone spawns facing ~south; commanding yaw 0
        # made the stabilizer whip through a 180° flip at the gun (run 1).
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

        # Efforts → attitude, holding CURRENT yaw (never command yaw motion).
        # Rotate the world-frame tilt (toward +north, +east) into the body
        # frame: forward tilt = nose down = negative FRD pitch; right tilt =
        # positive roll.
        yaw = ds.yaw_rad
        cy, sy = math.cos(yaw), math.sin(yaw)
        tilt_fwd = cy * out.tilt_x + sy * out.tilt_y
        tilt_right = -sy * out.tilt_x + cy * out.tilt_y
        pitch = -MAX_TILT_RAD * tilt_fwd
        roll = MAX_TILT_RAD * tilt_right

        # Slew-limit attitude targets: rapidly alternating large targets
        # tumbled the sim's stabilizer (run 3).
        now = time.time()
        dt = max(1e-3, now - self._last_cmd_t) if self._last_cmd_t else 1.0 / CONTROL_HZ
        self._last_cmd_t = now
        max_step = ATT_SLEW_RAD_S * dt
        roll = self._last_roll_cmd + max(-max_step, min(max_step, roll - self._last_roll_cmd))
        pitch = self._last_pitch_cmd + max(-max_step, min(max_step, pitch - self._last_pitch_cmd))
        self._last_roll_cmd = roll
        self._last_pitch_cmd = pitch

        vertical = max(EFFORT_FLOOR, min(1.0, out.vertical))
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
        print(f"  {ds_str}  {rs_str}  {hb_str}  {att}", flush=True)
