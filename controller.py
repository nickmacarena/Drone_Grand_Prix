"""Main control loop.

The sim only follows attitude+thrust commands — velocity and position
setpoints are ignored. So we build a position-to-attitude P controller:

    1. Compute world-NED error from drone to target gate.
    2. Yaw toward the gate (Tait-Bryan ZYX, NED).
    3. Rotate horizontal error into body frame using target yaw.
    4. Body-forward error → negative pitch (nose down = forward).
    5. Body-right   error → positive roll  (right tilt = right).
    6. Altitude error (vertical) → thrust offset from hover.
    7. Send SET_ATTITUDE_TARGET with a quaternion + thrust at 250 Hz.

The sim disarms us at race transition, so we re-arm whenever we see
armed=False.
"""

import math
import time

from mavlink_tx import send_arm, send_attitude_quaternion
from state import SharedState


CONTROL_HZ = 250
REARM_PERIOD_S = 0.5
LOG_PERIOD_S = 1.0

# Controller gains — tune from these starting points.
K_PITCH = 0.10           # rad per meter of forward error
K_ROLL = 0.10            # rad per meter of right error
K_ALT = 0.02             # thrust per meter of altitude error
HOVER_THRUST = 0.55
MAX_TILT_RAD = math.radians(25)
THRUST_MIN = 0.30
THRUST_MAX = 0.85


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> list[float]:
    """Tait-Bryan ZYX (yaw-pitch-roll) to quaternion [w, x, y, z], NED."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [w, x, y, z]


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class Controller:
    def __init__(self, mavlink_conn, shared: SharedState, system_boot_ms: int):
        self.mavlink_conn = mavlink_conn
        self.shared = shared
        self.system_boot_ms = system_boot_ms
        self._last_log_t = 0.0
        self._logged_gates = False
        self._last_arm_t = 0.0

    def arm(self):
        send_arm(self.mavlink_conn)
        self._last_arm_t = time.time()

    def update(self):
        self._maybe_rearm()
        roll, pitch, yaw, thrust = self._compute_attitude()
        q = euler_to_quaternion(roll, pitch, yaw)
        send_attitude_quaternion(self.mavlink_conn, self.system_boot_ms, q, thrust)
        self._maybe_log(roll, pitch, yaw, thrust)
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

    def _compute_attitude(self) -> tuple[float, float, float, float]:
        """Return (roll_rad, pitch_rad, yaw_rad, thrust). Holds level pre-race."""
        ds = self.shared.drone_state
        td = self.shared.track_data
        rs = self.shared.race_status

        # Hold level + zero thrust if we lack data or race isn't live
        if ds is None or td is None or rs is None or not rs.race_started:
            return 0.0, 0.0, 0.0, 0.0

        if rs.race_finished or rs.active_gate_index >= len(td.gates):
            # Try to hover in place — hold level, current yaw, hover thrust
            return 0.0, 0.0, ds.yaw_rad, HOVER_THRUST

        gate = td.gates[rs.active_gate_index]

        # World-frame position error (NED)
        en = gate.north_m - ds.north_m
        ee = gate.east_m - ds.east_m
        ed = gate.down_m - ds.down_m

        # Yaw: face the gate (atan2(east, north) in NED)
        target_yaw = math.atan2(ee, en)

        # Rotate world horizontal error into body frame using TARGET yaw,
        # since we want the drone to fly forward toward the gate.
        cy = math.cos(target_yaw)
        sy = math.sin(target_yaw)
        forward_err = cy * en + sy * ee   # body +x (forward)
        right_err = -sy * en + cy * ee    # body +y (right)

        # Forward error → pitch nose down (negative pitch in NED body frame).
        pitch_cmd = -clamp(K_PITCH * forward_err, -MAX_TILT_RAD, MAX_TILT_RAD)
        roll_cmd = clamp(K_ROLL * right_err, -MAX_TILT_RAD, MAX_TILT_RAD)

        # Altitude: ed > 0 means gate is below us (deeper, positive down).
        # We want to descend → less thrust than hover.
        thrust = HOVER_THRUST - K_ALT * ed
        thrust = clamp(thrust, THRUST_MIN, THRUST_MAX)

        return roll_cmd, pitch_cmd, target_yaw, thrust

    def _maybe_log(self, roll, pitch, yaw, thrust):
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
            f"pos=({ds.north_m:.1f},{ds.east_m:.1f},{ds.down_m:.1f})"
            if ds else "pos=None"
        )
        rs_str = f"active={rs.active_gate_index}" if rs else "race=None"
        hb_str = f"armed={hb.armed}" if hb else "hb=None"
        att_str = f"r={math.degrees(roll):.0f} p={math.degrees(pitch):.0f} y={math.degrees(yaw):.0f} thr={thrust:.2f}"
        print(f"  {ds_str}  {rs_str}  {hb_str}  {att_str}", flush=True)
