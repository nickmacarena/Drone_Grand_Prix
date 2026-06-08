"""Control loop orchestration.

Thin layer: read SharedState, call into pid.py for the math, send via mavlink_tx.
Handles re-arming and the pre-race hold.
"""

import math
import time

from attitude import euler_to_quaternion
from mavlink_tx import send_arm, send_attitude_quaternion
from pid import compute_attitude_target
from state import SharedState


CONTROL_HZ = 250
REARM_PERIOD_S = 0.5
LOG_PERIOD_S = 1.0


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
        roll, pitch, yaw, thrust = self._select_attitude_target()
        q = euler_to_quaternion(roll, pitch, yaw)
        send_attitude_quaternion(self.mavlink_conn, self.system_boot_ms, q, thrust)
        self._maybe_log(roll, pitch, yaw, thrust)
        time.sleep(1.0 / CONTROL_HZ)

    def _maybe_rearm(self):
        """Re-arm if the sim has disarmed us (happens on race transition)."""
        hb = self.shared.heartbeat
        if hb is None or hb.armed:
            return
        now = time.time()
        if now - self._last_arm_t < REARM_PERIOD_S:
            return
        send_arm(self.mavlink_conn)
        self._last_arm_t = now

    def _select_attitude_target(self) -> tuple[float, float, float, float]:
        """Decide what attitude+thrust to send right now."""
        ds = self.shared.drone_state
        td = self.shared.track_data
        rs = self.shared.race_status

        # Before race: level, zero thrust. Avoid early-start DQ.
        if ds is None or td is None or rs is None or not rs.race_started:
            return 0.0, 0.0, 0.0, 0.0

        # Race finished or course done: hold level + hover thrust.
        if rs.race_finished or rs.active_gate_index >= len(td.gates):
            from measurements import HOVER_THRUST
            return 0.0, 0.0, ds.yaw_rad, HOVER_THRUST

        # Fly to active gate via the cascaded controller.
        gate = td.gates[rs.active_gate_index]
        target = (gate.north_m, gate.east_m, gate.down_m)
        return compute_attitude_target(ds, target)

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
        att_str = (
            f"r={math.degrees(roll):.0f} p={math.degrees(pitch):.0f} "
            f"y={math.degrees(yaw):.0f} thr={thrust:.2f}"
        )
        print(f"  {ds_str}  {rs_str}  {hb_str}  {att_str}", flush=True)
