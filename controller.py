"""Main control loop.

Strategy for VQ1:
    Track data gives us every gate's NED position.
    Race status tells us which gate is active.
    Local position tells us where we are.

    Compute a velocity vector toward the active gate, send via
    SET_POSITION_TARGET_LOCAL_NED. The sim's stabilized controller
    handles attitude and motor mixing.

    The sim disarms us on race transition, so we re-arm whenever
    we detect armed=False.
"""

import math
import time

from mavlink_tx import send_arm, send_velocity_ned
from state import SharedState


CONTROL_HZ = 250
CRUISE_SPEED_MPS = 5.0
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
        vn, ve, vd = self._compute_velocity()
        send_velocity_ned(self.mavlink_conn, self.system_boot_ms, vn, ve, vd)
        self._maybe_log(vn, ve, vd)
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

    def _compute_velocity(self) -> tuple[float, float, float]:
        """Return (vn, ve, vd) m/s in NED. Zero if we lack data or race isn't live."""
        ds = self.shared.drone_state
        td = self.shared.track_data
        rs = self.shared.race_status

        if ds is None or td is None or rs is None:
            return 0.0, 0.0, 0.0

        # Don't move before the race officially starts — early motion = DQ.
        if not rs.race_started:
            return 0.0, 0.0, 0.0

        if rs.race_finished or rs.active_gate_index >= len(td.gates):
            return 0.0, 0.0, 0.0

        gate = td.gates[rs.active_gate_index]
        dn = gate.north_m - ds.north_m
        de = gate.east_m - ds.east_m
        dd = gate.down_m - ds.down_m

        dist = math.sqrt(dn * dn + de * de + dd * dd)
        if dist < 0.1:
            return 0.0, 0.0, 0.0

        scale = CRUISE_SPEED_MPS / dist
        return dn * scale, de * scale, dd * scale

    def _maybe_log(self, vn: float, ve: float, vd: float):
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
        rs_str = (
            f"active={rs.active_gate_index}"
            if rs else "race=None"
        )
        hb_str = f"armed={hb.armed}" if hb else "hb=None"
        vel_str = f"vel=({vn:.2f},{ve:.2f},{vd:.2f})"
        print(f"  {ds_str}  {rs_str}  {hb_str}  {vel_str}", flush=True)
