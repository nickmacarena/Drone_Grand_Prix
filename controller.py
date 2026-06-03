"""Main control loop.

Strategy for VQ1:
    Track data gives us every gate's NED position.
    Race status tells us which gate is active.
    Local position tells us where we are.

    Compute a velocity vector toward the active gate, send via
    SET_POSITION_TARGET_LOCAL_NED. The sim's stabilized controller
    handles attitude and motor mixing.
"""

import math
import time

from mavlink_tx import send_arm, send_velocity_ned
from state import SharedState


CONTROL_HZ = 250
CRUISE_SPEED_MPS = 3.0


class Controller:
    def __init__(self, mavlink_conn, shared: SharedState, system_boot_ms: int):
        self.mavlink_conn = mavlink_conn
        self.shared = shared
        self.system_boot_ms = system_boot_ms

    def arm(self):
        send_arm(self.mavlink_conn)

    def update(self):
        vn, ve, vd = self._compute_velocity()
        send_velocity_ned(self.mavlink_conn, self.system_boot_ms, vn, ve, vd)
        time.sleep(1.0 / CONTROL_HZ)

    def _compute_velocity(self) -> tuple[float, float, float]:
        """Return (vn, ve, vd) m/s in NED. Zero if we lack data."""
        ds = self.shared.drone_state
        td = self.shared.track_data
        rs = self.shared.race_status

        if ds is None or td is None or rs is None:
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
