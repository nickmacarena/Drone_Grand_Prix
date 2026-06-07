"""Main control loop.

DIAGNOSTIC MODE: sends attitude rates + thrust=0.6 (mirrors example values).
Verifies that the drone is armed and responsive to ANY control command.
If the drone lifts off in this mode, velocity setpoints are the problem.
If still nothing moves, the issue is upstream (arming, mode, etc).
"""

import time

from mavlink_tx import send_arm, send_attitude_rates
from state import SharedState


CONTROL_HZ = 250
TEST_THRUST = 0.6
LOG_PERIOD_S = 1.0


class Controller:
    def __init__(self, mavlink_conn, shared: SharedState, system_boot_ms: int):
        self.mavlink_conn = mavlink_conn
        self.shared = shared
        self.system_boot_ms = system_boot_ms
        self._last_log_t = 0.0
        self._logged_gates = False

    def arm(self):
        send_arm(self.mavlink_conn)

    def update(self):
        # Zero rates, thrust=0.6. Should make the drone hover/climb.
        send_attitude_rates(self.mavlink_conn, self.system_boot_ms, 0.0, 0.0, 0.0, TEST_THRUST)
        self._maybe_log()
        time.sleep(1.0 / CONTROL_HZ)

    def _maybe_log(self):
        now = time.time()
        if now - self._last_log_t < LOG_PERIOD_S:
            return
        self._last_log_t = now

        ds = self.shared.drone_state
        td = self.shared.track_data
        rs = self.shared.race_status

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
            f"active={rs.active_gate_index} started={rs.race_started}"
            if rs else "race=None"
        )
        print(f"  {ds_str}  {rs_str}  thrust={TEST_THRUST}", flush=True)
