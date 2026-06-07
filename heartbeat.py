"""Background HEARTBEAT sender.

MAVLink protocol requires both ends to heartbeat. The official example
omits this, but the example doesn't actually fly the drone — so we're
adding it here to rule out 'sim ignores us because we appear stale'.
"""

import threading
import time

from pymavlink import mavutil


HEARTBEAT_HZ = 1


class Heartbeat:
    def __init__(self, mavlink_conn):
        self.mavlink_conn = mavlink_conn
        self.is_running = True
        self.thread = threading.Thread(target=self._loop, daemon=False)
        self.thread.start()

    def get_thread_for_join(self):
        self.is_running = False
        return self.thread

    def _loop(self):
        while self.is_running:
            self.mavlink_conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,  # base_mode
                0,  # custom_mode
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            time.sleep(1.0 / HEARTBEAT_HZ)
