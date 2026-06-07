"""Component wiring. Mirrors the AIGP example's setup.py."""

from pymavlink import mavutil

from controller import Controller
from heartbeat import Heartbeat
from mavlink_rx import MAVLinkRX
from state import SharedState
from timesync import TimeSync

# Vision deferred until VQ2 — opencv/numpy install is blocked on Python 3.14 win_arm64.
# from vision_rx import VisionRX


def setup_components(server_ip: str, server_port: int, system_boot_ms: int):
    mavlink_conn = mavutil.mavlink_connection(f"udpin:{server_ip}:{server_port}")
    print("Waiting for heartbeat...", flush=True)
    mavlink_conn.wait_heartbeat()
    print(f"Connected to system: {mavlink_conn.target_system}", flush=True)

    shared = SharedState()
    mavlink_rx = MAVLinkRX(mavlink_conn, shared)
    timesync = TimeSync(mavlink_conn)
    heartbeat = Heartbeat(mavlink_conn)
    controller = Controller(mavlink_conn, shared, system_boot_ms)

    return {
        "mavlink_conn": mavlink_conn,
        "shared": shared,
        "mavlink_rx": mavlink_rx,
        "timesync": timesync,
        "heartbeat": heartbeat,
        "controller": controller,
    }
