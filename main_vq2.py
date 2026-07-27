"""VQ2 entry point (official sim v1.0.3391+).

Kept separate from main.py so the VQ1 solution that passed stays runnable.

    cd $HOME\\Drone_Grand_Prix
    git pull
    pip install pillow numpy          # once; no compiler needed on win_arm64
    $env:MISSION="frames"
    $env:FRAME_DIR="C:\\Users\\nick\\vq2_frames"
    python main_vq2.py | Tee-Object -FilePath vq2_run.log

Then click RACE. MISSION=hover holds level on IMU-derived attitude alone;
MISSION=frames does the same and captures camera frames to FRAME_DIR.
"""

import os
import time

from pymavlink import mavutil

from controller_vq2 import ControllerVQ2
from heartbeat import Heartbeat
from mavlink_rx import MAVLinkRX
from state import SharedState
from timesync import TimeSync

SIM_SERVER_UDP_IP = "0.0.0.0"
SIM_SERVER_UDP_PORT = 14550


def main():
    system_boot_ms = int(time.time() * 1000)

    conn = mavutil.mavlink_connection(
        f"udpin:{SIM_SERVER_UDP_IP}:{SIM_SERVER_UDP_PORT}")
    print("Waiting for heartbeat...", flush=True)
    conn.wait_heartbeat()
    print(f"Connected to system: {conn.target_system}", flush=True)

    shared = SharedState()
    rx = MAVLinkRX(conn, shared)
    ts = TimeSync(conn)
    hb = Heartbeat(conn)

    # Vision is optional: without pillow/numpy the rest still runs, which
    # keeps the IMU bring-up usable on a VM that has not been set up yet.
    vision = None
    try:
        from vision_rx import VisionRX
        vision = VisionRX(shared)
    except Exception as e:
        print(f"  [VISION] disabled: {e}", flush=True)

    controller = ControllerVQ2(conn, shared, system_boot_ms)

    print(f"Mission: {os.environ.get('MISSION', 'hover')}. Click RACE in the sim.",
          flush=True)
    controller.arm()

    try:
        while True:
            controller.update()
            rs = shared.race_status
            if rs is not None and rs.race_finished:
                print("Race finished!", flush=True)
                break
    except KeyboardInterrupt:
        print("Interrupted.", flush=True)

    if vision is not None:
        print(f"  [VISION] {vision.frames_decoded} frames decoded, "
              f"{vision.frames_saved} saved", flush=True)
        vision.get_thread_for_join().join(timeout=1.0)
    for t in (rx, ts, hb):
        t.get_thread_for_join().join(timeout=1.0)
    print("Exited.", flush=True)


if __name__ == "__main__":
    main()
