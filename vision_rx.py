"""Background camera receiver: reassembles the FPV JPEG stream (UDP 5600).

Rewritten for VQ2. Two changes from the VQ1 version:

1. **Pillow instead of OpenCV.** opencv-python has no win_arm64 wheels at any
   version, so it cannot be installed on the Windows ARM VM at all. Pillow and
   numpy both ship cp314 win_arm64 wheels, need no compiler, and give us JPEG
   decode plus array access — everything a gate detector needs.
       pip install pillow numpy

2. **Frame capture.** Set FRAME_DIR to dump decoded frames (plus a JSONL
   sidecar) to disk. Detector development then happens offline against real
   VQ2 imagery, instead of once per 5-minute sim run.

The wire format is unchanged from VQ1: chunked JPEG with a 24-byte header
    <IHHIIQ> = frame_id, chunk_id, total_chunks, jpeg_size, payload_size, sim_time_ns
"""

import io
import json
import os
import socket
import struct
import threading
import time

import numpy as np
from PIL import Image

from state import SharedState


VISION_UDP_IP = "0.0.0.0"
VISION_UDP_PORT = 5600

FRAME_DIR = os.environ.get("FRAME_DIR", "")          # "" disables capture
CAPTURE_PERIOD_S = float(os.environ.get("CAPTURE_PERIOD_S", "0.5"))
CAPTURE_MAX = int(os.environ.get("CAPTURE_MAX", "400"))

_HEADER_FMT = "<IHHIIQ"
_HEADER_SZ = struct.calcsize(_HEADER_FMT)


class VisionRX:
    def __init__(self, shared: SharedState):
        self.shared = shared
        self.is_running = True
        self.frames_decoded = 0
        self.frames_saved = 0
        self._first_logged = False
        self._next_capture = 0.0
        self._labels = None
        if FRAME_DIR:
            os.makedirs(FRAME_DIR, exist_ok=True)
            self._labels = open(os.path.join(FRAME_DIR, "labels.jsonl"), "w")
            print(f"  [VISION] capturing frames to {FRAME_DIR}", flush=True)
        self.thread = threading.Thread(target=self._loop, daemon=False)
        self.thread.start()

    def get_thread_for_join(self):
        self.is_running = False
        return self.thread

    def _loop(self):
        frames: dict[int, dict] = {}
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((VISION_UDP_IP, VISION_UDP_PORT))
        sock.settimeout(0.5)

        while self.is_running:
            try:
                packet, _ = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(packet) < _HEADER_SZ:
                continue

            frame_id, chunk_id, total_chunks, _, _, sim_time_ns = struct.unpack(
                _HEADER_FMT, packet[:_HEADER_SZ])
            payload = packet[_HEADER_SZ:]

            slot = frames.setdefault(frame_id, {"chunks": {}, "total": total_chunks,
                                                "t_ns": sim_time_ns})
            slot["chunks"][chunk_id] = payload

            if len(slot["chunks"]) < slot["total"]:
                continue
            del frames[frame_id]

            try:
                jpeg = b"".join(slot["chunks"][i] for i in range(slot["total"]))
                img = Image.open(io.BytesIO(jpeg))
                img.load()
            except Exception:
                continue                      # torn/dropped chunk; skip frame

            arr = np.asarray(img.convert("RGB"))
            self.shared.latest_frame = arr
            self.frames_decoded += 1

            if not self._first_logged:
                self._first_logged = True
                print(f"  [VISION] first frame: {arr.shape} {arr.dtype}", flush=True)

            self._maybe_capture(img, sim_time_ns)

        # Drain stale partial frames so a long run cannot grow unbounded.
        frames.clear()
        if self._labels:
            self._labels.close()

    def _maybe_capture(self, img, sim_time_ns):
        if not self._labels or self.frames_saved >= CAPTURE_MAX:
            return
        now = time.time()
        if now < self._next_capture:
            return
        self._next_capture = now + CAPTURE_PERIOD_S

        idx = self.frames_saved
        self.frames_saved += 1
        img.convert("RGB").save(os.path.join(FRAME_DIR, f"frame_{idx:04d}.jpg"),
                                quality=92)

        # Label with whatever state we have. VQ2 gives no pose, so this is
        # mostly race context — still enough to know which gate each frame
        # was hunting and roughly how far into the run it is.
        rs = self.shared.race_status
        imu = self.shared.imu
        self._labels.write(json.dumps({
            "i": idx,
            "sim_time_ns": int(sim_time_ns),
            "wall_t": round(now, 3),
            "next_gate_index": (rs.active_gate_index if rs else -1),
            "race_started": bool(rs.race_started) if rs else False,
            "accel": [imu.ax, imu.ay, imu.az] if imu else None,
            "gyro": [imu.gx, imu.gy, imu.gz] if imu else None,
        }) + "\n")
        self._labels.flush()
