"""Background camera receiver.

Listens on UDP port 5600 for JPEG-encoded camera frames sent in chunks.
Reassembles complete frames and stores the latest in SharedState.

Not used in VQ1 — track data tells us where the gates are. Wired up
and ready for VQ2 perception work.
"""

import socket
import struct
import threading

import cv2
import numpy as np

from state import SharedState


VISION_UDP_IP = "0.0.0.0"
VISION_UDP_PORT = 5600


class VisionRX:
    def __init__(self, shared: SharedState):
        self.shared = shared
        self.is_running = True
        self.thread = threading.Thread(target=self._loop, daemon=False)
        self.thread.start()

    def get_thread_for_join(self):
        self.is_running = False
        return self.thread

    def _loop(self):
        # Packet header: frame_id, chunk_id, total_chunks, jpeg_size, payload_size, sim_time_ns
        header_format = "<IHHIIQ"
        header_sz = struct.calcsize(header_format)
        frames: dict[int, dict] = {}

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((VISION_UDP_IP, VISION_UDP_PORT))
        sock.settimeout(0.5)

        while self.is_running:
            try:
                packet, _ = sock.recvfrom(65536)
            except socket.timeout:
                continue

            header = packet[:header_sz]
            payload = packet[header_sz:]
            frame_id, chunk_id, total_chunks, _, _, _ = struct.unpack(header_format, header)

            if frame_id not in frames:
                frames[frame_id] = {"chunks": {}, "total": total_chunks}
            frames[frame_id]["chunks"][chunk_id] = payload

            if len(frames[frame_id]["chunks"]) == total_chunks:
                jpeg_bytes = b"".join(
                    frames[frame_id]["chunks"][i] for i in range(total_chunks)
                )
                arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if image is not None:
                    self.shared.latest_frame = image
                del frames[frame_id]
