"""Background MAVLink receiver.

Listens for MAVLink messages and updates SharedState. Handles the
ENCAPSULATED_DATA messages that carry race status and track layout.
"""

import struct
import threading
import time

from pymavlink import mavutil

from state import (
    DroneState,
    Gate,
    HeartbeatStatus,
    ImuSample,
    RaceStatus,
    SharedState,
    TrackData,
)


ENCAPSULATED_RACE_STATUS_MSG_ID = 1
ENCAPSULATED_TRACK_INFO_MSG_ID = 2


class MAVLinkRX:
    def __init__(self, mavlink_conn, shared: SharedState):
        self.mavlink_conn = mavlink_conn
        self.shared = shared
        self.is_running = True

        # Track-data chunk reassembly (track info arrives in chunks)
        self._track_chunks: dict[int, dict[int, bytes]] = {}
        self._track_total: dict[int, int] = {}

        # Latest fields needed to build a DroneState (some come from different msgs)
        self._imu_logged = False
        self._pos = None  # (n, e, d, vn, ve, vd)
        self._att = None  # (roll, pitch, yaw)

        self.thread = threading.Thread(target=self._loop, daemon=False)
        self.thread.start()

    def get_thread_for_join(self):
        self.is_running = False
        return self.thread

    def _loop(self):
        while self.is_running:
            try:
                msg = self.mavlink_conn.recv_match(blocking=False)
            except ConnectionResetError:
                print("WARNING: MAVLink connection reset")
                return

            if msg is None:
                time.sleep(0.001)
                continue

            t = msg.get_type()
            if t == "BAD_DATA":
                continue
            elif t == "ATTITUDE":
                self._on_attitude(msg)
            elif t == "LOCAL_POSITION_NED":
                self._on_local_position_ned(msg)
            elif t == "ENCAPSULATED_DATA":
                self._on_encapsulated_data(msg)
            elif t == "DATA_TRANSMISSION_HANDSHAKE":
                self._track_chunks[msg.width] = {}
                self._track_total[msg.width] = msg.packets
            elif t == "HIGHRES_IMU":
                self._on_highres_imu(msg)
            elif t == "HEARTBEAT":
                self._on_heartbeat(msg)
            elif t == "COMMAND_ACK":
                self._on_command_ack(msg)
            elif t == "STATUSTEXT":
                self._on_statustext(msg)

    def _on_attitude(self, msg):
        self._att = (msg.roll, msg.pitch, msg.yaw)
        self._maybe_publish_drone_state()

    def _on_local_position_ned(self, msg):
        self._pos = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
        self._maybe_publish_drone_state()

    def _maybe_publish_drone_state(self):
        if self._pos is None or self._att is None:
            return
        n, e, d, vn, ve, vd = self._pos
        roll, pitch, yaw = self._att
        self.shared.drone_state = DroneState(
            north_m=n, east_m=e, down_m=d,
            vn_mps=vn, ve_mps=ve, vd_mps=vd,
            roll_rad=roll, pitch_rad=pitch, yaw_rad=yaw,
        )

    def _on_encapsulated_data(self, msg):
        payload = bytes(msg.data)
        data_type = payload[0]
        if data_type == ENCAPSULATED_RACE_STATUS_MSG_ID:
            self._on_race_status(payload)
        elif data_type == ENCAPSULATED_TRACK_INFO_MSG_ID:
            self._on_track_chunk(msg, payload)

    def _on_race_status(self, payload):
        # <BQqqIq>: data_type, sim_boot_ms, race_start_boot_ms, race_finish_ns, active_gate, last_gate_time
        _, sim_boot_ms, race_start_ms, race_finish_ns, active_gate, last_gate_time = struct.unpack_from(
            "<BQqqIq", payload
        )
        # Race has actually started once sim time has reached the scheduled start.
        # race_start_ms < 0 means the race isn't even scheduled yet.
        race_started = race_start_ms >= 0 and sim_boot_ms >= race_start_ms
        self.shared.race_status = RaceStatus(
            active_gate_index=active_gate,
            race_started=race_started,
            race_finished=race_finish_ns >= 0,
            last_gate_race_time_s=last_gate_time / 1e9 if last_gate_time > 0 else 0.0,
        )

    def _on_track_chunk(self, msg, payload):
        # <BH>: data_type, transfer_id
        _, transfer_id = struct.unpack_from("<BH", payload)
        if transfer_id not in self._track_total:
            return

        chunk = payload[3:]
        self._track_chunks[transfer_id][msg.seqnr] = chunk

        if len(self._track_chunks[transfer_id]) == self._track_total[transfer_id]:
            full = b"".join(self._track_chunks[transfer_id][i] for i in range(self._track_total[transfer_id]))
            del self._track_chunks[transfer_id]
            del self._track_total[transfer_id]
            self._parse_track_data(full)

    def _parse_track_data(self, payload):
        num_gates, = struct.unpack_from("<H", payload)
        payload = payload[2:]
        gates = []
        # Each gate: <Hfffffffff> = id, pos(3), quat(4), width, height = 2 + 9*4 = 38 bytes
        for _ in range(num_gates):
            gid, nx, ny, nz, qw, qx, qy, qz, w, h = struct.unpack_from("<Hfffffffff", payload)
            gates.append(Gate(gid, nx, ny, nz, qw, qx, qy, qz, w, h))
            payload = payload[38:]
        self.shared.track_data = TrackData(gates=tuple(gates))

    def _on_highres_imu(self, msg):
        """The only state sensor VQ2 leaves us."""
        self.shared.imu = ImuSample(
            t_us=int(getattr(msg, "time_usec", 0)),
            ax=float(msg.xacc), ay=float(msg.yacc), az=float(msg.zacc),
            gx=float(msg.xgyro), gy=float(msg.ygyro), gz=float(msg.zgyro),
            mx=float(getattr(msg, "xmag", 0.0)),
            my=float(getattr(msg, "ymag", 0.0)),
            mz=float(getattr(msg, "zmag", 0.0)),
            abs_pressure=float(getattr(msg, "abs_pressure", 0.0)),
            pressure_alt=float(getattr(msg, "pressure_alt", 0.0)),
            fields_updated=int(getattr(msg, "fields_updated", 0)),
        )
        if not self._imu_logged:
            self._imu_logged = True
            s = self.shared.imu
            # Which optional fields the sim actually populates decides whether
            # we get a heading reference (mag -> fixes yaw drift) or an
            # altitude reference (pressure_alt). Worth knowing on run one.
            print(f"  [IMU] first sample: fields_updated={s.fields_updated:#x} "
                  f"accel=({s.ax:+.2f},{s.ay:+.2f},{s.az:+.2f}) "
                  f"gyro=({s.gx:+.3f},{s.gy:+.3f},{s.gz:+.3f}) "
                  f"mag=({s.mx:+.2f},{s.my:+.2f},{s.mz:+.2f}) "
                  f"pressure_alt={s.pressure_alt:.2f}", flush=True)

    def _on_heartbeat(self, msg):
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.shared.heartbeat = HeartbeatStatus(
            armed=armed,
            base_mode=msg.base_mode,
            custom_mode=msg.custom_mode,
            system_status=msg.system_status,
        )

    def _on_command_ack(self, msg):
        # Silent — these were noisy debugging output and we've moved past needing them.
        # Re-enable by logging to a file if needed.
        pass

    def _on_statustext(self, msg):
        # Silent. Re-enable if the sim starts complaining about anything.
        pass
