"""Shared state and data types.

All telemetry, race status, and track data flows through SharedState.
Background threads (mavlink_rx, vision_rx) populate it. The control loop reads it.

Threading note: each field is replaced atomically with an immutable dataclass.
CPython's GIL makes single-field assignment safe — no explicit locks needed.
"""

from dataclasses import dataclass, field
from typing import Optional


@dataclass(frozen=True)
class DroneState:
    """Latest drone telemetry. Built from LOCAL_POSITION_NED + ATTITUDE."""
    north_m: float
    east_m: float
    down_m: float
    vn_mps: float
    ve_mps: float
    vd_mps: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float


@dataclass(frozen=True)
class Gate:
    """One gate on the course. Provided by the sim via track data."""
    gate_id: int
    north_m: float
    east_m: float
    down_m: float
    # Orientation quaternion in NED (w, x, y, z)
    qw: float
    qx: float
    qy: float
    qz: float
    width_m: float
    height_m: float


@dataclass(frozen=True)
class TrackData:
    """Full course layout. Sent once by the sim at the start."""
    gates: tuple[Gate, ...]


@dataclass(frozen=True)
class RaceStatus:
    """Race state from ENCAPSULATED_DATA race status messages."""
    active_gate_index: int
    race_started: bool
    race_finished: bool
    last_gate_race_time_s: float


@dataclass(frozen=True)
class ImuSample:
    """HIGHRES_IMU. In VQ2 this is the ONLY state sensor: ATTITUDE,
    LOCAL_POSITION_NED and ODOMETRY are all disabled."""
    t_us: int
    ax: float           # m/s^2, body FRD (specific force — includes gravity)
    ay: float
    az: float
    gx: float           # rad/s, body FRD
    gy: float
    gz: float
    # Optional extras. MAVLink defines them; whether the sim populates them is
    # reported by `fields_updated` and is worth knowing: a live magnetometer
    # would fix the estimator's unobservable yaw drift, and pressure_alt would
    # give an absolute altitude reference we otherwise do not have.
    mx: float = 0.0
    my: float = 0.0
    mz: float = 0.0
    abs_pressure: float = 0.0
    pressure_alt: float = 0.0
    fields_updated: int = 0


@dataclass(frozen=True)
class HeartbeatStatus:
    """Latest HEARTBEAT from the sim. Tells us armed state and current mode."""
    armed: bool
    base_mode: int        # MAV_MODE_FLAG bitfield
    custom_mode: int      # autopilot-specific mode value
    system_status: int    # MAV_STATE


@dataclass
class SharedState:
    """Mutable container. Background threads write, control loop reads."""
    drone_state: Optional[DroneState] = None
    track_data: Optional[TrackData] = None
    race_status: Optional[RaceStatus] = None
    heartbeat: Optional[HeartbeatStatus] = None
    imu: Optional[ImuSample] = None
    latest_frame: Optional[object] = None  # numpy array, kept loosely typed
