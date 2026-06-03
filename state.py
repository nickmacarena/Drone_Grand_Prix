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


@dataclass
class SharedState:
    """Mutable container. Background threads write, control loop reads."""
    drone_state: Optional[DroneState] = None
    track_data: Optional[TrackData] = None
    race_status: Optional[RaceStatus] = None
    latest_frame: Optional[object] = None  # numpy array, kept loosely typed
