"""Shared data types for the pipeline.

All pipeline stages import from here. These types are the contracts
between stages — changing a field here surfaces every place that
needs to be updated.

Conventions:
    - Positions: meters, NED (North-East-Down) frame
    - Velocities: m/s, NED frame
    - Angles: radians (convert at IO boundaries, not inside pipeline)
    - Throttle: 0.0 to 1.0
    - All types are frozen (immutable) to prevent accidental mutation
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class DroneState:
    """Telemetry snapshot at a single point in time.

    This is the primary input to both the planner and controller.
    Built from MAVSDK telemetry streams in io/telemetry.py.
    """

    # Position in NED frame, meters from sim origin.
    # Down is positive, so typical flight altitude is negative.
    north_m: float
    east_m: float
    down_m: float

    # Velocity in NED frame, m/s.
    vn_mps: float
    ve_mps: float
    vd_mps: float

    # Attitude in radians.
    # Roll: positive = right wing down
    # Pitch: positive = nose up
    # Yaw: positive = clockwise from north (0 = north, pi/2 = east)
    roll_rad: float
    pitch_rad: float
    yaw_rad: float

    # Timestamp in seconds from an arbitrary epoch.
    # Used to compute dt between controller ticks.
    timestamp_s: float


@dataclass(frozen=True)
class GateObservation:
    """A gate detected by perception in a single camera frame.

    Position is relative to the drone in NED frame. With monocular
    vision and no depth sensor, the distance estimate will be rough —
    derived from known gate dimensions vs. apparent size in frame.
    """

    # Estimated position of the gate center relative to the drone, meters NED.
    rel_north_m: float
    rel_east_m: float
    rel_down_m: float

    # How confident perception is in this detection (0.0–1.0).
    confidence: float

    # Which gate in the sequence this is (0-indexed).
    # None if perception can't determine which gate it's looking at.
    gate_index: int | None


@dataclass(frozen=True)
class Waypoint:
    """A target position for the drone to fly toward.

    Output of the planner, input to the controller.
    """

    # Target position in NED frame, meters from sim origin.
    north_m: float
    east_m: float
    down_m: float

    # Target heading in radians. None means "don't care about yaw,
    # just point toward the waypoint."
    yaw_rad: float | None = None


@dataclass(frozen=True)
class AttitudeCommand:
    """Command sent to the sim via SET_ATTITUDE_TARGET.

    Output of the controller. The sim's stabilized controller
    handles the low-level motor mixing.
    """

    # Throttle: 0.0 = no thrust, 1.0 = max thrust.
    # In hover, this is roughly 0.5 (balancing gravity).
    throttle: float

    # Target attitude in radians.
    # These are absolute angles, not rates.
    # Roll: positive = right wing down → drone moves east
    # Pitch: positive = nose up → drone moves south (NED)
    #   (careful: positive pitch = nose up = deceleration/backward in NED)
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
