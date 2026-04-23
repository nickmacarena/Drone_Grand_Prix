"""Smoke test: run one tick of the full pipeline with known inputs."""

import sys
import os
import math

# Add project root to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from drone_types import DroneState
from pipeline.perception import detect_gates
from pipeline.planner import plan_next_waypoint
from pipeline.controller import compute_attitude_command


def make_state(north=0.0, east=0.0, down=-3.0):
    return DroneState(
        north_m=north, east_m=east, down_m=down,
        vn_mps=0.0, ve_mps=0.0, vd_mps=0.0,
        roll_rad=0.0, pitch_rad=0.0, yaw_rad=0.0,
        timestamp_s=0.0,
    )


def test_full_tick():
    """One tick: perception → planner → controller. Verify no crashes and sane output."""
    state = make_state()

    observations = detect_gates(None, state)
    assert len(observations) > 0, "Stub perception should return gates"

    waypoint, next_idx = plan_next_waypoint(observations, state, next_gate_index=0)
    assert next_idx == 0, "Shouldn't have reached gate 0 yet (10m away)"
    assert waypoint.north_m == 10.0, "Should target first gate"

    cmd = compute_attitude_command(waypoint, state)
    assert 0.0 <= cmd.throttle <= 1.0, f"Throttle out of range: {cmd.throttle}"
    assert abs(cmd.roll_rad) <= math.radians(25), "Roll exceeds max tilt"
    assert abs(cmd.pitch_rad) <= math.radians(25), "Pitch exceeds max tilt"

    print(f"Waypoint: ({waypoint.north_m}, {waypoint.east_m}, {waypoint.down_m})")
    print(f"Command: thr={cmd.throttle:.2f} roll={math.degrees(cmd.roll_rad):.1f} pitch={math.degrees(cmd.pitch_rad):.1f} yaw={math.degrees(cmd.yaw_rad):.1f}")
    print("PASS")


if __name__ == "__main__":
    test_full_tick()
