"""Steering law for the cyan floor corridor. Pure Python, no numpy.

corridor.py does the image processing and needs numpy; this holds the control
law and deliberately does not, so it can be tested anywhere the rest of the
pure-Python suite runs. The interface between them is three floats.

WHY THIS EXISTS. Twenty-one official runs navigated by orange gate blobs, and
every recurring failure was a consequence of that choice rather than of the
control law: impostor tracks, a continuity gate that locked itself out, ceiling
gates, gate 1 visible for about two metres at the frame edge, and an elevation
walk-up that no rate limit can catch (a track sliding onto a higher object and a
gate genuinely rising as you close on it look identical frame to frame).

The corridor has none of those properties. It is continuous, there is only one of
it, it covers 2-18% of the frame against a gate blob's <1%, and it shows the
upcoming turn before the current gate is passed. Measured in live flight on the
last run: coverage 1.00 across 44 sampled rows with the lane centre 5 px off.

So the corridor decides WHERE TO GO, and the gate is demoted to a terminal height
cue plus the progress signal. That removes the whole class of failure above
rather than adding another guard to it.
"""

import math
from dataclasses import dataclass


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


@dataclass(frozen=True)
class CorridorConfig:
    # Lane centre -> steering. Yaw points the nose along the lane; roll corrects
    # crossrange directly, because waiting for the nose to come round and then
    # for forward tilt to push the new way is what made every turn go wide.
    kp_yaw: float = 1.6              # rad/s per rad of lane bearing
    kd_yaw: float = 0.25             # damping on the lane bearing rate
    max_yaw_rate: float = 1.2
    kp_lat: float = 1.1              # lateral tilt per rad of lane bearing
    max_lat: float = 0.60

    # Lookahead: the far lane centre relative to the near one says which way the
    # course bends. Turning early is the entire advantage of seeing the corridor
    # instead of the next gate, so this is a feed-forward term, not a correction.
    kp_look: float = 0.45            # rad/s per rad of lookahead bearing

    # Forward drive, eased when poorly lined up so we turn onto the lane rather
    # than charging across it. VQ2 has no time limit; there is nothing to gain
    # from carrying speed into a corner.
    cruise_tilt: float = 0.35
    min_tilt_frac: float = 0.20
    align_falloff_rad: float = 0.35

    # Trust gate. Coverage is the fraction of sampled image rows that yielded a
    # measurement; below this the corridor is a scrap of cyan somewhere rather
    # than a lane, and the gate servo should keep flying.
    min_coverage: float = 0.25


@dataclass
class CorridorState:
    lane_f: float = 0.0              # smoothed lane bearing, radians
    lane_rate: float = 0.0
    last_t: float | None = None
    have_lane: bool = False
    ema_alpha: float = 0.4
    rate_alpha: float = 0.35


@dataclass(frozen=True)
class CorridorCommand:
    tilt_fwd: float = 0.0
    tilt_right: float = 0.0
    yaw_rate: float = 0.0
    have_lane: bool = False


def step(state: CorridorState, cfg: CorridorConfig, t: float,
         lane_rad: float | None, look_rad: float | None,
         coverage: float) -> CorridorCommand:
    """One control cycle.

    `lane_rad` is the bearing of the lane centre in the near field, positive to
    the right; `look_rad` the same for the far field minus the near, positive
    when the course bends right; `coverage` the fraction of rows measured. Pass
    lane_rad=None when the corridor was not seen (or the aircraft is not upright
    — that check belongs to the caller, which has the attitude estimate).
    """
    dt = 0.0 if state.last_t is None else max(0.0, t - state.last_t)
    state.last_t = t

    usable = lane_rad is not None and coverage >= cfg.min_coverage
    if not usable:
        state.have_lane = False
        return CorridorCommand(have_lane=False)

    prev = state.lane_f
    a = state.ema_alpha if state.have_lane else 1.0
    state.lane_f += a * (lane_rad - state.lane_f)
    if dt > 1e-3 and state.have_lane:
        state.lane_rate += state.rate_alpha * (
            (state.lane_f - prev) / dt - state.lane_rate)
    state.have_lane = True

    look = 0.0 if look_rad is None or math.isnan(look_rad) else look_rad
    yaw = (cfg.kp_yaw * state.lane_f
           + cfg.kd_yaw * state.lane_rate
           + cfg.kp_look * look)
    yaw = _clamp(yaw, -cfg.max_yaw_rate, cfg.max_yaw_rate)

    lateral = _clamp(cfg.kp_lat * state.lane_f, -cfg.max_lat, cfg.max_lat)

    align = 1.0 / (1.0 + (abs(state.lane_f) / cfg.align_falloff_rad) ** 2)
    drive = max(cfg.min_tilt_frac, align)

    return CorridorCommand(
        tilt_fwd=cfg.cruise_tilt * drive,
        tilt_right=lateral,
        yaw_rate=yaw,
        have_lane=True,
    )


@dataclass(frozen=True)
class Blend:
    """What the aircraft should actually do this cycle."""
    tilt_fwd: float
    tilt_right: float
    yaw_rate: float
    used_corridor: bool


def blend(cmd: CorridorCommand, servo_tilt_fwd: float, servo_tilt_right: float,
          servo_yaw_rate: float, braking: bool) -> Blend:
    """Combine corridor steering with the gate servo.

    The corridor decides WHERE TO GO whenever it has a lane; the gate servo keeps
    the vertical channel (handled by the caller, which owns thrust) and takes over
    steering entirely when the lane is lost. Braking is preserved either way,
    because shedding speed outranks steering.

    Lives here so controller_vq2 and the Elodin harness cannot drift apart. They
    did drift once already — elodin_servo never called gate_passed() and projected
    only the active gate, so Elodin was validating code the sim could not reach.
    """
    if not cmd.have_lane:
        return Blend(servo_tilt_fwd, servo_tilt_right, servo_yaw_rate, False)
    return Blend(cmd.tilt_fwd, cmd.tilt_right, cmd.yaw_rate, True)
