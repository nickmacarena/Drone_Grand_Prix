"""Visual servoing: fly toward the gate you SEE. Sim-agnostic, pure Python.

VQ2 removes position, velocity and gate coordinates, so the VQ1 approach
(PD onto a waypoint) has no inputs. What remains is how a human FPV pilot
actually flies: keep the gate centred in the picture and drive forward.

    yaw     turn to centre the gate horizontally  (az -> 0)
    thrust  climb/descend to centre it vertically (el -> el_setpoint)
    pitch   fixed cruise tilt, eased off when poorly lined up
    roll    held level; steering is done with yaw so the gate stays in frame

`el_setpoint` is NOT zero. The camera is pitched 20 deg up, so driving the
gate onto the optical axis would leave us flying 20 deg BELOW it forever.
Centring on our own flight path means el -> -CAM_TILT_UP.

No altitude sensor is needed anywhere here: "the gate looks too high" is
itself the climb signal. When the gate is lost we cannot know which way is
up-course, so the drone holds level, stops climbing, bleeds off speed and
yaws toward wherever it last saw the gate.
"""

import math
from dataclasses import dataclass

from attitude import clamp
from bearing import CAM_TILT_UP_DEG, GateObservation


@dataclass(frozen=True)
class ServoConfig:
    # Horizontal: bearing -> yaw rate (rad/s per rad of azimuth error)
    kp_yaw: float = 2.0
    max_yaw_rate: float = 1.6

    # Vertical: elevation error -> normalized vertical effort
    kp_el: float = 1.6
    max_vertical: float = 0.7
    # Put the gate on our own flight path, not on the up-tilted optical axis.
    el_setpoint_rad: float = -math.radians(CAM_TILT_UP_DEG)

    # Forward drive
    cruise_tilt: float = 0.55        # normalized forward tilt when lined up
    min_tilt_frac: float = 0.15      # floor so we never fully stall out
    align_falloff_rad: float = 0.60  # azimuth error that halves forward drive

    # Confidence gating: a low-confidence detection still steers, but gently.
    min_confidence: float = 0.15

    # Behaviour when no gate is visible
    search_yaw_rate: float = 0.7     # rad/s, toward where it was last seen
    lost_coast_s: float = 0.4        # keep driving briefly (gate just left FOV
                                     # because we are about to fly through it)


@dataclass
class ServoState:
    last_az: float = 0.0
    time_since_seen: float = 1e9
    last_t: float | None = None
    searching: bool = False


@dataclass(frozen=True)
class ServoOutput:
    tilt_fwd: float      # body-forward tilt effort, [-1, 1]
    tilt_right: float    # body-right tilt effort, [-1, 1]
    vertical: float      # vertical effort around hover, [-1, 1]
    yaw_rate: float      # rad/s
    have_target: bool


def step(state: ServoState, cfg: ServoConfig, t: float,
         obs: GateObservation | None) -> ServoOutput:
    dt = 0.0 if state.last_t is None else max(0.0, t - state.last_t)
    state.last_t = t

    usable = obs is not None and obs.confidence >= cfg.min_confidence
    if usable:
        state.last_az = obs.az
        state.time_since_seen = 0.0
        state.searching = False
    else:
        state.time_since_seen += dt

    # ── Gate visible: centre it and drive through ───────────────────────
    if usable:
        yaw_rate = clamp(cfg.kp_yaw * obs.az, -cfg.max_yaw_rate, cfg.max_yaw_rate)

        el_err = obs.el - cfg.el_setpoint_rad
        vertical = clamp(cfg.kp_el * el_err, -cfg.max_vertical, cfg.max_vertical)

        # Ease off forward drive when poorly lined up, so we turn onto the
        # gate rather than charging past its edge. Scale by confidence too.
        align = 1.0 / (1.0 + (abs(obs.az) / cfg.align_falloff_rad) ** 2)
        drive = max(cfg.min_tilt_frac, align) * obs.confidence
        return ServoOutput(
            tilt_fwd=cfg.cruise_tilt * drive,
            tilt_right=0.0,
            vertical=vertical,
            yaw_rate=yaw_rate,
            have_target=True,
        )

    # ── Just lost it: almost certainly flying through it right now ──────
    if state.time_since_seen <= cfg.lost_coast_s:
        return ServoOutput(
            tilt_fwd=cfg.cruise_tilt * 0.8,
            tilt_right=0.0,
            vertical=0.0,
            yaw_rate=0.0,
            have_target=False,
        )

    # ── Properly lost: hold level, stop climbing, sweep toward last sight ──
    state.searching = True
    direction = 1.0 if state.last_az >= 0.0 else -1.0
    return ServoOutput(
        tilt_fwd=0.0,
        tilt_right=0.0,
        vertical=0.0,
        yaw_rate=direction * cfg.search_yaw_rate,
        have_target=False,
    )
