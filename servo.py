"""Visual servoing: fly toward the gate you SEE. Sim-agnostic, pure Python.

VQ2 removes position, velocity and gate coordinates, so the VQ1 approach
(PD onto a waypoint) has no inputs. What remains is how a human FPV pilot
actually flies: keep the gate centred in the picture and drive forward.

    yaw     turn to centre the gate horizontally  (az -> 0)
    thrust  climb/descend to centre it vertically (el -> el_setpoint)
    pitch   fixed cruise tilt, eased off when poorly lined up
    roll    held level; steering is done with yaw so the gate stays in frame

Input angles are LEVEL-FRAME (bearing.stabilized_bearing), not raw camera
angles. This is not cosmetic: raw elevation is contaminated by the drone's own
nose-down cruise pitch, so servoing on it chases your own attitude — the first
Stage 3a run climbed 5 m above gate 0, pushed it out of the vertical FOV, and
searched forever. De-rotating by the IMU attitude estimate decouples them and
makes el = 0 mean "gate at my altitude".

No altitude sensor is needed anywhere here: "the gate looks too high" is
itself the climb signal. When the gate is lost we cannot know which way is
up-course, so the drone holds level, stops climbing, bleeds off speed and
yaws toward wherever it last saw the gate.
"""

import math
from dataclasses import dataclass

from attitude import clamp


@dataclass(frozen=True)
class ServoConfig:
    # Horizontal: bearing -> yaw rate (rad/s per rad of azimuth error)
    kp_yaw: float = 2.0
    max_yaw_rate: float = 1.6

    # Vertical: elevation error -> normalized vertical effort
    kp_el: float = 1.6
    max_vertical: float = 0.7
    # Angles are LEVEL-frame (see bearing.stabilized_bearing), so 0 means
    # "gate at our own altitude" — no dependence on camera mounting tilt.
    el_setpoint_rad: float = 0.0

    # Forward drive
    cruise_tilt: float = 0.55        # normalized forward tilt when lined up
    min_tilt_frac: float = 0.15      # floor so we never fully stall out
    align_falloff_rad: float = 0.60  # azimuth error that halves forward drive

    # Confidence gating: a low-confidence detection still steers, but gently.
    min_confidence: float = 0.15

    # Target tracking. A real detector is noisy, drops frames and runs slower
    # than the control loop; without these the servo threw away its fix on
    # every missed frame and flip-flopped into search (2/6 gates at 2 deg
    # noise / 30 % drop / 10 Hz). Smooth the bearing, and coast on the last
    # good fix through short gaps.
    ema_alpha: float = 0.35          # per-sighting blend toward the new angle
    hold_s: float = 0.8              # keep steering on a stale fix this long
    hold_drive_decay: float = 0.6    # forward drive multiplier while coasting

    # Behaviour when no gate is visible
    search_yaw_rate: float = 0.35    # rad/s, toward where it was last seen
    # The camera is pitched 20 deg UP with a 58.7 deg vertical FOV, so nothing
    # more than ~9 deg below the flight path is visible at all. A pure yaw
    # sweep can therefore hunt forever past a gate that is simply below us
    # (proved in tests/test_servo.py). Descending raises it into frame.
    search_descend: float = 0.18     # vertical effort, downward
    search_descend_s: float = 3.0    # ...for this long only. An unbounded
                                     # descent flies into the ground and then
                                     # sits there spinning (observed: reached
                                     # z=0 at t=28 and never recovered).
    search_creep: float = 0.38       # NOT a token creep. Forward tilt pitches
                                     # the nose down, swinging the camera from
                                     # ~9 deg of downward view to ~23 deg. A
                                     # search that levels off SHRINKS the very
                                     # FOV it needs to reacquire a low gate.
    lost_coast_s: float = 0.4        # keep driving briefly (gate just left FOV
                                     # because we are about to fly through it)


@dataclass
class ServoState:
    last_az: float = 0.0
    time_since_seen: float = 1e9
    last_t: float | None = None
    searching: bool = False
    az_f: float = 0.0                # smoothed bearing (the working estimate)
    el_f: float = 0.0
    have_fix: bool = False


@dataclass(frozen=True)
class Target:
    """A sighting, in the LEVEL frame. Whatever produces it — truth stand-in
    or real image detector — the servo sees only this."""
    az: float           # radians, + = right of our heading
    el: float           # radians, + = above our altitude
    confidence: float   # 0..1


@dataclass(frozen=True)
class ServoOutput:
    tilt_fwd: float      # body-forward tilt effort, [-1, 1]
    tilt_right: float    # body-right tilt effort, [-1, 1]
    vertical: float      # vertical effort around hover, [-1, 1]
    yaw_rate: float      # rad/s
    have_target: bool


def step(state: ServoState, cfg: ServoConfig, t: float,
         obs: Target | None) -> ServoOutput:
    dt = 0.0 if state.last_t is None else max(0.0, t - state.last_t)
    state.last_t = t

    usable = obs is not None and obs.confidence >= cfg.min_confidence
    if usable:
        # Blend into the running estimate rather than trusting one frame.
        a = cfg.ema_alpha if state.have_fix else 1.0
        state.az_f += a * (obs.az - state.az_f)
        state.el_f += a * (obs.el - state.el_f)
        state.have_fix = True
        state.last_az = state.az_f
        state.time_since_seen = 0.0
        state.searching = False
        confidence = obs.confidence
    else:
        state.time_since_seen += dt
        confidence = 0.0

    # ── Have a fix, fresh or recent: centre it and drive through ────────
    # Coasting on a stale fix is what makes a real (noisy, dropping, slower
    # than the control loop) detector survivable.
    if state.have_fix and state.time_since_seen <= cfg.hold_s:
        stale = not usable
        yaw_rate = clamp(cfg.kp_yaw * state.az_f, -cfg.max_yaw_rate, cfg.max_yaw_rate)

        el_err = state.el_f - cfg.el_setpoint_rad
        vertical = clamp(cfg.kp_el * el_err, -cfg.max_vertical, cfg.max_vertical)

        # Ease off forward drive when poorly lined up, so we turn onto the
        # gate rather than charging past its edge.
        align = 1.0 / (1.0 + (abs(state.az_f) / cfg.align_falloff_rad) ** 2)
        drive = max(cfg.min_tilt_frac, align)
        if stale:
            drive *= cfg.hold_drive_decay
        else:
            drive *= max(cfg.min_tilt_frac, confidence)
        return ServoOutput(
            tilt_fwd=cfg.cruise_tilt * drive,
            tilt_right=0.0,
            vertical=vertical,
            yaw_rate=yaw_rate,
            have_target=True,
        )

    # ── Properly lost: hold level, stop climbing, sweep toward last sight ──
    state.searching = True
    state.have_fix = False
    direction = 1.0 if state.last_az >= 0.0 else -1.0
    descending = state.time_since_seen <= cfg.hold_s + cfg.search_descend_s
    return ServoOutput(
        tilt_fwd=cfg.search_creep,
        tilt_right=0.0,
        vertical=(-cfg.search_descend if descending else 0.0),
        yaw_rate=direction * cfg.search_yaw_rate,
        have_target=False,
    )
