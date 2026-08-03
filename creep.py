"""Gate-at-a-time state machine: align, commit, pass, pivot, repeat.

Nick's design, and it is better than the continuous blend it replaces. Twenty-six
official runs of that blend produced a run of MARGINAL failures — a threshold
nearly right and the outcome flipping on it. Run 27 braked at pitch +27 deg
because the tracked range read 4.3 m against a 4.0 m inhibit; 0.3 m decided the
crash. Phases do not have that property: each does one job and the transitions
are explicit.

The deeper reason it is right: WE KEPT CORRECTING AT RANGES WHERE CORRECTIONS
CANNOT HELP. Inside a metre or two the aircraft cannot meaningfully change where
it goes, but the loops went on commanding roll, yaw and pitch, and that is what
put it into the gate — the elevation walk-up, the 30 deg brake, the impostor
jumps, all of them fired in the last two metres. Align first, then commit and
touch nothing.

    SEEK     hold station, sweep for an orange gate
    ALIGN    kill speed, centre the gate in az and el, no forward drive
    TRANSIT  short forward pulses, attitude level, VISION IGNORED
    PIVOT    hold station, yaw onto the cyan lane
    -> SEEK

TRANSIT ignoring vision is the load-bearing part. Every gate crash we have had
came from reacting to a sighting inside the last two metres. If you are aligned
at five, the correct action is to fly straight and change nothing.

PIVOT is where the corridor earns its keep: the direction signal, used once per
gate, while the aircraft is stationary and can act on it — rather than blended
continuously into a controller already doing four other things.

Motion is a PULSE, not a cruise. Cruise-plus-brake has ended up fast three times
because the brake armed late or missed a threshold by inches; a pulse cannot
exceed its own impulse. Accelerate briefly, decelerate as long, and the aircraft
inches forward with velocity returning to roughly zero each cycle. Slow is then
a structural property rather than a gain to get right.

Pure Python. No numpy, no simulator.
"""

import math
from dataclasses import dataclass

SEEK = "SEEK"
ALIGN = "ALIGN"
TRANSIT = "TRANSIT"
PIVOT = "PIVOT"


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


@dataclass(frozen=True)
class CreepConfig:
    # ── SEEK ────────────────────────────────────────────────────────
    seek_yaw_rate: float = 0.35      # rad/s, slow sweep
    seek_sweep_rad: float = 1.75     # +-100 deg, then reverse. Never a full
                                     # circle: turning further than this lets the
                                     # aircraft face back down the course, and
                                     # the detector will then offer a gate we
                                     # have already passed.
    acquire_conf: float = 0.55
    acquire_max_el_rad: float = 0.31  # 18 deg; excludes the hangar's ceiling gates

    # ── ALIGN ───────────────────────────────────────────────────────
    # No forward drive at all. Speed is killed first, then the gate is centred
    # with yaw and thrust only, so arrival is a decision rather than an accident.
    align_kp_yaw: float = 1.4
    align_max_yaw_rate: float = 0.9
    align_kp_el: float = 0.8
    align_max_vertical: float = 0.35
    align_brake_tilt: float = 0.55   # nose-UP while still closing
    align_closure_stop: float = 0.6  # m/s measured; below this we are "stopped"
    align_az_tol_rad: float = 0.05   # ~3 deg
    align_el_tol_rad: float = 0.06   # ~3.5 deg
    align_settle_s: float = 0.5      # hold the tolerance this long before commit
    align_timeout_s: float = 25.0    # give up and re-seek

    # ── TRANSIT ─────────────────────────────────────────────────────
    # Pulse: accelerate, then decelerate for as long, so velocity returns to
    # about zero each cycle and displacement accumulates slowly.
    # The two impulses MUST match: on_s * tilt == off_s * brake. They did not
    # (0.45*0.45 against 0.55*0.45), so every cycle netted a backwards impulse of
    # -0.045 and the aircraft reversed 75 m down the course in Elodin. The test
    # reported that mean and it was read as "close to zero"; a systematically
    # negative mean is not close to zero, it is a direction.
    transit_pulse_on_s: float = 0.45
    transit_pulse_off_s: float = 0.45
    transit_tilt: float = 0.45        # forward during the ON phase
    transit_brake_tilt: float = 0.45  # nose-UP during the OFF phase
    transit_timeout_s: float = 30.0   # no pass? back to ALIGN and try again

    # ── PIVOT ───────────────────────────────────────────────────────
    pivot_kp_yaw: float = 1.2
    pivot_max_yaw_rate: float = 0.8
    pivot_tol_rad: float = 0.09      # ~5 deg
    pivot_timeout_s: float = 6.0     # no lane? carry on and seek anyway
    pivot_min_s: float = 0.4         # always clear the gate structure first


@dataclass
class CreepState:
    phase: str = SEEK
    phase_t0: float | None = None
    last_t: float | None = None
    sweep_pos: float = 0.0
    sweep_dir: float = 1.0
    settled_t: float = 0.0
    gate_index: int | None = None
    pulse_t: float = 0.0


@dataclass(frozen=True)
class CreepCommand:
    tilt_fwd: float = 0.0
    tilt_right: float = 0.0
    yaw_rate: float = 0.0
    vertical: float = 0.0
    phase: str = SEEK
    ignore_vision: bool = False


def _enter(state: CreepState, phase: str, t: float) -> None:
    state.phase = phase
    state.phase_t0 = t
    state.settled_t = 0.0
    state.pulse_t = 0.0
    if phase == SEEK:
        state.sweep_pos = 0.0


def step(state: CreepState, cfg: CreepConfig, t: float,
         az: float | None, el: float | None, conf: float, closure: float,
         lane_rad: float | None, gate_index: int | None) -> CreepCommand:
    """One cycle.

    `az`/`el` are the gate's level-frame bearings (None when unseen), `conf` its
    confidence, `closure` the measured range decrease in m/s (positive closing),
    `lane_rad` the cyan lane bearing, `gate_index` the sim's active gate.
    """
    dt = 0.0 if state.last_t is None else max(0.0, t - state.last_t)
    state.last_t = t
    if state.phase_t0 is None:
        state.phase_t0 = t
    elapsed = t - state.phase_t0

    # The sim advancing the gate index is the only unambiguous progress signal
    # VQ2 gives us, and it is what ends a TRANSIT.
    passed = (gate_index is not None and state.gate_index is not None
              and gate_index != state.gate_index)
    if gate_index is not None:
        state.gate_index = gate_index

    if state.phase == TRANSIT and passed:
        _enter(state, PIVOT, t)
        elapsed = 0.0

    # ── SEEK ────────────────────────────────────────────────────────
    if state.phase == SEEK:
        have = (az is not None and el is not None
                and conf >= cfg.acquire_conf
                and abs(el) <= cfg.acquire_max_el_rad)
        if have:
            _enter(state, ALIGN, t)
        else:
            state.sweep_pos += state.sweep_dir * cfg.seek_yaw_rate * dt
            if abs(state.sweep_pos) >= cfg.seek_sweep_rad:
                state.sweep_dir = -state.sweep_dir
            return CreepCommand(yaw_rate=state.sweep_dir * cfg.seek_yaw_rate,
                                phase=SEEK)

    # ── ALIGN ───────────────────────────────────────────────────────
    if state.phase == ALIGN:
        if az is None or el is None:
            if elapsed > cfg.align_timeout_s:
                _enter(state, SEEK, t)
            return CreepCommand(phase=ALIGN)      # hold station, wait for it

        yaw = _clamp(cfg.align_kp_yaw * az,
                     -cfg.align_max_yaw_rate, cfg.align_max_yaw_rate)
        vertical = _clamp(cfg.align_kp_el * el,
                          -cfg.align_max_vertical, cfg.align_max_vertical)
        # Still closing? Kill the speed before doing anything else — the launch
        # puts several m/s into the aircraft that no cruise setting removes.
        fwd = -cfg.align_brake_tilt if closure > cfg.align_closure_stop else 0.0

        aligned = (abs(az) <= cfg.align_az_tol_rad
                   and abs(el) <= cfg.align_el_tol_rad
                   and closure <= cfg.align_closure_stop)
        state.settled_t = state.settled_t + dt if aligned else 0.0
        if state.settled_t >= cfg.align_settle_s:
            _enter(state, TRANSIT, t)
        elif elapsed > cfg.align_timeout_s:
            _enter(state, SEEK, t)
        else:
            return CreepCommand(tilt_fwd=fwd, yaw_rate=yaw, vertical=vertical,
                                phase=ALIGN)

    # ── TRANSIT ─────────────────────────────────────────────────────
    if state.phase == TRANSIT:
        if elapsed > cfg.transit_timeout_s:
            _enter(state, ALIGN, t)
            return CreepCommand(phase=ALIGN)
        state.pulse_t += dt
        period = cfg.transit_pulse_on_s + cfg.transit_pulse_off_s
        if state.pulse_t >= period:
            state.pulse_t -= period
        on = state.pulse_t < cfg.transit_pulse_on_s
        # Vision is IGNORED here: aligned at range, the right action is to fly
        # straight and change nothing. Every gate crash came from reacting to a
        # sighting inside the last two metres.
        return CreepCommand(
            tilt_fwd=cfg.transit_tilt if on else -cfg.transit_brake_tilt,
            phase=TRANSIT, ignore_vision=True)

    # ── PIVOT ───────────────────────────────────────────────────────
    if elapsed < cfg.pivot_min_s:
        return CreepCommand(phase=PIVOT)          # clear the structure first
    if lane_rad is None:
        if elapsed > cfg.pivot_timeout_s:
            _enter(state, SEEK, t)
        return CreepCommand(phase=PIVOT)
    if abs(lane_rad) <= cfg.pivot_tol_rad or elapsed > cfg.pivot_timeout_s:
        _enter(state, SEEK, t)
        return CreepCommand(phase=SEEK)
    return CreepCommand(
        yaw_rate=_clamp(cfg.pivot_kp_yaw * lane_rad,
                        -cfg.pivot_max_yaw_rate, cfg.pivot_max_yaw_rate),
        phase=PIVOT)
