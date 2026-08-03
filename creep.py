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
    # Damping on the elevation RATE. Without it the vertical channel is P-only:
    # by the time el reaches zero the aircraft has vertical velocity, and hover
    # thrust PRESERVES it because vz is unobservable in steady flight. That is
    # how ALIGN climbed 1.0 -> 6.8 m before committing.
    align_kd_el: float = 0.9
    align_max_vertical: float = 0.35
    align_brake_tilt: float = 0.55   # nose-UP while still closing
    align_closure_stop: float = 0.6  # m/s measured; below this we are "stopped"
    align_az_tol_rad: float = 0.05   # ~3 deg
    align_el_tol_rad: float = 0.06   # ~3.5 deg
    # rad/s; commit only when el is STEADY, not merely small — a zero crossing at
    # speed is not an alignment. MEASURED, not chosen: with az inside a degree and
    # closure at zero, el_rate still jitters +-0.14 around zero, so a 0.05 bound
    # reset the settle timer forever and ALIGN never committed. The point is to
    # reject a SUSTAINED drift, and drift shows as a bias; jitter averages out. The
    # 0.5 s settle window is what actually filters it.
    align_el_rate_tol: float = 0.22
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
    # The period must exceed the ATTITUDE loop's settling time, or the command
    # flips before the aircraft has reached the commanded tilt and it jitters in
    # pitch without translating. At 0.45 s it wandered around the origin for 100 s
    # with range stuck at 11-14 m; the residual asymmetry showed up as drift to
    # y=-6.2 rather than progress. 1.5 s lets each phase actually establish:
    # ~1.08 m/s^2 for 1.5 s gives ~1.6 m/s peak and ~3.6 m per cycle.
    transit_pulse_on_s: float = 1.5
    transit_pulse_off_s: float = 1.5
    transit_tilt: float = 0.45        # forward during the ON phase
    # COAST, do not reverse. Symmetric pulses cancel against drag: the forward
    # phase builds a little speed, drag caps it, and the equal reverse phase pushes
    # the aircraft straight back. In Elodin x oscillated +-0.5 m around the origin
    # for 95 s with range pinned at 12.2-12.5 m. Drag is already the brake — level
    # off and let it work, and displacement per cycle is strictly positive while
    # speed stays bounded by how brief the ON phase is.
    transit_brake_tilt: float = 0.0   # level during the OFF phase
    # MEASURED, not chosen. Creep covers ~0.33 m/s in Elodin, so 12 m to gate 0
    # takes ~36 s — and this was 30, so the timeout fired a few seconds before
    # arrival and threw it back to ALIGN, forever. The failure looked like "does
    # not translate"; it was translating fine and being interrupted.
    transit_kp_el: float = 0.6       # altitude hold only, not gate chasing
    transit_max_vertical: float = 0.15
    transit_timeout_s: float = 120.0  # no pass? back to ALIGN and try again

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
    el_prev: float | None = None
    el_t: float | None = None
    dbg_t: float = -1e9
    el_rate: float = 0.0
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


VERBOSE = True     # phase changes are rare; log them everywhere


def _enter(state: CreepState, phase: str, t: float, why: str = "") -> None:
    if VERBOSE and phase != state.phase:
        print(f"  [PHASE] {state.phase} -> {phase} at t={t:6.2f}"
              f"{'  (' + why + ')' if why else ''}", flush=True)
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
        _enter(state, PIVOT, t, f"gate {gate_index} reached")
        elapsed = 0.0

    # ── SEEK ────────────────────────────────────────────────────────
    if state.phase == SEEK:
        have = (az is not None and el is not None
                and conf >= cfg.acquire_conf
                and abs(el) <= cfg.acquire_max_el_rad)
        if have:
            _enter(state, ALIGN, t, "gate acquired")
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
                _enter(state, SEEK, t, "align timeout, gate lost")
            return CreepCommand(phase=ALIGN)      # hold station, wait for it

        yaw = _clamp(cfg.align_kp_yaw * az,
                     -cfg.align_max_yaw_rate, cfg.align_max_yaw_rate)
        # Differentiate across the gap between SIGHTINGS, not per control cycle.
        # el only changes when a new frame arrives, so a per-cycle derivative is
        # zero between frames and a spike on each update — which pinned el_rate
        # above tolerance and stopped ALIGN ever committing, with az, el and
        # closure all well inside their bounds. servo.py's rate already does this
        # correctly; I did not carry the lesson across.
        if state.el_prev is None or el != state.el_prev:
            gap = (t - state.el_t) if state.el_t is not None else 0.0
            if state.el_prev is not None and gap > 1e-3:
                state.el_rate += 0.4 * ((el - state.el_prev) / gap - state.el_rate)
            state.el_prev = el
            state.el_t = t
        vertical = _clamp(cfg.align_kp_el * el + cfg.align_kd_el * state.el_rate,
                          -cfg.align_max_vertical, cfg.align_max_vertical)
        # Still closing? Kill the speed before doing anything else — the launch
        # puts several m/s into the aircraft that no cruise setting removes.
        fwd = -cfg.align_brake_tilt if closure > cfg.align_closure_stop else 0.0

        az_ok = abs(az) <= cfg.align_az_tol_rad
        el_ok = abs(el) <= cfg.align_el_tol_rad
        rate_ok = abs(state.el_rate) <= cfg.align_el_rate_tol
        clos_ok = closure <= cfg.align_closure_stop
        aligned = az_ok and el_ok and rate_ok and clos_ok
        # `aligned` is an AND of four terms and the timeout message only showed
        # three. Naming the failing one directly beats guessing at it, which has
        # already cost three runs.
        if VERBOSE and t - state.dbg_t >= 2.0:
            state.dbg_t = t
            print(f"  [ALIGN] az={math.degrees(az):+6.2f}{'ok' if az_ok else 'XX'} "
                  f"el={math.degrees(el):+6.2f}{'ok' if el_ok else 'XX'} "
                  f"elrate={state.el_rate:+6.3f}{'ok' if rate_ok else 'XX'} "
                  f"clos={closure:5.2f}{'ok' if clos_ok else 'XX'} "
                  f"settled={state.settled_t:.2f}", flush=True)
        state.settled_t = state.settled_t + dt if aligned else 0.0
        if state.settled_t >= cfg.align_settle_s:
            _enter(state, TRANSIT, t,
                   f"aligned az={math.degrees(az):+.1f} el={math.degrees(el):+.1f} "
                   f"closure={closure:.2f}")
        elif elapsed > cfg.align_timeout_s:
            _enter(state, SEEK, t, f"align timeout az={math.degrees(az):+.1f} "
                                   f"el={math.degrees(el):+.1f} closure={closure:.2f}")
        else:
            return CreepCommand(tilt_fwd=fwd, yaw_rate=yaw, vertical=vertical,
                                phase=ALIGN)

    # ── TRANSIT ─────────────────────────────────────────────────────
    if state.phase == TRANSIT:
        if elapsed > cfg.transit_timeout_s:
            _enter(state, ALIGN, t, "transit timeout, no gate pass")
            return CreepCommand(phase=ALIGN)
        state.pulse_t += dt
        period = cfg.transit_pulse_on_s + cfg.transit_pulse_off_s
        if state.pulse_t >= period:
            state.pulse_t -= period
        on = state.pulse_t < cfg.transit_pulse_on_s
        # STEERING is ignored here: aligned at range, the right action is to fly
        # straight and not chase the bearing, because every gate crash came from
        # reacting to a sighting inside the last two metres.
        #
        # ALTITUDE is different and must NOT be ignored. Commanding vertical=0
        # means hover thrust, and hover thrust holds vertical VELOCITY, not
        # height — the unobservable-vz problem again. The aircraft sank 3.4 m to
        # the floor over 30 s and sat there, which is why it never translated.
        # So the vertical channel stays live, on a small gain and a hard clamp:
        # enough to hold height, far too little to reproduce the elevation
        # walk-up that clamping was introduced to stop.
        vert = 0.0
        if el is not None:
            vert = _clamp(cfg.transit_kp_el * _clamp(el, -0.2, 0.2),
                          -cfg.transit_max_vertical, cfg.transit_max_vertical)
        return CreepCommand(
            tilt_fwd=cfg.transit_tilt if on else -cfg.transit_brake_tilt,
            vertical=vert, phase=TRANSIT, ignore_vision=True)

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
