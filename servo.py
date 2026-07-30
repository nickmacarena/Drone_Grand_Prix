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
    kd_yaw: float = 0.30      # same idea, applied to the azimuth bearing
    max_yaw_rate: float = 1.6
    # Slew limit. Run 8 acquired gate 1 at u=609 (42 deg right, at the FOV edge
    # — the course turns hard right after gate 0) and commanded 1.47 rad/s in a
    # SINGLE step, 1 m past the gate plane with gate 0's posts still alongside.
    # It clipped one and inverted to roll +174. The target was right; the
    # violence was not.
    max_yaw_accel: float = 2.2       # rad/s^2

    # After clearing a gate, fly straight briefly before steering. At 1 m past
    # the gate plane the structure is still beside us, and that is the worst
    # possible moment for a hard turn. Tracking continues throughout — only the
    # steering is held off.
    # Kept SHORT. Elodin (course "vq2turn") shows gate 1's bearing growing from
    # 42 to 60 deg — outside the 90 deg HFoV — within ~5 m of the gate-0 plane.
    # At ~7 m/s a 0.45 s inhibit burns 3 m of exactly the window where the next
    # gate is still visible, so it caused the very loss it was meant to prevent.
    # The slew limit above is what keeps the turn from snapping; this only needs
    # to carry us past the gate plane itself.
    # MEASURED, not chosen: 0.25 s took the VQ1 replica 6/6 -> 5/6 and vq2turn
    # 4/4 -> 2/4, while 0.15 holds both. The window is genuinely knife-edged
    # because gate 1's bearing leaves the FOV within ~2 m of the gate-0 plane,
    # so every extra 0.1 s of not steering costs about a metre of that budget.
    # The post-pass tumbles in runs 15/16 are therefore NOT caused by too little
    # clearance — widening this only loses the next gate. Do not raise it without
    # rerunning both courses.
    clear_gate_s: float = 0.15

    # Gates can legitimately sit near the FOV edge (gate 1 is ~42 deg off), so
    # the first sweep must be able to reach past that.
    hint_max_age_s: float = 2.5      # how long a next-gate hint stays useful

    # Vertical: elevation error -> normalized vertical effort.
    # kd_el damps using the MEASURED rate of change of the elevation bearing.
    # This is the one rate signal VQ2 actually gives us: it comes from
    # differentiating vision, not from integrating accelerometers, so the
    # unobservability that makes vz_est useless in steady flight (constant
    # velocity => zero net accel => indistinguishable from hover) does not
    # apply. Without it the vertical loop is pure proportional on a
    # second-order plant and MUST overshoot — official run 6 climbed to gate
    # 0's centre, sailed past it, and clipped the top bar.
    kp_el: float = 0.8
    kd_el: float = 1.3
    max_vertical: float = 0.7
    # Angles are LEVEL-frame (see bearing.stabilized_bearing), so 0 means
    # "gate at our own altitude" — no dependence on camera mounting tilt.
    el_setpoint_rad: float = 0.0

    # Lateral: roll toward the gate as well as yawing onto it.
    #
    # Until now tilt_right was hard 0.0 in every branch — steering was yaw-only,
    # so direction changes had to wait for the nose to come round and then for
    # forward tilt to push the new way. Coming out of a turn the aircraft keeps
    # its old momentum and slides wide: in Elodin "vq2turn" it lined up on gate
    # 1 perfectly at 6.4 m (az=-0.3 deg) and still crossed the plane 1.0 m off
    # centre, outside the 0.75 m half-opening. Roll gives crossrange authority
    # directly instead of via the heading.
    kp_lat: float = 0.9
    max_lat: float = 0.60      # was 0.35, giving only ~1.2 m/s^2 of crossrange

    # Forward drive
    cruise_tilt: float = 0.55        # normalized forward tilt when lined up
    min_tilt_frac: float = 0.15      # floor so we never fully stall out
    align_falloff_rad: float = 0.60  # azimuth error that halves forward drive

    # BRAKING. Official run 12 acquired gate 1 at 33 deg right and swung it
    # cleanly through centre to 38 deg left — the steering worked — but
    # translated out of the window while turning. Nick: "it flew through gate 0
    # too fast, so even though it turned to face the next gate, the momentum
    # carried gate 1 out of its pov."
    #
    # This is not a gain shortfall, it is geometry. Turning an 8 m/s velocity
    # vector through 42 deg needs dv = 2*v*sin(21 deg) = 5.7 m/s; with 1.2 m/s^2
    # of lateral authority that takes 2.7 s, and gate 1's bearing leaves the FOV
    # in well under 1 s. You cannot out-gain it. You have to shed speed.
    #
    # Note the speed is NOT ours to avoid by cruising gentler: MAX_TILT_RAD is
    # 20 deg, so cruise_tilt 0.55 commands 11 deg => 1.9 m/s^2 => ~2.9 m/s by
    # gate 0, yet it arrives at ~8 m/s. The ramp launch supplies the rest. The
    # only lever we have is to actively brake, which the old
    # drive = max(min_tilt_frac, align) could never do — it kept forward tilt
    # positive however far off-axis the target was.
    brake_az_rad: float = 0.45       # beyond ~26 deg off axis, decelerate
    brake_tilt: float = 0.45         # nose-UP fraction while braking

    # Braking AFTER the pass is late — the momentum is already there. We capture
    # the next gate's bearing as a hint while still flying the current one (see
    # PolarityCheck's neighbour, hint_az), so when the next gate is known to be
    # far off axis we can arrive slower and ready to turn. Only applied close in,
    # so the whole approach is not crawled.
    pre_brake_range_m: float = 7.0
    pre_brake_frac: float = 0.45

    # Confidence gating: a low-confidence detection still steers, but gently.
    min_confidence: float = 0.15
    # ACQUIRING a track is a stronger claim than maintaining one, so it needs
    # more evidence. A marginal blob may keep an established track alive; it
    # may not start one.
    acquire_confidence: float = 0.55
    # The hangar has CEILING-MOUNTED gates (visible in run 6 frame 0), and the
    # detector reports the biggest orange blob with no notion of which gates
    # belong to the course. Run 15 locked onto one and climbed at +14.3 m/s
    # (thr 0.39, a_up +45 on impact); run 13 saw the same thing at v=34.9.
    #
    # A course gate is roughly at our own altitude — gate 0 measures +14 deg at
    # the gun — so refuse to ACQUIRE on anything steeper. Maintaining an
    # existing track is unrestricted, since a gate legitimately rises in view as
    # we close on it.
    acquire_max_el_rad: float = 0.44   # 25 deg

    # ── Track continuity ────────────────────────────────────────────────
    # The detector reports the best orange blob in EACH FRAME independently,
    # with no memory. Official run 6 showed what that costs: three consecutive
    # sightings read rng 4.3 -> 1.0 -> 12.6 m and u 331 -> 403 -> 621 px. No
    # gate moves like that; those were different objects. The servo chased the
    # last one to the frame edge at a saturated 92 deg/s, lost it, and span.
    #
    # A gate is a physical object, so its bearing and range must evolve
    # continuously. Sightings that violate that are rejected as mistaken
    # identity. Bounds are generous — this rejects the impossible, not the
    # merely surprising.
    # Bounds must be TIGHTER than the drone's own achievable motion or they
    # reject nothing useful. Run 6's u=532/rng=16.6 sighting passed the first
    # attempt at this gate because max_az_rate was 2.5 rad/s — larger than the
    # 1.6 rad/s the aircraft can even yaw.
    max_az_step_rad: float = 0.12    # instantaneous slack (detector jitter)
    max_az_rate: float = 1.8         # rad/s of plausible bearing change
    max_closing_speed: float = 12.0  # m/s of plausible range change
    range_slack_m: float = 1.0       # plus fixed slack (width estimate is coarse)
    # If the track keeps rejecting everything, the track itself is probably
    # wrong — drop it and re-acquire rather than stay blind forever.
    reject_timeout_s: float = 0.7
    rate_alpha: float = 0.4          # smoothing on the differentiated bearings

    # Target tracking. A real detector is noisy, drops frames and runs slower
    # than the control loop; without these the servo threw away its fix on
    # every missed frame and flip-flopped into search (2/6 gates at 2 deg
    # noise / 30 % drop / 10 Hz). Smooth the bearing, and coast on the last
    # good fix through short gaps.
    ema_alpha: float = 0.35          # per-sighting blend toward the new angle
    hold_s: float = 0.8              # keep steering on a stale fix this long
    hold_drive_decay: float = 0.6    # forward drive multiplier while coasting

    # Behaviour when no gate is visible.
    #
    # Official run 7 passed gate 0 and then never found gate 1. The old search
    # yawed in ONE direction forever ("toward where it was last seen"), so it
    # turned ~147 deg off course, chased one of the many other red gates in the
    # hangar, lost that too, and kept turning. A course continues roughly
    # FORWARD, so the heading we passed the gate on is the best prior we have.
    # Sweep outward from it, alternating and widening, instead of committing to
    # one direction.
    search_yaw_rate: float = 0.55    # rad/s while sweeping
    sweep_limit_rad: float = 1.15    # first sweep: +/- 66 deg from the prior
    sweep_growth: float = 1.6        # widen on each reversal
    sweep_limit_max_rad: float = 3.4 # eventually cover the full circle
    # The camera is pitched 20 deg UP with a 58.7 deg vertical FOV, so nothing
    # more than ~9 deg below the flight path is visible at all. A pure yaw
    # sweep can therefore hunt forever past a gate that is simply below us
    # (proved in tests/test_servo.py). Descending raises it into frame.
    # The camera cannot see below ~9 deg, so sinking DOES raise a low gate into
    # frame — but it must not be a one-way trip. VQ2 gives no altitude and
    # vz_est is unobservable in steady flight, so hover thrust PRESERVES any
    # descent rather than arresting it: run 7 sank from -1.1 to -9.4 m/s while
    # searching and never recovered. So oscillate instead of descending, and
    # spend equal time above the entry altitude, which nets to zero drift.
    search_descend: float = 0.18     # peak vertical effort, either direction
    # Period matters as much as amplitude. At 2.4 s the excursion is only
    # ~0.65 m, which nets zero drift but cannot find a gate below the FOV's
    # -9 deg floor — the VQ1 replica descends 8.6 m between gates 1 and 2 and
    # regressed 6/6 -> 2/6 when this was too short. 6 s gives a several-metre
    # sweep down (the blind side) and then recovers it. sin() starts negative,
    # so the DOWN half comes first, which is where gates hide.
    search_vert_period_s: float = 6.0  # full down-then-up cycle
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
    hint_az: float | None = None     # bearing of a SECOND gate seen while
    hint_t: float = 0.0              # tracking the current one — i.e. the next
    clear_until: float | None = None  # steer-inhibit deadline after a gate pass
    yaw_cmd: float = 0.0             # previous yaw command, for the slew limit
    sweep_pos: float = 0.0           # commanded yaw accumulated while searching
    sweep_dir: float = 1.0
    sweep_limit: float | None = None
    search_t: float = 0.0            # time spent in the current search
    az_rate: float = 0.0             # d(az)/dt, smoothed — the damping signal
    el_rate: float = 0.0
    rng_f: float | None = None       # smoothed range, for continuity gating
    t_last_accept: float | None = None
    rejecting_since: float | None = None
    rejected: int = 0                # diagnostic: how many sightings refused


@dataclass(frozen=True)
class Target:
    """A sighting, in the LEVEL frame. Whatever produces it — truth stand-in
    or real image detector — the servo sees only this."""
    az: float           # radians, + = right of our heading
    el: float           # radians, + = above our altitude
    confidence: float   # 0..1
    range_m: float | None = None   # from apparent width; None disables the
                                   # range half of the continuity gate


@dataclass(frozen=True)
class ServoOutput:
    tilt_fwd: float      # body-forward tilt effort, [-1, 1]
    tilt_right: float    # body-right tilt effort, [-1, 1]
    vertical: float      # vertical effort around hover, [-1, 1]
    yaw_rate: float      # rad/s
    have_target: bool
    braking: bool = False   # decelerating into a turn. WHEN to brake is servo
                            # policy; HOW HARD is plant-specific, because the
                            # two sims scale tilt_fwd differently — Elodin feeds
                            # it straight to its mixer while the official sim
                            # multiplies by MAX_TILT_RAD. Setting brake_tilt to
                            # 0.85 for the sim's sake took the VQ1 replica from
                            # 6/6 to 2/6 in Elodin, which is the mismatch, not a
                            # disagreement about policy.


def _slew(state: ServoState, cfg: ServoConfig, want: float, dt: float) -> float:
    """Rate-limit the yaw command so no single sighting can snap the aircraft."""
    if dt <= 0.0:
        return state.yaw_cmd
    step = cfg.max_yaw_accel * dt
    state.yaw_cmd += clamp(want - state.yaw_cmd, -step, step)
    return state.yaw_cmd


def _continuous(state: ServoState, cfg: ServoConfig, t: float,
                obs: "Target") -> bool:
    """Could this sighting be the same physical gate as the current track?

    Rejects mistaken identity, not honest noise: the bounds allow a fast gate
    crossing the frame and a fast closure, and only refuse motion no rigid
    object could produce in the elapsed time.
    """
    gap = 0.0 if state.t_last_accept is None else max(0.0, t - state.t_last_accept)

    if abs(obs.az - state.az_f) > cfg.max_az_step_rad + cfg.max_az_rate * gap:
        return False

    if (state.rng_f is not None and obs.range_m is not None
            and math.isfinite(obs.range_m)):
        if abs(obs.range_m - state.rng_f) > (cfg.range_slack_m
                                             + cfg.max_closing_speed * gap):
            return False
    return True


def step(state: ServoState, cfg: ServoConfig, t: float,
         obs: Target | None) -> ServoOutput:
    dt = 0.0 if state.last_t is None else max(0.0, t - state.last_t)
    state.last_t = t

    usable = obs is not None and obs.confidence >= cfg.min_confidence
    if usable and not state.have_fix:
        # No track yet: acquiring needs stronger evidence than maintaining, and
        # must not start on something far above the course (see acquire_max_el).
        usable = (obs.confidence >= cfg.acquire_confidence
                  and abs(obs.el) <= cfg.acquire_max_el_rad)
    if usable and state.have_fix and not _continuous(state, cfg, t, obs):
        # Sighting is inconsistent with the track: almost certainly a
        # different object. Coast on the track instead of jumping to it.
        usable = False
        state.rejected += 1
        # A confident sighting that is NOT the gate we are tracking is, most
        # often, the NEXT gate. Runs 7-9 all saw gate 1 this way while still
        # locked on gate 0 (u=509..609, conf 0.74-1.00) and threw it away, then
        # had no idea which way to turn once gate 0 was behind them.
        if obs.confidence >= cfg.acquire_confidence:
            state.hint_az = obs.az
            state.hint_t = t
        if state.rejecting_since is None:
            state.rejecting_since = t
        elif t - state.rejecting_since > cfg.reject_timeout_s:
            state.have_fix = False       # the track was the wrong one
            state.rng_f = None
            state.rejecting_since = None
    elif usable:
        state.rejecting_since = None

    if usable:
        # Blend into the running estimate rather than trusting one frame.
        a = cfg.ema_alpha if state.have_fix else 1.0
        az_prev, el_prev = state.az_f, state.el_f
        state.az_f += a * (obs.az - state.az_f)
        state.el_f += a * (obs.el - state.el_f)
        # Bearing rates, from the vision stream itself. Smoothed, because
        # differentiating a noisy signal amplifies the noise.
        gap = None if state.t_last_accept is None else t - state.t_last_accept
        if gap and gap > 1e-3:
            b = cfg.rate_alpha
            state.az_rate += b * ((state.az_f - az_prev) / gap - state.az_rate)
            state.el_rate += b * ((state.el_f - el_prev) / gap - state.el_rate)
        if obs.range_m is not None and math.isfinite(obs.range_m):
            r = obs.range_m
            state.rng_f = r if state.rng_f is None else (
                state.rng_f + cfg.ema_alpha * (r - state.rng_f))
        state.have_fix = True
        state.t_last_accept = t
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
        want_yaw = clamp(cfg.kp_yaw * state.az_f + cfg.kd_yaw * state.az_rate,
                         -cfg.max_yaw_rate, cfg.max_yaw_rate)
        if state.clear_until is not None and t < state.clear_until:
            want_yaw = 0.0            # clearing the gate we just passed
        yaw_rate = _slew(state, cfg, want_yaw, dt)

        el_err = state.el_f - cfg.el_setpoint_rad
        vertical = clamp(cfg.kp_el * el_err + cfg.kd_el * state.el_rate,
                         -cfg.max_vertical, cfg.max_vertical)

        # Ease off forward drive when poorly lined up, so we turn onto the
        # gate rather than charging past its edge.
        align = 1.0 / (1.0 + (abs(state.az_f) / cfg.align_falloff_rad) ** 2)
        drive = max(cfg.min_tilt_frac, align)
        if stale:
            drive *= cfg.hold_drive_decay
        else:
            drive *= max(cfg.min_tilt_frac, confidence)
        # NOTE: pre-emptive braking on a next-gate hint was tried here and
        # REVERTED. It looked principled — we know a sharp turn is coming before
        # we reach the current gate — but it took the VQ1 replica from 6/6 to
        # 2/6, and I had only tested it on vq2turn. On a course where the
        # detector regularly sees distant gates at wide bearings, the hint is
        # almost always "sharp turn ahead", so it crawled into every gate.
        # Any revival needs the hint to be attributable to the NEXT gate
        # specifically, which nothing currently establishes.
        # Far off axis: stop adding speed and start removing it, so the turn can
        # actually be flown before the target leaves the FOV.
        braking = abs(state.az_f) > cfg.brake_az_rad
        fwd = -cfg.brake_tilt if braking else cfg.cruise_tilt * drive
        lateral = clamp(cfg.kp_lat * state.az_f, -cfg.max_lat, cfg.max_lat)
        if state.clear_until is not None and t < state.clear_until:
            lateral = 0.0          # not while the gate structure is alongside
        return ServoOutput(
            tilt_fwd=fwd,
            tilt_right=lateral,
            vertical=vertical,
            yaw_rate=yaw_rate,
            have_target=True,
            braking=braking,
        )

    # ── Properly lost: hold level, stop climbing, sweep toward last sight ──
    if not state.searching:                      # search just began
        state.sweep_pos = 0.0
        state.sweep_limit = cfg.sweep_limit_rad
        state.search_t = 0.0
        # Start toward wherever the gate last was; with no history, go right.
        state.sweep_dir = 1.0 if state.last_az >= 0.0 else -1.0
    state.searching = True
    state.have_fix = False
    state.az_rate = 0.0
    state.el_rate = 0.0
    state.search_t += dt

    # Bounded, widening sweep about the heading we started searching from.
    limit = state.sweep_limit or cfg.sweep_limit_rad
    state.sweep_pos += state.sweep_dir * cfg.search_yaw_rate * dt
    if abs(state.sweep_pos) >= limit:
        state.sweep_dir = -state.sweep_dir
        state.sweep_limit = min(limit * cfg.sweep_growth, cfg.sweep_limit_max_rad)

    # Vertical search oscillates, so a fruitless search does not walk the
    # aircraft into the floor (see search_descend).
    phase = 2.0 * math.pi * state.search_t / max(cfg.search_vert_period_s, 1e-3)
    want_yaw = state.sweep_dir * cfg.search_yaw_rate
    if state.clear_until is not None and t < state.clear_until:
        want_yaw = 0.0
    return ServoOutput(
        tilt_fwd=cfg.search_creep,
        tilt_right=0.0,
        vertical=-cfg.search_descend * math.sin(phase),
        yaw_rate=_slew(state, cfg, want_yaw, dt),
        have_target=False,
    )


@dataclass
class PolarityCheck:
    """Evidence that steering actually reduces a tracked gate's azimuth.

    Runs 6-10 all failed the same way: YAW_SIGN was +1, which textbook FRD says
    is correct for "turn right" and this sim says is not, so the azimuth loop sat
    in positive feedback. Five official runs to notice. The relationship is
    directly measurable in flight — commanding yaw toward a gate must SHRINK its
    azimuth — so a wrong constant should be self-correcting, not a lost run.

    Lives here rather than in the controller because it is a statement about
    bearings and steering, and because that keeps it testable without pymavlink.
    """
    wrong: int = 0
    total: int = 0
    decided: bool = False

    # Thresholds: the commanded yaw must dominate the bearing change caused by
    # translation, and there must be a measurable response to judge.
    #
    # The range gate is the one that was missing and it invalidated run 11's
    # verdict. Translation contributes v*sin(az)/r to the bearing rate; at
    # rng=3.7 m, 37 deg off axis, ~8 m/s that is ~1.3 rad/s, triple the
    # commanded yaw. Close in, azimuth moves because we are flying past the
    # gate, so it says nothing about the sign. Only judge from far away.
    min_yaw_rate: float = 0.30
    min_az_rate: float = 0.05
    min_range_m: float = 15.0
    samples: int = 12
    margin: float = 0.60


def check_polarity(chk: PolarityCheck, yaw_cmd: float, az_rate: float,
                   range_m: float | None = None) -> int:
    """Accumulate evidence. Returns -1 to flip, +1 confirmed, 0 undecided.

    Correct steering makes yaw_cmd and d(az)/dt OPPOSITE in sign. Only samples
    far enough out that rotation, not translation, drives the bearing.
    """
    if chk.decided:
        return 0
    if range_m is None or not math.isfinite(range_m) or range_m < chk.min_range_m:
        return 0            # too close: translation swamps rotation
    if abs(yaw_cmd) < chk.min_yaw_rate or abs(az_rate) < chk.min_az_rate:
        return 0            # a zero response would read as "correct"
    chk.total += 1
    if yaw_cmd * az_rate > 0.0:
        chk.wrong += 1
    if chk.total < chk.samples:
        return 0
    chk.decided = True
    return -1 if (chk.wrong / chk.total) >= chk.margin else +1


def yaw_convention(samples, fx: float = 320.0, cx: float = 320.0,
                   min_span_deg: float = 5.0, min_samples: int = 6,
                   tol: float = 0.5):
    """Regress the gate's BEARING against yaw over a pure rotation.

    `samples` is [(yaw_rad, u_px), ...] gathered while rotating in place with no
    translation (controller_vq2 MISSION=yawtest). Returns
    (verdict, slope, span_deg, note), verdict being:
        "standard"     slope ~ -1: rotating right sweeps the scene left
        "inverted"     slope ~ +1
        "inconclusive" too few points, too little rotation, or a slope whose
                       MAGNITUDE is wrong — i.e. something other than rotation
                       moved the gate

    Regressing bearing rather than raw pixels is what makes the magnitude test
    meaningful: u = cx + fx*tan(bearing) is nonlinear, so du/dyaw depends on how
    far round you turned (-320 px/rad near centre, -487 across 60 deg), and no
    fixed pixel tolerance can be right for every span. In bearing space pure
    rotation gives exactly -1 whatever the span.

    The magnitude test matters more than the sign. A gate close enough to fill
    the frame has its centroid pinned by the frame edges and barely moves under
    rotation: the first offline attempt at this returned a confident "standard"
    from one pair in which 37.7 deg of yaw moved u by 2.9 px, where rotation
    demands ~247. The sign was right and the datum was still worthless.
    """
    if len(samples) < min_samples:
        return "inconclusive", 0.0, 0.0, f"only {len(samples)} sightings"
    ys = [p[0] for p in samples]
    bs = [math.atan2((p[1] - cx) / fx, 1.0) for p in samples]
    n = len(samples)
    my, mb = sum(ys) / n, sum(bs) / n
    sxx = sum((y - my) ** 2 for y in ys)
    span = math.degrees(max(ys) - min(ys))
    if sxx < 1e-9 or span < min_span_deg:
        return "inconclusive", 0.0, span, f"yaw only moved {span:.1f} deg"
    slope = sum((y - my) * (b - mb) for y, b in zip(ys, bs)) / sxx
    if abs(abs(slope) - 1.0) > tol:
        return ("inconclusive", slope, span,
                f"|d(bearing)/d(yaw)| = {abs(slope):.2f}, expected ~1.0; "
                f"something other than rotation moved the gate")
    return ("standard" if slope < 0 else "inverted"), slope, span, ""


def gate_passed(state: ServoState, t: float | None = None,
                cfg: ServoConfig | None = None) -> None:
    """Called when the sim's active_gate_index advances.

    Proof we are through, and the moment the old track becomes meaningless: it
    describes a gate now behind us. Re-centre the search prior on the current
    heading, because the next gate is most likely ahead.
    """
    state.have_fix = False
    state.rng_f = None
    state.az_f = 0.0
    state.el_f = 0.0
    state.az_rate = 0.0
    state.el_rate = 0.0
    state.t_last_accept = None
    state.rejecting_since = None
    state.searching = False          # forces a fresh, re-centred sweep
    state.sweep_pos = 0.0
    state.sweep_limit = None
    state.search_t = 0.0
    c = cfg or ServoConfig()
    # Head toward the next-gate hint if we have a fresh one; otherwise keep the
    # last bearing we saw. Zeroing this (as the first version did) destroys the
    # only clue we have about which way the course turns.
    if (state.hint_az is not None and t is not None
            and t - state.hint_t <= c.hint_max_age_s):
        state.last_az = state.hint_az
    state.hint_az = None
    if t is not None:
        state.clear_until = t + c.clear_gate_s
