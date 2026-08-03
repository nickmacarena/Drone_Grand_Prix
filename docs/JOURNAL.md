# Journal

Append-only log of sessions, decisions, and reasoning. Never edit past entries.

---

## 2026-03-25 to 2026-03-26 — Project Setup

- Created repo, added README and VQ1 tech specs PDF
- Set up PX4 SITL + Gazebo on macOS; build issues documented in LOCAL_DEV_SETUP.md
- Built `connect.py` — confirmed MAVLink telemetry over UDP port 14550
- Chose port 14550 (GCS link) over 14540 — 14540 gets a different MAVLink component

---

## 2026-03-31 — Control Baseline + Pipeline Design

- Built `control.py` — arm, takeoff, NED position setpoints, land; confirmed drone moves in Gazebo
- Read tech spec carefully; decided not to target competition sim interface yet — focus on PX4 SITL for the next month while competition sim is unavailable
- Designed pipeline architecture and documented in ARCHITECTURE.md
- Created `docs/` folder structure

---

## 2026-04-22 — Competition FAQ Changes Architecture

- New FAQ confirms control output is **throttle/roll/pitch/yaw only** — not position setpoints
- This means we need a position→attitude PID controller; `SET_ATTITUDE_TARGET` is the real interface
- Previous `control.py` using `set_position_ned` is only useful for PX4 SITL dev, not competition
- Updated ARCHITECTURE.md: controller now outputs `AttitudeCommand`, added `AttitudeCommand` type, reordered phases to prioritize controller tuning
- Other FAQ takeaways: Windows only for competition sim, monocular vision (no depth), gates in order, < 10 gates for VQ1, full 3D with elevation changes, VQ1 is completion-focused

---

## 2026-04-24 — Windows VM Validation

- Validated Parallels Windows ARM64 VM end-to-end ahead of competition sim drop
- Pipeline tests pass on Windows; MAVSDK loads; `connect.py` hangs gracefully without sim
- **Decision: x64 Python, not ARM64.** `grpcio` (MAVSDK dependency) has no prebuilt `win_arm64` wheels at any version — ARM64 Python required compiling from source via MSVC. x64 Python via Windows-on-ARM emulation has full wheel coverage and matches what the competition sim almost certainly expects. Installed at `%LOCALAPPDATA%\Programs\Python\Python314-x64\`.
- Spec calls for Python 3.14.2; installed 3.14.4 (latest patch). Flag if the sim's anti-cheat validates exact patch version.
- File sync: repo lives on Parallels shared folder (`\\psf\Home\Documents\Drone_Grand_Prix`), which IS the Mac home directory — Mac edits are instantly visible in the VM. No GitHub round-trips needed for sync; still push for backup.
- Required `safe.directory` git config for the UNC path (Parallels shares don't record file ownership).
- Latency benchmark: pipeline tick 20us median / 80us p99 — ~100x headroom at 120Hz. CPU is not a concern.
- Found Windows timer resolution issue: `asyncio.sleep(1/120)` overshoots ~8ms. Not VM-specific. Fix later via `timeBeginPeriod(1)` or by driving the loop off telemetry events instead of sleep.
- Open risk: sim's GPU framerate under Parallels — unknown until sim drops. Contingency would be bare-metal Windows.

---

## 2026-06-03 — VQ1 Simulator Released, Major Rewrite

- Simulator released, downloaded and extracted `PyAIPilotExample` reference code
- Reading the example overturned several prior assumptions:
    - Sim is pymavlink-based, not MAVSDK
    - Position/velocity setpoints **are** supported (April FAQ misled us)
    - Sim provides full track layout via `ENCAPSULATED_DATA` — vision not required for VQ1
    - Sim provides `active_gate_index` via race status — we don't sequence gates ourselves
    - Control loop is 250 Hz, attitude commands use body rates (not absolute angles)
- Rewrote codebase from scratch to match example structure: `mavlink_rx/tx`, `vision_rx`, `timesync`, `controller`, `setup`, `main`, `state`
- Moved all pre-sim code into `legacy/` — kept for reference, not for extension
- VQ1 baseline strategy: velocity vector toward active gate, no vision
- ARCHITECTURE.md rewritten to reflect actual sim interface

---

## 2026-06-03 — VM Migrated to VMware Fusion, Networking Verified

- Parallels trial expired; migrated to VMware Fusion 26H1 (free for personal use)
- Windows 11 ARM 25H2 VM, 4 CPU / 8 GB RAM, NAT networking
- VM IP `192.168.129.128`, Mac reachable from VM at `192.168.129.1` (Fusion's NAT host route)
- Verified bidirectional UDP on ports 14550 (MAVLink) and 5600 (vision)
- Updated `main.py` to bind `0.0.0.0` instead of `127.0.0.1` so the listener accepts VM traffic
- FlightSim.exe runs in VM but with limited 3D performance (no GPU passthrough, x86 emulation). Adequate for connectivity testing; serious dev will need a Windows box with a real GPU.

---

## 2026-06-06 to 2026-06-07 — Control Mode Investigation

Several runs against the live sim. Key findings:

- **Re-arming required.** The sim disarms us at the race-start transition. Controller now re-arms whenever it sees `armed=False`. Added HEARTBEAT handler to read armed state.
- **Early start = DQ.** Sending any control commands before `race_started=True` is treated as an early start. Controller holds zero thrust until then.
- **`race_started` semantics.** The race status message has both a `sim_boot_time_ms` and a `race_start_boot_time_ms`. We were treating "race start scheduled" as "race started." Correct condition is `sim_boot_time >= race_start_time`.
- **Velocity setpoints do not work.** `SET_POSITION_TARGET_LOCAL_NED` with velocity-only mask is accepted but the drone doesn't track the setpoint — it falls and drifts.
- **Position setpoints do not work either.** Same message with position-only mask: drone goes in unpredictable directions, climbs uncontrollably.
- **Attitude+thrust works cleanly.** `SET_ATTITUDE_TARGET` with zero rates + thrust=0.6 produces controllable, predictable climb. Confirmed the sim has a working attitude inner loop.
- **Added our own HEARTBEAT sender at 1 Hz.** MAVLink protocol requirement. Example omits this but the example doesn't actually fly.

Also added handlers for HEARTBEAT, COMMAND_ACK, STATUSTEXT for visibility into the sim's state.

---

## 2026-07-27/28 — First official VQ2 run + gate detector working

**Run 1 on the real VQ2 plant** (`main_vq2.py`, MISSION=frames). Confirmed:
- **Vision works**: Pillow decode, 1380 frames decoded, 45 saved. No OpenCV.
- **IMU-only hover calibration = 0.244** (VQ1 measured 0.25) — derived from
  accelerometer alone, since VQ2 removed velocity.
- **Estimator initialised and read pitch -17.8 deg at rest** = the ~18 deg
  start ramp we characterised in VQ1, independently rediscovered.
- **`fields_updated=0x3f`: accel + gyro ONLY.** mag=nan, pressure_alt=nan.
  So heading and absolute altitude are permanently unobservable from the IMU.
  That is exactly what the visual-servoing architecture already assumed
  (body-relative bearings; elevation from the image), but it closes the door
  on any compass/altimeter fallback.
- **It tumbled** once the attitude loop engaged (-17.8 -> +73.5 -> -137.8 deg).
  Cause: I assumed the rev-3390 rad/s type_mask bit normalised the body-rate
  SIGN conventions. It does not. VQ1 measured them as [+1,-1,-1] and detected
  them every run; sign auto-detection is now restored, with the estimator as
  observer since ATTITUDE telemetry is gone. **Detect, never assume.**

### Runs 4-5 + Elodin bring-up harness — an IMU cannot see a steady descent
Run 4 fixed the ceiling (settle vz +5.5 -> +0.50) and got the first clean
sign measurements — roll AND pitch both responded ~0.4 rad/s to a 0.5 command,
no tumble. Then the drone sank at ~10 m/s into the base of gate 0 while thrust
saturated at 0.84 trying to catch it.

Four official-sim runs had now gone to bring-up rather than flying, so per the
plan stated in advance, debugging moved to Elodin: `elodin_vq2_bringup.py`
drives the SAME `ControllerVQ2` (frame conventions injected) from Elodin's IMU
and prints TRUE altitude/vz beside the controller's estimates. One 90-second
run found it:

    t=3.0  TRUE vz=-2.33 | EST vz=+0.86  err=+3.20
    t=7.0  TRUE vz=-2.27 | EST vz=+0.59  err=+2.86

**The drone was falling at 2.3 m/s while the estimator insisted it was climbing
at 0.6.** The damper therefore trimmed thrust BELOW hover and drove the very
descent it could not observe.

Root cause is physics, not a coding slip: **at constant velocity net
acceleration is zero, so the accelerometer reads exactly 1 g — a steady
descent is indistinguishable from a hover.** Integration sees only CHANGES in
velocity, and the leak term turns any leftover transient into a standing bias.

Fix: vz damping is confined to the settle transient (measured from a genuine
known-zero on the pad, doing its one job of arresting the calibration climb),
and the estimate is zeroed at settle so no residual carries forward. After
that, **vision owns altitude** — a gate's elevation in frame is an absolute
reference, which is precisely what an IMU cannot provide. Not a workaround:
it is why the architecture put elevation control on the camera to begin with.

Verified in Elodin: after settle, thrust holds at hover (0.16 vs the previous
0.13) and the descent self-arrests -1.69 -> -0.53 -> -0.19 -> -0.07 m/s with
altitude converging. `EST vz` remains biased, as physics dictates, but is now
inert because nothing acts on it.

(Harness caveat: Elodin's VQ1 replica spawns the drone mid-air, so it free-falls
for the first second and contaminates the calibration measurement there. The
official sim starts on a ramp and measured hover consistently at 0.238-0.251.)

### Run 3 — we were flying into the hangar ceiling
Gyro-based sign detection gave a clean first measurement (`axis 0: commanded
+0.50 -> gyro +0.453`, near-unity gain, sign +1 — which also confirms the
rev-3390 rad/s bit works). But axis 1 read +5.05 rad/s: by then the drone was
already inverted at roll -173 deg, so that number is a tumble, not a response.

Root cause was not the pulses. Calibration burns CAL_THRUST against a 0.238
hover for over a second, which leaves ~5.5 m/s of CLIMB. The settle phase then
held *hover* thrust — which means zero ACCELERATION, not zero velocity — so the
drone coasted upward through settle and sign detection, ~20 m inside a roofed
hangar, and hit the ceiling (`a_up` spike to +13.9 m/s^2, then tumbling).

In VQ1 the settle phase damped this using velocity telemetry. VQ2 deleted
LOCAL_POSITION_NED, and I replaced that with a passive wait, which arrests
nothing. **Nothing in VQ2 reports how fast you are climbing.**

Fix: integrate vertical acceleration into a `vz_est` (reset to a known zero on
the pad, leaked slowly to bound bias) and use it as a damping term on thrust
everywhere during bring-up. It drifts over minutes but is accurate over the
seconds needed to stop a climb, and it is never used as absolute altitude.
Also gentled the calibration burn (0.35 -> 0.30, 1.2 s -> 1.0 s): every m/s of
climb bought there has to be paid back before flying.

Simulated against the real numbers: peak bring-up altitude 20 m -> **3.0 m**,
vertical speed at mission start 5.5 -> **0.23 m/s**.

### Stage 3b — gate detector: WORKING on real imagery
The captured frames overturn the README's "high-fidelity 3D-scanned
environments": VQ2 is a dark, desaturated indoor hangar with **saturated
orange-red square gates** and cyan guidance lines threading between them. A
hue threshold isolates the gates almost perfectly.

`detector.py` (numpy + Pillow only — opencv has no win_arm64 wheels ever):
hue mask measured from real frames -> 4x block-reduced mask -> pure-Python
union-find connected components -> aspect filter -> centroid.
- The centroid of a square FRAME is the centre of its opening: the servo's
  aim point for free.
- Apparent width -> monocular range via the known 1.5 m opening.
- Colour rejects the white ceiling panels that would fool a shape-only
  detector; aspect rejects the cyan guide lines.

**Measured on 45 real frames: 38/45 detected, 14 ms/frame (~71 fps).** All 7
non-detections are correct refusals — 6 are ceiling-facing tumble frames with
zero gate-coloured pixels, and 1 is the drone at/inside a gate (a diffuse red
wash with no gate to aim at). Effectively 100% correct behaviour.

Detector is now wired into `controller_vq2` race mode:
detect -> stabilized_bearing (IMU de-rotation) -> servo.

---

## 2026-07-26 — VQ2 dropped: it deletes almost everything VQ1 stood on

Sim v1.0.3391 + PyAIPilotExample-v4. Diffed v4 against v1; the example carries
explicit "disabled" notices:

- **LOCAL_POSITION_NED — disabled.** No position. The NED planner has nothing
  to run on.
- **ATTITUDE — disabled.** No roll/pitch/yaw. Our attitude P-loop and the
  measured response matrix lose their feedback signal.
- **ODOMETRY — disabled.**
- **Track data — nulled.** Gate positions/orientations/dimensions gone.
  `active_gate_index` still sequences, but never says WHERE the gate is.

Surviving: HIGHRES_IMU (accel+gyro), the FPV camera (`vision_rx.py` is
byte-identical to VQ1), RACE_STATUS, COLLISION, HEARTBEAT, TIMESYNC,
ACTUATOR_OUTPUT_STATUS. All MAVLink plumbing, re-arm and race-start
discipline carry over unchanged.

**Gift in the v4 diff** (sim rev 3390): a type_mask extension bit
`ATTITUDE_TARGET_TYPEMASK_DCL_BODY_RATES_RADS = 16` makes the sim interpret
body rates as documented physical rad/s instead of the legacy scaling we
reverse-engineered over 16 runs. "Recommended for new integrations" — opt in.

So VQ2 is the vision problem the competition is named for: estimate your own
pose and find gates from a camera + IMU, nothing else. Plan (staged, per the
discipline that cracked VQ1): 1) IMU attitude estimator, 2) closed-loop
stabilization on the estimate, 3) gate detection offline, 4) visual servoing,
5) full course. Elodin first — its FPV camera matches the VADR intrinsics.

### Stage 1 — IMU attitude estimator: PASS

`estimator.py`, Mahony complementary filter + gyro-bias learning, pure Python
(no numpy: the VM still can't build it for 3.14). Frame-parameterized via
`up_world` so one filter serves Elodin (FLU/ENU) and AIGP (FRD/NED).

`tests/test_estimator.py` — synthetic IMU forward model, no sim needed. It
immediately caught a **correction sign error**: the intuitive `cross(est,
meas)` is positive feedback and diverged to 180 deg. Rotating the BODY FRAME
by +w moves a fixed world vector's body-frame representation the OTHER way,
so the correct term is `cross(meas, est)`. Cases 1-2 had passed trivially
because initialization sets attitude directly and never exercises the
correction path — hardened with a mid-flight estimate corruption.

Elodin validation (`elodin_estimator_check.py`, passive alongside the proven
ground-truth solver, full VQ1-replica lap):
- mean tilt error **3.5 deg**, under 1 deg in smooth flight; 11.8 deg peak
  during the takeoff transient (thrust contaminates the accel reference).
- yaw drift ~5 deg/min — slow, and unobservable in principle from gravity
  alone. Fine by design: VQ2 control is visual servoing on body-relative
  bearings, so absolute heading is never needed.

### Stage 2 — closed-loop on the estimator: PASS
`ATT_SOURCE=truth|imu` switch in elodin_solver (one code path, not a fork).
With `imu`, attitude for the whole control stack comes from the estimator
alone; position/velocity still from truth, isolating the attitude variable.

**6/6 gates in 47.54 s** vs 47.29 s on ground truth — near-identical. Proof
the switch really took effect: the sim is deterministic, and the gate splits
differ (6.67 vs 6.98 at gate 0), so this was a genuinely different flight.

### Stage 3 — Elodin's FPV camera has never worked; DECISION: stop fixing it
Building a labeled frame dataset (frames + pose + gate truth, so detection can
be developed offline) surfaced why every run since day one reported
`FPV frames: 0`. Three separate defects:
1. `sim/camera.py register()` shipped `far=0.65` — a 65 cm far clipping plane
   on an FPV racing camera. Every gate is clipped out of existence. (patched
   locally -> 400 m)
2. `sim/main.py` only accepted a frame when `tick == _last_render_tick`, i.e.
   in the SAME tick it was requested; any render latency discards every frame.
   (patched locally -> pair one frame per request, report latency)
3. `ctx.render_cameras()` raises **"No render bridge available"** in BOTH
   `elodin run` and `elodin editor`. Likely cause: version skew — CLI is
   0.17.3 (from scripts/install_elodin.sh) while pyproject pins the SDK to
   `elodin==0.17.2`, and the render bridge is exactly the sim-process <->
   render-server protocol between them. Untested fix: align the two versions.

**Decision: do not chase this further now.** Rationale: (a) fixing it risks
destabilizing the control stack that currently works, (b) Elodin's gates are
simple GLB models while VQ2's courses are high-fidelity 3D scans, so a
detector tuned on Elodin imagery would not transfer anyway, and (c) the
camera is not on the critical path — see the split below.

**Decoupling the two hard problems** (each testable independently):
- *3a — control on bearings* (Elodin, fast, NO camera needed): compute the
  gate's true bearing analytically from pose+gate truth and feed the servoing
  controller only that bearing. Validates the VQ2 control architecture, which
  must fly with no position and no velocity.
- *3b — image -> bearing* (needs real frames): capture VQ2 frames to disk from
  the official sim, then develop the detector offline against authoritative
  imagery. Keep the interface narrow (frame in -> bearing + confidence out)
  so 3b drops into 3a without touching control.

### Stage 3a — bearing-only flight: PASS (6/6 gates, 53.60 s)
The full VQ2 control architecture, validated end to end in Elodin:
attitude from the IMU estimator, steering from a gate bearing, and **no
position, velocity or gate coordinates anywhere in the control path**.
53.60 s vs 47.29 s for the VQ1 position-based solution — 13 % slower, which
is a fair price for flying half-blind.

Ground truth is used for exactly one thing: standing in for the camera via
`bearing.observe_from_truth()`. The controller only ever sees
(az, el, width_px, confidence), so a real detector drops in unchanged.

Three real bugs, every one caught OFFLINE rather than in sim runs:

1. **Elevation coupled to our own pitch.** Raw camera `el` cannot tell "gate
   is above me" from "I am pitched nose-down to cruise". The vertical loop
   chased its own attitude, climbed 5 m over gate 0, pushed it out of the
   vertical FOV and searched forever. Fix: `bearing.stabilized_bearing()`
   de-rotates the sighting into a LEVEL frame using the IMU estimate, so
   el = 0 means "gate at my altitude", independent of camera mounting tilt.
   This is the payoff for having built the estimator first.
2. **Yaw steered the wrong way.** In FLU +body-z is UP, so a positive yaw
   rate turns LEFT, while positive azimuth means RIGHT. az grew +1 -> +15 deg
   under "correction" until the gate left frame. YAW_SIGN = -1 (Elodin FLU),
   +1 (official sim FRD).
3. **A 20 deg up-tilted camera cannot see below ~9 deg of the flight path.**
   VADR-TS-002 mounts the camera +20 deg up with a 58.7 deg vertical FOV, so
   a gate BELOW is not dim or partial — it is absent from the image, and a
   yaw-only sweep hunts past it forever. This is a permanent property of the
   competition hardware, not a coding slip, and it matters: VQ1's course
   descends 26 m. Search now descends and creeps forward to raise a low gate
   back into frame.

`tests/test_servo.py` is the tool that found #2 and #3: it runs the REAL
bearing+servo code against a 20-line kinematic vehicle and proves the chain
(geometry -> projection -> level bearing -> servo -> motion -> convergence)
in milliseconds, in both frame conventions. 8/8 cases converge. Same leverage
the synthetic IMU test gave Stage 1.

### Stage 3a robustness — how good must the real detector be?
Ran the stand-in detector degraded (`DETECTOR_NOISE_DEG=2 DETECTOR_DROP=0.3
DETECTOR_HZ=10`) to derive a spec for Stage 3b BEFORE building it:

| detector                          | gates |
|-----------------------------------|-------|
| perfect (30 Hz, no noise/drops)   | 6/6 in 53.60 s |
| 2 deg noise, 30 % drop, 10 Hz     | 2/6 -> **3/6** after the fixes below |

Not solved, but the failure mode changed from fatal to slow: it now
reacquires gate 2 (t=46.4) instead of dying, and simply runs out of the 60 s
window. Two things fixed along the way, both instructive:

1. **The servo discarded its fix on every missed frame.** Added a target
   tracker: EMA-smoothed bearing, and coast on the last good fix for `hold_s`
   before declaring the gate lost. (Necessary but not sufficient.)
2. **The search behaviour fought itself.** Stopping to "look around" LEVELS
   the drone, and because the camera is body-mounted at +20 deg, levelling
   shrinks the downward view from ~23 deg (pitched at cruise) to ~9 deg — the
   search destroyed the very FOV it needed to find a gate below. Worse, the
   descent I added had no floor: the drone rode it to z=0 and sat there
   spinning. Search now keeps meaningful forward tilt (nose down = camera
   down) and bounds the descent in time.

**Transferable insight: on this airframe forward speed IS downward vision.**
Same camera geometry in the official sim, so this applies there unchanged.

Remaining robustness work (not started): faster reacquisition — a systematic
sweep pattern rather than a fixed-direction yaw, and using `active_gate_index`
transitions to predict roughly where the next gate should appear.

**Methodology note.** The offline test initially FAILED this correct fix,
because it flew level and never pitched — it could not represent the very
coupling the fix relies on. A test that omits the mechanism you depend on
will confidently mislead you. It now integrates a first-order pitch lag
driven by the forward-tilt command; 8/8 cases pass.

### Vision deps on the Windows ARM VM: SOLVED (no opencv needed)
PyPI 2026-07-26: **opencv-python has NO win_arm64 wheels at any version**
(and no cp314 win_amd64) — but **Pillow ships cp314 win_arm64**, as does
numpy (2.5.1) and simplejpeg. So `pip install pillow numpy` gives JPEG decode
+ array math on the VM with zero compilation: no MSVC, no x64 emulation.

**Constraint for all vision code: numpy + Pillow only.** cv2 exists in the
Mac-side elodin venv and is fine for prototyping//analysis, but anything that
ships must import only numpy and PIL.

---

## 2026-06-12 — ★ VQ1 COMPLETE IN THE OFFICIAL SIMULATOR — 6/6 GATES ★

Sixteen race attempts over two days, each fixing one measured problem. The
working architecture, end to end:

    race start → CAL (0.9s, zero rates + fixed thrust → hover fit)
              → SIGN (pulse each axis → body-rate sign conventions)
              → SETTLE (level, trim hover vs vertical speed)
              → MAP (hold small tilts → 2x2 attitude→accel response matrix)
              → FLY (planner → matrix⁻¹ → rate commands at 100 Hz)

Every plant parameter is measured in-race in ~5 s — nothing assumed. The
same planner that swept the Elodin replica runs unchanged on top.

Hard-won sim facts (validated, not inferred):
- Only body-rates+thrust commands are honored; quaternion-attitude mode
  ignores the thrust field entirely.
- Body-rate sign conventions vs MAVLink standard: roll +, pitch −, yaw −
  (reproducible across 6+ runs).
- ATTITUDE telemetry is physical attitude; the start pad is a real ~18°
  ramp (every run reads +18 pitch at rest).
- Track-data gate z is the gate BASE, not the opening: aim 1.35 m higher
  (2.7 m outer frame). Run 14 flew under; run 16 crossed mid-box.
- Drone yaw spawns ~140°; never command yaw motion — pin it.
- Hover thrust ≈ 0.25 (auto-measured each run; varies slightly).

Stage discipline (per Nick, after run 12): MISSION=hover → waypoint → race,
each gated on a clean log. Hover and waypoint stages passed first try once
calibration ran open-loop before sign detection.

Run 16: all 6 gates, ~95 s, finish detected, clean exit. Run record
uploaded by the sim. VQ1 ✓ — next: verify the portal shows the run, then
VQ2 prep (visually complex; reread the spec when it drops).

---

## 2026-06-11 (later) — Bypassed Betaflight; VQ1 Replica COMPLETE 6/6 in 47.29s

Decision (with Nick): stop fixing the broken BF bridge, fly Elodin's clean
physics directly with our own stack.

- `flight_stack.py`: reduced-attitude P (body-z × desired-z error) → rate P →
  X-quad mixer, gains derived from the racing preset's known constants.
  sim/main.py patched to accept DIRECT_MOTORS from the solver (BF still runs
  in lockstep, output ignored).
- First flight was textbook: clean takeoff, smooth cruise, motors at the
  theoretical hover (0.114) — first run all session where throttle authority
  was real.
- Easy course: 3/3 in 12.29s after adding pass-through aiming (PD-to-position
  parks AT the gate; aim ~2.5m past it and let the sim advance the index).
- VQ1 replica fixes: (1) pass-through offset must be horizontal-only or a
  descending course crosses the plane below the inner box; (2) altitude
  i-term windup during long descents — near-disabled the integrator (hover
  feedforward is exact) and stiffened kd_alt.
- **VQ1 replica: 6/6 gates, 47.29s** (cap is 8 min). Ready to port.

Port plan: AIGP adapter = same planner + NED→(x,y,alt) conversion + efforts →
SET_ATTITUDE_TARGET quaternion+thrust (the official sim's stabilizer plays
flight_stack's role). Re-run system_id-style calibration for hover thrust.

---

## 2026-06-11 — Elodin Harness Forensics: the Betaflight Bridge Is Broken

Replicated VQ1 in Elodin (sim/course.py VQ1_COURSE from logged track data) and
spent the day chasing why nothing could descend. Calibration probe
(elodin_probe.py) + raw db telemetry traced it to the harness itself:

- **Gyro axis signs in sensors.py are wrong.** The shipped code sends
  (wx, -wy, +wz); Betaflight SITL (gazebo build) needs (wx, -wy, -wz).
  The inverted yaw loop saturates the motors in a permanent max-differential
  fight (one motor at 1.0, one at idle floor), and **airmode lifts collective
  to ~0.53 regardless of throttle stick** — Betaflight has never tracked our
  throttle in any run. The baseline's "altitude control" is an illusion.
- ANGLE mode is also unusable: the attitude estimator gets frame-inconsistent
  gyro/accel and levels to a wandering 30-50° tilt. (Engaged via aux2≥1700;
  the shipped baseline never engages it, so the authors never saw either bug.)
- DEFAULT_CONFIG is a smoke-test plant: linear_drag z=40 N/(m/s) caps
  vertical speed at ~0.6 m/s. Patched locally to create_5inch_racing_quad().
  The baseline's hover constant (1135) matches the racing preset, not the
  shipped default — more evidence the default is unintentional.
- With corrected signs + racing preset, all three rate loops survive but
  limit-cycle at ±2.5 rad/s with motor saturation (BF default PIDs vs tiny
  inertia + lockstep latency) — airmode still owns the throttle.
- Clean measurements obtained: stick→rate ≈ 0.30 (deg/s)/PWM from level;
  probe runner tools/run_elodin.sh (elodin run never exits on its own —
  every "batch" run before this only executed its first command).
- Upstream has no fixes and no issues filed. Worth reporting once confirmed.

Local patches to the elodin clone: sensors.py (gyro signs), config.py
(racing preset), course.py (VQ1 replica + course selector), main.py
(course env wiring). Decision pending: keep fixing BF's tuning vs bypass
Betaflight with our own attitude controller on the clean physics core.

---

## 2026-06-10 — Sim-Agnostic Planner Working in Elodin

- Strategy shift: develop against the Elodin practice harness (Betaflight SITL,
  headless on Mac, seconds per iteration); official sim becomes calibration/
  integration only. See ARCHITECTURE.md "Two-Sim Development Loop".
- New `planner.py`: sim-agnostic (x, y, alt) world frame, normalized effort
  outputs; control law is the Elodin baseline's proven per-axis PD + altitude
  PD+I, with gains normalized out of PWM units.
- New `elodin_solver.py`: adapter mapping planner efforts → Betaflight RC PWM;
  gates imported from the sim's course definition (no hardcoding).
- Result: **3/3 gates, 9.43s lap** (baseline: 9.52s) on first headless run.
- Cleanup note: a stale `sim/main.py` from the 6/08 editor session had been
  spinning at 335% CPU for two days and can deadlock new runs (holds the
  Betaflight bridge ports). `pkill -f sim/main.py` before runs if in doubt.
- VQ1 doc check (VADR-TS-001): §4.5 telemetry includes "simulator navigation
  reference data" (= track data), §8.1 objective is course completion, §4.6
  defers vision to a separate spec → no vision needed for VQ1; planner takes
  gate list as input so vision can slot in for VQ2 if required.
- Next: AIGP adapter — feed track-data gates into the same planner, map efforts
  → SET_ATTITUDE_TARGET quaternion + thrust, calibrate hover/tilt constants in
  a handful of official-sim runs.

---

## 2026-06-08 — Pivot to Middle-Tier Cascaded Controller

Decision: single P controller is fragile and won't compete. Build a proper cascaded PID with system identification.

- Updated ARCHITECTURE.md with cascaded-controller design and new repo layout
- Plan: `system_id.py` measures hover thrust + attitude response → `measurements.py` constants → `pid.py` cascaded loops use measured values
- VQ1 strategy: identify, implement, fly through 6 gates in completion-grade time. Aim for 5–7 sim runs total, not 20+ from hand-tuning.
- VQ2 prep: read up on MPC + differential flatness for quadrotors (Foehn/Scaramuzza). Not implementing yet.

## VQ2 official run 6 — first flight under closed-loop vision, and the impostor spin

Removing in-race calibration worked: the drone flew at the gun and detected
gate 0 immediately. First time the built stack has actually flown as a whole.

It then span up and crashed. The mechanism, straight from the log:

    DET u=323.1 rng= 5.8 conf=1.00   yaw=  +0.1   <- gate dead centre
    DET u=331.2 rng= 4.3 conf=1.00   yaw=  +1.6
    DET u=403.5 rng= 1.0 conf=1.00   yaw= +11.1   <- 480 px blob
    DET u=621.3 rng=12.6 conf=0.44   yaw= +68.9   <- teleported to the edge
    no-gate                          yaw=+130.1

az at u=621 is 43 deg; kp_yaw 2.0 saturates max_yaw_rate 1.6 rad/s = 92 deg/s,
which is exactly the observed 46 deg per half-second. The servo obeyed. The
sightings were wrong: no gate goes 4.3 -> 1.0 -> 12.6 m while jumping 290 px.

The detector reports the best orange blob in EACH FRAME with no memory, so
"the gate" can be a different object every frame. This was invisible until now
because Stage 3a validated the servo on synthetic bearings, which are
continuous by construction, and Elodin's dead render bridge means no image can
ever reach the servo there. The bearing transform was NOT at fault — h and g
both come from the same quaternion, so the unobservable yaw cancels exactly in
the angle between them (verified by hand in both NED and ENU).

Fix: track continuity gating in servo.step(). Acquiring a track needs
confidence >= 0.55; maintaining one needs only 0.15. Once a track exists, a
sighting is rejected as mistaken identity if its bearing or range could not
have evolved from the track in the elapsed time (2.5 rad/s, 20 m/s, plus fixed
slack). Rejecting continuously for 0.7 s drops the track — a wrong track must
not lock out a real gate forever.

Regression test replays the four real sightings above: yaw command falls from
saturated 1.6 rad/s to 0.04. All eight original convergence cases still pass,
so the gate rejects the impossible without refusing honest tracking.

Open: the impostors themselves are unexplained. 47 frames were saved to
vq2_frames6 — replaying the detector over them offline will show what it
actually latched onto, which is the difference between mitigating this and
fixing it.

## VQ2 official run 6 — the real cause: no vertical rate feedback

Frames from run 6 (47 JPEGs + IMU sidecar) were replayed offline. That
overturned the run-5 diagnosis. The yaw runaway is a SYMPTOM, not the cause.

What the frames show, in order:
  frame  9  gate 0 centred, u=323, w= 84 px
  frame 11  closing,        u=333, w=118 px
  frame 12  closing,        u=349, w=197 px
  frame 13  the gate's TOP BANNER fills the bottom of the view

Frame 13 is the drone above the top bar looking down at it. It clipped the top
of gate 0. Frames 14+ are a tumble (inverted "Station" text, floor overhead),
and every wild yaw excursion in the log happens after that impact. Bearing
tracking up to frame 13 was good: u went 323 -> 333 -> 349, nicely centred.

So the aircraft flies HIGH — which is exactly what runs 1 and 2 showed ("flew
slightly above and past the first gate", twice). That was the signal all along.

The elevation math is NOT wrong. Hand-computing stabilized_bearing for frame 9
(u=323, v=152, pitch -10.9 deg) gives el_up = +14.1 deg -> vertical = 0.394 ->
thrust = hover * (1 + 0.8 * 0.394) = 0.326, matching the logged thr=0.32. Gate
0 really does sit ~2.4 m above the start pad, so climbing was correct.

The bug is that nothing stopped the climb. Fixing the earlier descent bug meant
setting VZ_DAMP_AFTER_SETTLE = False, because vz_est is unobservable in steady
flight (constant velocity => zero net accel => reads exactly 1 g). Correct fix,
but it left a PURE PROPORTIONAL controller driving a second-order plant, which
must overshoot.

The damping signal we were missing is observable and was sitting in front of
us: d(elevation bearing)/dt. It comes from DIFFERENTIATING VISION, not from
integrating accelerometers, so none of the vz_est unobservability applies.

Gains chosen against a model, not by sim trial-and-error. Worst-case vertical
miss at the gate plane, swept over gate offsets -2..+3.5 m and closing speeds
4..9 m/s:
    kp_el 1.6, kd_el 0.0  (what we flew)  1.65 m   <- hits the top bar
    kp_el 1.6, kd_el 1.3                  0.51 m
    kp_el 0.8, kd_el 1.3  (adopted)       0.37 m   <- 2x margin in a 0.75 m half-opening

Two permanent additions, both aimed at the fact that the kinematic servo test
commands velocity directly and therefore CANNOT exhibit overshoot — it passed
happily through all of this:
  * tests/test_servo.py::test_vertical_overshoot integrates the real plant
    (thrust -> accel -> velocity -> altitude) and asserts the miss fits the
    opening. It also asserts a P-only loop FAILS it (1.85 m), so the test has
    teeth.
  * tools/replay_frames.py re-flies captured frames through detector+servo
    offline. It found this in one pass instead of a five-minute sim cycle.

Also tightened the continuity gate from run 5: max_az_rate was 2.5 rad/s,
LOOSER than the 1.6 rad/s the aircraft can even yaw, so it could only reject
the physically absurd. Now 1.8 rad/s / 0.12 rad step / 12 m/s closing. On run
6's frames the gate now accepts frames 0-13 and rejects the frame-14 impostor.

Known bias, not yet fixed: range is computed from the detected OUTER frame
width but divided by the 1.5 m INNER opening, so it underestimates by roughly
the frame-to-opening ratio (~1.6x). Stationary on the pad it reports 5.7 m.
Control never uses range — only the continuity gate and logging do, both in
consistent units — so this is cosmetic for now.

## VQ2 official run 7 — GATE 0 CLEARED, then lost in the lost-target path

active_gate_index advanced 0 -> 1. First gate ever passed in VQ2 on the
official sim. The PD fix held the approach: v stayed 169 -> 191 near centre
instead of dropping away as it did in run 6.

Then it never found gate 1, and failed two ways, both in code that had never
run before because we had never passed a gate:

1. Search yawed ONE direction forever. `direction = 1 if last_az >= 0 else -1`
   never reverses, so it turned ~147 deg off course, chased one of the many
   other red gates in the hangar, lost that too, and kept going.

2. No altitude reference exists when nothing is visible. This is vz_est
   unobservability in a new place: hover thrust PRESERVES vertical velocity
   rather than arresting it, so once sinking, nothing stopped it (-1.1 ->
   -9.4 m/s). search_descend was actively making it worse — and gate 0 sits
   ABOVE the pad, so descending to search is backwards.

Fixes:
  * Bounded widening sweep. A course continues roughly forward, so the heading
    we passed the gate on is the best prior available. Sweep +/-35 deg about it,
    alternating, widening 1.6x per reversal up to full circle. Test: peak 89 deg
    and it comes back, vs run 7's monotone 147 and climbing.
  * Vertical search oscillates (2.4 s period) instead of descending, so a
    fruitless search nets zero altitude drift. Test asserts |integral| < 0.15;
    measured -0.000.
  * servo.gate_passed(), called on the active_gate_index transition. That index
    is the only unambiguous progress signal VQ2 gives us and run 7 ignored it
    completely: after clearing gate 0 the servo kept steering on a track that
    described a gate now behind it. It now clears the track and re-centres the
    search prior.

Nick's observation that gate 1 never entered the camera's view is the thing to
confirm from vq2_frames8 — if gate 1 is genuinely outside a 90 deg HFoV cone
tilted 20 deg up, no search tuning fixes it and the course geometry has to
drive where we point the camera.

## VQ2 official run 8 — gate 0 again, and the course turns RIGHT

Gate 0 cleared for the second consecutive run, faster (3 log samples), and
[GATE] passed 0 -> 1 fired correctly, clearing the stale track as intended.

Then, one sample later, roll +174 and inverted.

The important finding is about the COURSE, not the code. Immediately after the
pass the detector reported u=609.6 rng=15.0 conf=0.74. That confidence is
exactly the detector's own edge-fade at u=609 (min(609, 639-609)=30, 30/40 =
0.75), which means the sighting is a gate ~15 m away at 42 deg to the RIGHT —
inside the 90 deg HFoV, just at its edge. That is almost certainly gate 1.

So Nick's "gate 1 never entered the camera's POV" is nearly right but the
consequence is the opposite of what I assumed: gate 1 IS visible, at the far
right edge, because the course turns hard right after gate 0. The target
selection was correct. The RESPONSE was fatal:

  * az=42 deg * kp_yaw 2.0 = 1.47 rad/s, essentially saturated, commanded in a
    SINGLE control step with no slew limit.
  * Applied ~1 m past the gate plane, with gate 0's posts still alongside. A
    hard yaw there clips a post — which is what roll +174 is.

Fixes (deliberately NOT raising acquire_confidence, which would have rejected
the correct gate-1 sighting at 0.74 and made this worse):
  * max_yaw_accel = 2.2 rad/s^2 slew limit on the yaw command, so no single
    sighting can snap the aircraft.
  * clear_gate_s = 0.45 s of straight flight after a gate pass before any
    steering. Tracking continues through it; only steering is inhibited.

Test asserts all three properties: zero steering during clearance, yaw accel
within the limit, and that it still does turn (settles at +1.47 rad/s).

Course knowledge gained: gate 0 sits ~2.4 m above the start pad, and gate 1 is
roughly 15 m from gate 0 at about 42 deg to the right.

## VQ2 official run 9 + the fix found offline: lateral control and the next-gate hint

Run 9 cleared gate 0 (third consecutive run) and the slew limit measurably
worked — the post-gate yaw jump fell from +44 deg/interval (run 8) to
+12 deg/interval. But the outcome was identical: yaw ran to +154, it sank, it
inverted. Nick, correctly: "What did you change? I see no improvement."

Fair. Changing one parameter per five-minute run and inferring from logs is the
exact loop this project rejected at the start. So the course geometry we had
measured got rebuilt in Elodin as course "vq2turn" (gate 0 at +2.4 m and 12 m
out; gate 1 at 15 m, 42 deg right; two further turns), and everything below was
found offline in 90-second runs with ground truth.

Findings that the official logs could not have given us:

1. GATE 1 IS ONLY VISIBLE FOR ABOUT TWO METRES. At the gate-0 plane its
   bearing is 41.8 deg, just inside the 45 deg half-FOV. Five metres later it is
   at 59.6 deg — outside the cone. The window is tiny and closes fast.

2. clear_gate_s = 0.45 WAS CAUSING THE LOSS IT WAS MEANT TO PREVENT. At ~7 m/s
   it inhibits steering for 3 m, i.e. exactly the metres where gate 1 is still
   visible. Cut to 0.15 s; the slew limit is what keeps the turn smooth.

3. THE HARNESS WAS TESTING DIFFERENT CODE. elodin_servo never called
   gate_passed(), and its synthetic detector projected only the ACTIVE gate, so
   it could not reproduce either target-hopping or next-gate glimpses. It now
   projects every gate and returns the widest in frame, which is what the real
   detector does (biggest orange blob, no notion of "active").

4. THE NEXT-GATE HINT. Runs 7, 8 and 9 all saw gate 1 while still tracking gate
   0 (u=509..609, conf 0.74-1.00). The continuity gate correctly rejected those
   as not-gate-0 and then DISCARDED them. Now a confident rejected sighting is
   stored as hint_az, and gate_passed() steers the new search toward it.
   Also stopped zeroing last_az on a gate pass — that destroyed the only clue
   available about which way the course turns.

5. THE REAL MISS CAUSE: NO LATERAL CONTROL. tilt_right was hard 0.0 in every
   branch, so direction changes waited on the nose coming round and then on
   forward tilt pushing the new way. Coming off a turn the aircraft keeps its
   old momentum and slides wide: it lined up on gate 1 perfectly at 6.4 m
   (az = -0.3 deg) and still crossed the plane 1.0 m off centre, outside the
   0.75 m half-opening. Added kp_lat 0.9 / max_lat 0.35 (suppressed during the
   gate-clearance window). That single change took vq2turn from 1/4 to 4/4.

6. A REGRESSION CAUGHT AND FIXED. The oscillating vertical search from run 7
   had period 2.4 s, which displaces only ~0.65 m — fine for not sinking, but
   it removed the ability to find gates BELOW the FOV's -9 deg floor. VQ1
   replica went 6/6 -> 2/6. Period 6.0 s gives a several-metre sweep down (the
   blind side comes first, since sin() starts negative) and recovers it, netting
   zero drift. Both courses now pass.

Elodin state: vq1 6/6 COMPLETE 55.85 s; vq2turn 4/4 COMPLETE 19.23 s.

The vq2turn course lives in the local elodin clone (sim/course.py), which is not
in this repo. Definition is reproduced in the entry above so it can be rebuilt.

## VQ2 official run 10 — the azimuth loop was in POSITIVE FEEDBACK all along

Nick, watching the run: "It seemed to turn and face left as it was about to pass
through the first gate." That observation resolved runs 6 through 10.

From the log, after clearing gate 0:
    DET u=521.4  (32 deg right)   yaw= +49.8
    no-gate                        yaw=+107.6      <- "turned right" 58 deg
    DET u=630.5  (44 deg right)    yaw=+107.9

We commanded right, the yaw estimate advanced 58 deg, and the target moved from
32 deg right to 44 deg RIGHT. A genuine right turn sweeps a right-hand target to
centre and out the left side. It can only move further right if the aircraft
turned LEFT. Every run since 6 shows the same monotone runaway in the same
direction: gate right -> command right -> turn left -> gate further right.

YAW_SIGN was +1.0, justified in the comment by "in FRD +body-z already turns
right". That is textbook-correct and empirically wrong for this sim. Now -1.0.

Why the in-race sign detection never caught it: it verifies that the commanded
rate and the GYRO agree in sign, which they do (+0.50 -> +0.112 every run). It
never checks whether the resulting rotation moves the IMAGE the expected way.
Those are different claims and only the second one matters for servoing.

Because that fix is a one-line inference that would otherwise cost another
five-minute run to test, the aircraft now checks itself in flight:
servo.check_polarity() accumulates samples where the commanded yaw is large
enough to dominate translation (>0.30 rad/s) AND there is a measurable bearing
response (>0.05 rad/s), then flips once if 60% of 12 samples show the wrong
sign. So a wrong constant self-corrects in ~0.4 s instead of losing a run.

It lives in servo.py rather than the controller: it is a claim about bearings and
steering, and putting it there keeps it testable without pymavlink (the first
attempt imported controller_vq2 and needed numpy/PIL/pymavlink stubs — a good
sign the logic was in the wrong module).

tests/test_yaw_polarity.py asserts a correct plant is left alone, an inverted one
is flipped, and the loop then converges (|az| -> 0.0 deg) rather than oscillating.

Also worth recording: run 10's approach to gate 0 was the best yet — u=322.0,
v=156.5 dead centre, and it passed with the new lateral control active. The
approach was never the problem after the PD fix; the departure was.

## VQ2 official run 11 — the polarity check fired, and it was WRONG

    [YAW] steering inverted (11/12 samples) -> yaw_sign=+1

It flipped back to +1 and made the run worse. The verdict cannot be trusted, and
neither can the run-10 inference that motivated flipping to -1 in the first
place. Both rest on the same confound:

Bearing rate has two sources, rotation and translation. Translation contributes
v*sin(az)/r. The check ran at rng=3.7 m with the gate 37 deg off axis at roughly
8 m/s, giving ~1.3 rad/s from translation alone — about TRIPLE the commanded yaw
rate. Azimuth was moving because the aircraft was flying past the gate, not
because it was rotating.

The decisive tell: run 10 flew yaw_sign=+1 and run 11 flew -1, and BOTH show
azimuth increasing. Both signs cannot be wrong, so azimuth increase was never
evidence about the sign at all. Six runs of reasoning from race logs, all
confounded the same way, none of it valid.

The |yaw_cmd| > 0.30 rad/s gate was meant to ensure rotation dominates. It does
not come close at short range. Added min_range_m = 15.0, and the auto-flip is now
opt-in (YAW_AUTOFLIP=1) rather than default: acting on confounded data actively
cost a run.

Rotation and translation only separate when translation is zero, so MISSION=
yawtest does exactly that: hover in place, no forward tilt, no roll, rotate at
0.35 rad/s for 6 s, and report how the gate's u moved. A right turn sweeps the
scene left, so u must DECREASE. Unambiguous, ~12 s, and needs a gate only to be
seen, not passed.

Separately, run 11 revealed a target-selection problem worth its own fix later:
right after clearing gate 0 the detector reported u=581 v=59 rng=3.7 — a large
blob high and right — and the servo climbed at +10 m/s toward it (a_up=+10.18,
thr=0.39, el ~ +31 deg). The hangar has ceiling-mounted gates (visible in run 6
frame 0), so "biggest orange blob" can be something well above the course.

Tests: close-range and missing-range evidence are now refused outright.

## MISSION=yawtest attempt 1 failed, attempt 2 rebuilt; offline measurement rejected

Attempt 1 produced no verdict, for a dull reason: it applied hover thrust while
parked on the 17.8 deg nose-down ramp. At exactly hover the aircraft barely
unsticks, and that attitude supplies g*sin(17.8) ~ 3 m/s^2 of FORWARD
acceleration, so it slid off the ramp into gate 0 (rng 5.6 -> 2.2 -> 0.8,
a_up=-14.66) before the rotation phase began. Yaw moved 2.4 deg in total.

An offline attempt using run 6's frames + gyro sidecar found exactly ONE usable
pair and reported "STANDARD". Rejected it: 37.7 deg of claimed rotation moved u
by 2.9 px, where pure rotation demands ~247. That pair is a 376 px blob filling
the frame, whose centroid is pinned by the frame edges and cannot register
rotation. The sign was right and the datum was still worthless — a good reminder
that a plausible sign with an impossible magnitude is not evidence.

Attempt 2:
  * lift (thrust 0.33, 1.2 s) -> matched reverse pulse (2*hover - 0.33, 1.2 s,
    which is the only way to arrest a climb without velocity feedback) ->
    settle -> rotate 6 s. Ends ~4.7 m up with vz ~ 0, clear of gate 0's 3.2 m top.
  * Regresses over the WHOLE rotation instead of comparing two endpoints.
  * Regression moved into servo.yaw_convention() — third time logic like this
    turned out to need pymavlink/numpy stubs to test, which each time meant it
    was in the wrong module.

Regressing BEARING, not raw pixels: u = cx + fx*tan(bearing) is nonlinear, so
du/dyaw is -320 px/rad near centre but -487 across 60 deg, and no fixed pixel
tolerance is right for every span. In bearing space pure rotation gives exactly
-1.00 regardless of span (verified at 20, 60 and 100 deg). The magnitude test is
what rejects pinned-centroid data, and it matters more than the sign.

Tests cover: standard, inverted, pinned centroid (refused), no rotation
(refused), too few samples (refused), and span independence. One synthetic case
initially failed at 100 deg span because the generator emitted u values outside
the 640 px frame — unphysical, since a gate leaves the 90 deg HFoV partway
through such a turn. Fixed by dropping invisible samples, as reality does.

STILL UNKNOWN: the actual yaw convention. Six race runs of inference were all
confounded, the offline attempt was rejected, and yawtest attempt 1 never
rotated. YAW_SIGN remains -1.0 and is NOT trusted.

## Broken build reached the sim AGAIN — and the report logic was wrong too

yawtest attempt 2 crashed instantly: AttributeError, no _integrate_vz. Index-based
string surgery (slice from one "def" anchor to another) swallowed SIX methods:
_sign_step, _calibrate, _integrate_vz, _damped_hover, _vertical_accel and
_check_yaw_polarity. This is the SECOND time this exact accident has shipped —
_calibrate/_vertical_accel went the same way earlier. Restored from ff7f4da.

The reason it keeps reaching the simulator is that nothing ever ran the
controller. Every test covered servo/estimator/bearing in isolation; none checked
that ControllerVQ2 was even callable. tests/test_controller_smoke.py now:
  * drives update() for every mission (hover/race/yawtest) against a synthetic
    plant, asserting the loop runs and thrust stays in [0,1]
  * runs the no-gate-visible path, where the search/sweep code lives
  * greps every self._method() reference against the defined methods, which
    catches this accident directly and cheaply

Building that test then exposed a real bug in _report_yaw_test. The slope measures
d(bearing)/d(yaw_est), and BOTH respond to the applied rate, so it is independent
of yaw_sign — it characterises the PLANT, not our constant. The first version
computed `want = self.yaw_sign if standard else -self.yaw_sign`, which returns a
different answer per run from identical data. Correct derivation: servoing needs
yaw_rate_c > 0 to shrink az; with applied = rate_sign[2]*yaw_sign*yaw_rate_c and a
standard plant d(az)/dt = -applied, so rate_sign[2]*yaw_sign > 0, giving
YAW_SIGN = rate_sign[2] for a standard plant and -rate_sign[2] for an inverted one.

So a good measurement would still have produced a wrong recommendation — half the
time, silently.

The smoke test's plant now RESPONDS: commanded yaw is fed back as gyro and a
world-fixed gate is reprojected, so yawtest yields a real verdict. All four
combinations are asserted (standard/inverted plant x flying +1/-1) and the verdict
is confirmed independent of the sign being flown, which is the property that bug
violated. Note the test must pass yaw_sign to the constructor: the signature
default binds at def time, so reassigning the module global does nothing — the
first version of the test silently flew -1 in all four cases.

Still unknown: the real yaw convention. YAW_SIGN remains -1.0, untrusted.

## yawtest attempt 3: rotated cleanly, and the answer was in the log

Attempt 3 rotated properly (yaw 0 -> -19.6 -> -36.1 -> -47.1 deg) but hit gate 0
and died three seconds before its report deadline. The measurement was extractable
from the log by hand:

    yaw    -0.9 ->  -19.6  (d=-18.7)   bearing +38.5 -> +18.4   slope +1.08
    yaw   -19.6 ->  -36.1  (d=-16.5)   bearing +18.4 ->  -2.6   slope +1.27
    yaw   -36.1 ->  -47.1  (d=-11.0)   bearing  -2.6 -> +41.8   slope -4.04  REJECTED

Two usable pairs, mean +1.17. The magnitude near 1 is what makes it credible —
that is the signature of genuine rotation rather than a pinned centroid or
translation. The third pair self-rejects: u jumped 301 px, a different gate.

Positive slope => the plant is INVERTED => YAW_SIGN = -rate_sign[2] = -1, which is
what we are already flying. So the yaw sign is NOT the bug, and six runs of chasing
it were chasing nothing. Recording that as a settled negative result.

Redesign, attempt 4. Three attempts tried to hover first and all died to the same
fact: the aircraft reaches gate 0 within ~1.5 s of the gun no matter what is
commanded (rng 5.3 -> 2.5 in two samples), so any lift/arrest/settle preamble is
destroyed before measuring. Hovering was never the requirement — separating
rotation from translation was, and speed achieves that equally well. Translation
contributes v*sin(az)/r, about 0.4 rad/s at 10 m and 8 m/s, so rotating at 1.2
rad/s dominates 3:1 while moving. Now: rotate immediately at 1.2 rad/s, use only
sightings beyond 8 m, and print the running verdict every second so a crash cannot
cost the measurement again.

The smoke test caught a real interaction the moment this landed: its synthetic gate
had width_px=90 => range 5.3 m, inside the new 8 m gate, so the measurement got
zero samples. Fixture corrected to width_px=40 (~12 m, as gate 0 is at the gun).
That is the first time a test has caught a controller regression before the sim did.

## VQ2 official run 12 — steering CONFIRMED working; the barrier is momentum

The best run yet, and it settles two things.

Gate 0 passed (fourth consecutive). Then:
    DET u=533.4  yaw= -22.6    gate 1 acquired at 33 deg right
    DET u=487.1  yaw= -72.6
    DET u=292.4  yaw= -92.7    swept through centre
    DET u= 76.7  yaw= -98.2    and out to 38 deg LEFT

The servo acquired gate 1, turned onto it, and drove its bearing from 33 deg
right through centre to 38 deg left. STEERING WORKS, and YAW_SIGN=-1 is confirmed
by behaviour as well as by the yawtest regression. Both yaw questions are closed.

Nick: "it flew through gate 0 too fast, so even though it turned to face the next
gate, the momentum carried gate 1 out of its pov." Correct, and it is geometry
rather than tuning. Turning an 8 m/s velocity vector through 42 deg needs
dv = 2*v*sin(21 deg) = 5.7 m/s. Lateral authority was max_lat 0.35 of a 20 deg
MAX_TILT = 1.2 m/s^2, so 2.7 s — and gate 1's bearing leaves the FOV in under 1 s.
No gain fixes that; speed has to come off.

Critically, the speed is not ours to avoid by cruising gentler. MAX_TILT_RAD is
20 deg, so cruise_tilt 0.55 commands 11 deg => 1.9 m/s^2 => about 2.9 m/s by gate
0, yet it arrives at ~8 m/s. The ramp launch supplies the rest, so lowering
cruise_tilt would barely touch it. The only lever is to actively brake, which the
old drive = max(min_tilt_frac, align) could never do: it kept forward tilt
POSITIVE however far off axis the target was, so the aircraft accelerated
throughout every turn.

Changes:
  * brake_az_rad 0.45 / brake_tilt 0.45 — beyond ~26 deg off axis, command
    nose-UP and decelerate instead of merely easing off.
  * max_lat 0.35 -> 0.60, doubling crossrange authority to ~2.4 m/s^2.
  * Pre-emptive braking: because the next-gate hint is captured while still
    flying the current gate, a known sharp turn ahead now slows the approach
    (drive *= 0.45 inside 7 m) rather than being discovered on the far side,
    where the momentum already exists.

Elodin, both courses still complete — slower, as braking implies:
    vq2turn  4/4  19.23s -> 24.92s
    vq1      6/6  55.85s -> 58.96s

## VQ2 official run 13 — the brake fired and was far too weak

Nick: "Looks like it didn't slow down nearly enough?" Exactly right, with numbers:
brake_tilt 0.45 of a 20 deg MAX_TILT_RAD is a 9 deg nose-up command, giving
g*tan(9) = 1.55 m/s^2. Shedding the ~8 m/s carried through gate 0 needs 5.1 s and
about 1 s was available. The log confirms it engaged (pitch reached +6.9, nose-up)
and simply did not matter.

MAX_TILT_RAD = 20 deg was capping every horizontal authority at once — braking,
crossrange, all of it. Raised to 35 deg. Vertical cost is affordable: at 30 deg
the thrust vector loses cos(30)=0.87 of its lift, needing 1.15x hover, and the
vertical channel already reaches 1.56x.

Braking is now DECOUPLED from the servo's normalized scale. The two harnesses
interpret tilt_fwd differently — Elodin feeds it straight to its motor mixer while
the official sim multiplies by MAX_TILT_RAD — so raising brake_tilt to 0.85 for
the sim's benefit meant "85% of full mixer authority" in Elodin. ServoOutput now
carries a `braking` flag: WHEN to brake is servo policy, HOW HARD is plant
specific (BRAKE_TILT_RAD = 30 deg => 5.7 m/s^2 => 1.4 s to shed 8 m/s).

TWO REGRESSIONS, one of them mine to own:

1. brake_tilt 0.85 took the VQ1 replica 6/6 -> 2/6. Explained by the scaling
   mismatch above, and fixed by decoupling.

2. It was still 2/6 after that fix, so the first diagnosis was wrong. The real
   culprit was PRE-EMPTIVE BRAKING, which I had added and tested ONLY on vq2turn,
   never on vq1. It looked principled — the next-gate hint tells us a sharp turn
   is coming before we reach the current gate — but on a course where the detector
   regularly sees distant gates at wide bearings, the hint is almost always
   "sharp turn ahead", so it crawled into every gate. Reverted.

   Reviving it requires attributing the hint to the NEXT gate specifically, which
   nothing in the current design establishes. A confident sighting that is not the
   tracked gate could be the next gate, a previous one, or any of the hangar's
   ceiling gates.

The lesson is the cheap one: run BOTH courses after every servo change. Testing
only the course a change was designed for is how a change that helps one case
silently destroys another.

Elodin, both courses passing again:
    vq1      6/6  COMPLETE  58.96s
    vq2turn  4/4  COMPLETE  24.92s

## VQ2 official run 14 — raising MAX_TILT_RAD made the speed problem WORSE

Crashed into the base of gate 0 faster than any previous run. The log says why in
one column: pitch stayed at -17.4 / -16.8 / -19.1 and never levelled, where
earlier runs sat near -11.

cruise_tilt, max_lat and brake_tilt are all FRACTIONS of MAX_TILT_RAD. Raising it
from 20 to 35 deg to buy braking authority raised the cruise angle from 11 to
17.5 deg in lockstep, so the aircraft accelerated harder than ever. The change
intended to fix excess speed directly increased it. That coupling was visible in
the code and I did not look for it.

Fixed by giving each authority its own ABSOLUTE angle, since what is wanted is a
gentle cruise with strong turning and braking — three different requirements that
one shared cap cannot express:

    FWD_TILT_RAD   14 deg  -> cruise 7.7 deg  -> 1.33 m/s^2 forward  (was 1.9)
    LAT_TILT_RAD   30 deg  -> lateral 18 deg  -> 3.19 m/s^2 crossrange (was 1.2)
    BRAKE_TILT_RAD 30 deg  ->                    5.66 m/s^2, 1.4 s to shed 8 m/s

So cruise acceleration is now LOWER than the original 20 deg setup while braking
is 3.6x stronger and crossrange 2.7x stronger. Those numbers move in the
directions the last three runs actually asked for.

IMPORTANT LIMITATION: these constants live in controller_vq2 and Elodin's harness
maps tilt through its own mixer, so Elodin cannot validate this change at all. It
verified the servo POLICY (when to brake, when to turn) and is blind to the
plant scaling. The only test of these three numbers is the official sim.

Also seen and not yet addressed: one detection at v=34.9 (very high in frame,
conf 0.87) produced vz=+11.53. A gate that high is likely one of the hangar's
ceiling-mounted gates, and the vertical channel obeyed it. Target selection by
"biggest orange blob" still has no notion of which gate belongs to the course.

## VQ2 official run 15 — the approach is SOLVED; two real bugs found

Best approach of the project. The decoupled tilt limits did exactly what they were
meant to:
    pitch -7.2 / -7.7  (run 14 was stuck at -17)   u = 324.0, 320.6, 325.0
Dead centre, gentle, and gate 0 passed cleanly. The brake then fired at full
authority (pitch +30.1 = BRAKE_TILT_RAD) immediately after the pass. Both of the
last two changes worked.

It then climbed at +14.3 m/s (thr 0.39) and hit something (a_up +45, then a string
of >10 g spikes as it tumbled). Two distinct bugs behind that:

1. CEILING GATES. The detector reports the biggest orange blob with no notion of
   which gates are on the course, and this hangar has ceiling-mounted gates
   (plainly visible in run 6 frame 0). The servo locked onto one and the vertical
   channel obediently climbed into it. Run 13 showed the same thing at v=34.9.
   Fix: acquisition now refuses sightings steeper than 25 deg elevation. Gate 0
   measures +14 deg at the gun, so the cut admits the course and rejects the roof.
   MAINTAINING a track is unrestricted, since a gate legitimately rises in view
   as we close on it — the test asserts a track may follow one to 31 deg.

2. THE SEARCH BEHAVIOUR WAS DEAD CODE ON THE OFFICIAL SIM. _race_step did
       if not out.have_target: return 0.0, 0.0, 0.0, self.hover
   discarding tilt_fwd, yaw_rate and vertical whenever the gate was lost. So the
   bounded sweep, the oscillating vertical search and search_creep — all built and
   debugged across runs 7-9 — never executed on the sim. They only ever ran in
   Elodin, whose harness consumes tilt_fwd directly. On the official sim a lost
   gate meant sitting level at hover thrust and coasting, with no attempt to
   reacquire anything.

   That is the sharpest example yet of the harness divergence problem: Elodin
   validated code the sim could not reach. Worth remembering that "tested in
   Elodin" only means "the servo policy is tested" unless the controller actually
   consumes the output.

Elodin, both courses still passing: vq1 6/6 58.96s, vq2turn 4/4 24.92s.

## VQ2 official run 16 — stop inferring tracker state; add the telemetry

Gate 0 passed for the fifth consecutive run with a clean approach (pitch -7.3,
u = 323/320). Then a tumble within one sample of the pass, again.

The detections around the pass were:
    approaching  u=320.0 rng=3.6   then  u=486.9 rng=6.2   (a different gate)
    [GATE] passed 0 -> 1
    post-pass    u= 73.5 rng=2.7   then  u=487.5 rng=1.4   (hopping left/right)

And this log CANNOT say what the servo did with any of them. The DET line reports
raw detector output, which does not distinguish a sighting the tracker accepted
from one it correctly rejected as an impostor. Several runs have now been diagnosed
by inferring tracker state from detector output, and that is exactly the habit that
produced the wrong yaw-sign conclusion.

So the log line now carries the tracker itself:
    TRK az=.. el=.. rng=.. fix=Y/n rej=N SRCH BRK
which shows the smoothed bearing being steered on, whether a track exists, how many
sightings the continuity gate has refused, and whether search/braking are active.
Next run should be readable without guesswork.

Also tried and REVERTED: clear_gate_s 0.15 -> 0.25, on the theory that 1.2 m at
8 m/s is too little to get clear of the gate structure before roll and yaw begin.
It took the VQ1 replica 6/6 -> 5/6 and vq2turn 4/4 -> 2/4. The window is genuinely
knife-edged: gate 1's bearing leaves the FOV within ~2 m of the gate-0 plane, so
every extra 0.1 s of not steering costs about a metre of that budget. Which also
settles a question — the post-pass tumbles are NOT caused by insufficient
clearance, because widening it only loses the next gate. Comment in servo.py now
records the measurement so it is not retried.

Both courses restored: vq1 6/6 58.96s, vq2turn 4/4 24.92s.

## VQ2 official run 17 — the telemetry paid for itself immediately

TRK confirmed two fixes are working and exposed two new bugs. This is the first
run diagnosed from data rather than inference.

WORKING:
  * The continuity gate held gate 0 cleanly and rejected the mid-approach
    impostor: TRK az +0.4 -> +0.1 -> +0.9, rng 5.6 -> 4.0 -> 1.1, rej=1 while DET
    was reporting u=487 rng=6.1. Exactly the intended behaviour, and previously
    unobservable.
  * The elevation cut refused the v=40.8 sighting right after the pass (fix=n).

NEW BUG 1 — BRAKING LATCHED ON PERMANENTLY.
    TRK az=+29.9 el=+24.6 rng=14.5 fix=Y rej=15  BRK
    TRK az=-32.4 el=+28.1 rng= 1.1 fix=Y rej=24  BRK
    TRK az=-35.2 el=+56.8 rng= 0.9 fix=Y rej=24  BRK
BRK appears on every line after the pass. The target hopped between +-30 deg so
|az_f| never fell below brake_az_rad, and the aircraft held 30 deg nose-up while
the elevation channel commanded thr=0.39 max climb. Nose-up plus max climb with a
tilted thrust vector is a departure, not a turn. Braking is a TRANSIENT and was
written as though it were a flight mode.
Fix: brake_max_s = 0.7 s of continuous braking, then fly again regardless; and
brake_vert_clamp = 0.25 caps climb authority while nose-up. Test asserts braking
occupies under 35% of a 4 s window with a target pinned off-axis (measured 18%)
and that |vertical| stays inside the clamp.

NEW BUG 2 — THE ELEVATION CUT WAS TOO LOOSE. 25 deg admitted sightings at +24.6
and +28.1 deg, one of which was then tracked to +56.8 deg. Gate 0 reads +14.0 deg
at the gun (now measurable on the TRK line), so the cut is 18 deg. Test asserts
run 17's +24.6 deg sighting no longer acquires while gate 0's +14 deg still does.

Also visible and not yet addressed: rej went 15 -> 24 within a second and az
flipped +29.9 -> -32.4, i.e. reject_timeout_s is dropping the track and
re-acquiring on the opposite side. The tracker is thrashing in an environment full
of gates. A track that has been rejecting for 0.7 s currently gets discarded; it
may be better to keep coasting on it than to hand the aircraft to whatever is
brightest.

Elodin, both courses improved (braking no longer latching costs less time):
    vq1      6/6  57.49s  (was 58.96)
    vq2turn  4/4  21.13s  (was 24.92)

## VQ2 official run 18 — the brake was never armed; speed now governs speed

Nick: "Never really slowed down or changed course towards the second gate." The
telemetry says exactly why: BRK does not appear ONCE in the whole log.

Braking was armed only by |az_f| > brake_az_rad (26 deg). On the approach az is ~0
because we are aimed at the gate, and the post-pass targets sat at -10 and -25 deg,
all inside the threshold. So a brake built to fix excess speed could only fire when
badly mis-aimed, and the aircraft therefore arrived at gate 0 at full speed on
every single run. Two commits' worth of braking work had no effect on the problem
it was written for.

Speed has to be governed by SPEED. Closure rate is observable — it is the
derivative of the tracked range — so braking now also triggers on closing faster
than max_closure, regardless of aim.

Range caveat that matters for the threshold: rng comes from the detected OUTER
frame width divided by the 1.5 m INNER opening, so it reads ~1.6x short and so
does closure. The official sim's ~8 m/s true is ~5 m/s measured.

max_closure is PLANT-SPECIFIC and defaults to OFF, the same treatment
BRAKE_TILT_RAD needed. Setting 2.5 m/s globally took the VQ1 replica 6/6 -> 1/6,
because Elodin's aircraft cruises at ~2.6 m/s and so braked permanently.
controller_vq2 now sets 4.0, which brakes the official sim's 5 m/s measured
closure and leaves Elodin's 2.6 alone. Third time a shared constant has meant two
different things in the two harnesses; the pattern is that anything expressed in
plant units belongs to the plant, not the servo.

Also confirmed working from the same log: the 18 deg elevation cut refused the
post-pass sightings at v=88.8 and v=51.5 (fix=n, SRCH), where the old 25 deg cut
would have acquired them.

Elodin, both courses still passing and faster:
    vq1      6/6  56.48s  (was 57.49)
    vq2turn  4/4  21.13s  (unchanged)

## VQ2 official run 19 — the continuity gate was locking itself out

TRK made the failure legible: the track acquired gate 0 correctly at el=+3.8 deg
and 5.5 m, then the SAME track read el=+48.3 at 4.2 m and +31.6 at 1.0 m, and the
vertical channel commanded max climb (thr 0.39) while the aircraft was already
committed to the gap.

Two real bugs, one of which explains several earlier runs.

BUG 1 — ELEVATION WAS NEVER CHECKED FOR CONTINUITY. _continuous() gated azimuth
and range and not elevation, so a track could walk from a course gate onto a
ceiling gate as long as azimuth and range stayed plausible. Acquisition was
already elevation-limited; maintaining was not, which left the same hole one step
later. Now gated, with a test on run 19's actual numbers.

BUG 2 — THE GATE COMPARED AGAINST A LAGGING ESTIMATE, AND LOCKED ITSELF OUT.
az_f/el_f/rng_f are EMA-smoothed and therefore lag. Instrumenting a gate closing
at 11 m/s:
     t  obs_rng  rng_f  d_rng  allowR
  0.21      9.6   10.7   1.66    1.84
  0.28      8.8   10.7   1.88    1.84  REJ
  0.42      7.2    9.7   2.54    1.84  REJ
  0.77      3.2    9.7   6.54    6.04  REJ
The EMA lag alone exceeds the allowance under fast closure. Worse, rng_f only
updates on ACCEPT, so the first rejection freezes it and every later sighting is
refused for good. THAT is where runs 17 and 18 got rej counts of 24, 32 and 39,
and why reject_timeout_s kept discarding good tracks and re-acquiring on whatever
was brightest. It was never target ambiguity; the gate was eating its own track.

Fix: compare against the track PREDICTED FORWARD to now (x_f + x_rate * gap) for
all three quantities. A genuinely rising, fast-closing gate now goes from 7
rejections to 0, while run 19's 44 deg elevation jump is still refused.

Also tried and REMOVED: a terminal-commit fade scaling corrections down inside
2.0 m. It was aimed at run 19's el=+48.3, but that symptom's cause was bug 1 — the
track had walked onto a different gate, not a geometry blow-up we had to live with.
With bug 1 fixed the fade treats a problem that no longer exists, and it cost the
VQ1 replica 6/6 -> 2/6 by suppressing the corrections needed to line up on a
descending course. Removed, and the reasoning recorded in servo.py.

Worth noting how this went right: three changes landed together, VQ1 regressed,
and isolating the one with the weakest surviving justification restored it. The
instrumented print that found bug 2 took two minutes and settled what several runs
of reasoning had not.

Elodin, both courses passing: vq1 6/6 57.49s, vq2turn 4/4 21.20s.

## VQ2 official run 20 — and an honest stocktake

Gate 0 passed for the seventh consecutive run. Nothing beyond it, in twenty runs.

What run 20 added: el reached +57.2 deg again DESPITE the new elevation continuity
check, and converting the track to metres shows why that is not a bug at all —
0.33 m, then 1.70 m, then 1.86 m of genuine vertical miss. At 1.2 m range a 1.86 m
miss simply IS a 57 deg angle. The continuity gate was right to allow it.

That points at a real design flaw: servoing the vertical channel on an ANGLE makes
the loop gain scale with 1/range. The same 1.0 m miss reads 4.8 deg at 12 m and
39.8 deg at 1.2 m, so the response is feeble while there is time to fix it and
saturated once there is not. Implemented a metre-based channel, swept the gains
against the plant model (kp 0.70 / kd 0.50, worst miss 0.03 m versus 0.37 m for
angle) — and it took the VQ1 replica 6/6 -> 2/6, because on a course descending
8.6 m between gates the offset is ~8.5 m at long range and the channel saturates
far from the gate.

REVERTED, with the reasoning kept in servo.py. The right form is almost certainly
time-to-go normalisation (null the miss by arrival, ~2*offset/t_go^2) rather than
distance alone, which is proper terminal guidance and a real piece of work rather
than a constant tweak.

PATTERN WORTH NAMING: that is the fourth consecutive Elodin regression from a
change that was individually well-argued — pre-emptive braking, the commit fade,
brake_tilt scaling, and now the metre channel. Each helped the specific failure in
the last log and hurt the general case. This is overfitting to the most recent
run, and the two-course check is the only thing catching it. The check is doing its
job; the generating process is the problem.

Current state, both Elodin courses passing: vq1 6/6 57.49s, vq2turn 4/4 21.20s.
Twenty official runs: gate 0 reliable, gate 1 never reached.

## Corridor detection: the cyan floor guide as a navigation primitive

Nick raised the cyan floor lines and confirmed there is no time limit. Both
change the problem, and the second one especially: nearly every crash since run
12 traces to arriving somewhere too fast to correct, and speed was only ever
there because I assumed racing mattered.

WHAT THE CYAN ACTUALLY IS. Two bright cyan stripes painted on the hangar floor
running the length of the course ("rails"), with the lane between them being the
route ("corridor"). Separately, a cyan ribbon is visible THROUGH the gate opening
showing where the course continues. Those are my names, not the sim's.

Measured on frame 9 of run 6, taking the midpoint of the leftmost and rightmost
cyan pixel per image row:
    row    left  right  width  centre
    350      69    573    504     321
    320      94    548    454     321
    280     128    514    386     321
    240     161    480    319     320
    200     196    446    250     321
    170     222    420    198     321
Lane centre is 320-321 at every row against an image centre of 320. Six
independent estimates agreeing to a pixel, versus a gate centroid that jitters by
tens of pixels and periodically jumps to a different object.

Do NOT use the row-wise centroid of cyan pixels: it is dragged by the ribbon and
by one rail having more pixels than the other. Leftmost-to-rightmost midpoint is
what gives the +-1 px stability.

corridor.py measured across all 47 run-6 frames, 12.5 ms/frame (the gate detector
is 14 ms):
    frames 1-8, stationary on the pad:  lateral +0.1 px, repeated exactly
    frame 9,  race start:               +1.0
    frame 11, still approaching gate 0: +20.8 px, LOOKAHEAD +85.7 px
    frame 12, gate filling the view:    coverage collapses to 0.14
Frame 11 is the important one: the right-hander is announced while gate 0 is
still ahead of us. Twelve runs have died because gate 1 is only visible at the
frame edge for about two metres AFTER the pass. This sees the turn before it.

Coverage collapsing at frame 12 is expected and fine — that is the point where
the gate itself is the better cue anyway.

WHY THIS MATTERS ARCHITECTURALLY. Every recurring failure of the last twenty runs
— impostor tracks, the continuity gate eating itself, ceiling gates, the reject
timeout thrash, the two-metre window — is downstream of navigating by orange
blobs in a hangar full of identical orange gates. None of them exist for a
corridor there is only one of.

MISSION=corridor flies exactly like race and only LOGS the corridor; it touches no
control output. Deliberate: find out whether +-0.1 px survives real flight, and
whether the corridor continues past gate 0, before rebuilding navigation on it.

Open questions this run should answer:
  * does the corridor persist along the whole course, or only near the start?
  * does rail separation track height above the floor? If so it is an ALTITUDE
    estimate, which VQ2 does not provide and whose absence has caused a third of
    our crashes.

## Corridor run 1 (MISSION=corridor) — the corridor runs the whole course

38 frames from the official sim, replayed offline with tools/replay_frames.py
--corridor. Both open questions answered.

1. PRE-RACE CYAN IS FINE. Frames 1-8 give lat=+0.1 px and 2-18% cyan, identical
   to run 6. The live log's "cyan=0.00%" was reading frames that had not rendered
   yet, not a detector failure. Nothing to fix.

2. THE CORRIDOR CONTINUES PAST GATE 0. Frame 13 is decisive: the aircraft is at
   gate 0 with orange filling the left edge, and the cyan corridor sweeps RIGHT
   and runs on into the distance past several more gates. One continuous ribbon
   through the course. In the live log it "vanished" only because the aircraft had
   already crashed.

   In flight the lookahead behaved as it did offline: +9.5 -> +43.8 px across
   frames 10-13, announcing the right-hander while gate 0 was still ahead.

THREE CORRECTIONS TO THE PLAN:

  * It is a CURVED RIBBON, not two straight parallel rails. The
    leftmost/rightmost-per-row method tracks the bend accurately, but "width"
    means ribbon width, which is why w_near was so erratic (363 -> 126 -> 57 ->
    135). DROP the altitude-from-rail-separation idea: the width is dominated by
    how much near field is in frame, not by height above the floor.

  * FRAMES 36-37 ARE A TRAP. They report the highest coverage in the whole set
    (0.91 and 1.00, 4.4% and 13.1% cyan) and they are the aircraft lying INVERTED
    after the crash — ceiling panels visible top and bottom. lat=+78.3 there is
    meaningless. Without an attitude gate, the best-looking corridor measurements
    in a run are the ones taken after it ended. The IMU attitude is available, so
    the corridor must be refused when |roll| exceeds ~45 deg.

  * Only 5 valid in-flight frames (9-13) because the whole flight lasted about
    1.5 s. Flying slowly, which the no-time-limit answer permits, would multiply
    the data from a single run.

VERDICT: the corridor is good enough to navigate on. It is continuous, unambiguous
(there is only one), covers 2-18% of the frame against a gate blob's <1%, and
shows the upcoming turn before the current gate is passed. Every recurring failure
of the last twenty runs is downstream of navigating by orange blobs; none of them
apply here.

## Slow flight — and two latent bugs it exposed

Nick confirmed VQ2 has no time limit. Nearly every crash since run 12 came from
arriving somewhere too fast to correct, and that speed was only ever there because
I assumed racing mattered.

  cruise_tilt 0.55 -> 0.35, and max_closure 4.0 -> 1.25 measured (~2 m/s true;
  range reads ~1.6x short because it divides the detected OUTER frame width by
  the 1.5 m INNER opening).

SPEED BRAKING IS A REGULATOR, NOT A TRANSIENT. Both brake reasons shared
brake_max_s, so the speed brake quit after 0.7 s while still closing at 6 m/s —
useless for the one job it exists for. Azimuth braking stays time-bounded (run 17
showed it latching on forever); speed braking now holds until the speed is down.
Test: 93% of a 3 s window while still closing, versus 18% for the azimuth case.

Slow flight then exposed two bugs that momentum had been hiding:

1. IT FLEW BACK THROUGH A GATE IT HAD ALREADY PASSED. In vq2turn it cleared gate 1
   at (23.2,-10.7), swept round, and re-approached GATE 0 at (11.8,-1.1).
   sweep_limit_max_rad was 3.4 rad, enough to point the aircraft back down the
   course, after which the detector offers the nearest gate — behind us. There is
   no notion of "gates behind us are not targets", so the sweep must not create
   the opportunity: capped at 1.75 rad (+-100 deg). Momentum used to carry us
   through a search; at 2 m/s nothing does.

2. FORWARD TILT WAS DOING TWO JOBS AT ONCE. search_creep translates AND pitches
   the nose down to see below the flight path, so cutting it (needed, or the
   aircraft wanders 18 m during a search) blinded the search to low gates —
   caught by the "gate below" case, which failed at 6.58 m. Descending physically
   finds low gates without needing nose-down attitude, so search_descend goes
   0.18 -> 0.30 and the two jobs are decoupled.

Also: the kinematic servo test flew a fixed 8 m/s with a 25 s budget, set when the
aim was to race. Updated to 3 m/s and 60 s, the regime actually flown. Worth being
careful here — I first assumed the "gate below" failure was a stale time budget,
and it was not: at 6.58 m it was a real capability loss, and the fix was bug 2
rather than a looser test.

Corridor also gets an attitude gate: run-22 frames 36-37 scored the best coverage
in the set (0.91, 1.00) while lying inverted after the crash, reporting lat=+78 px.
Refused above 45 deg of roll or pitch.

Elodin, both courses COMPLETE and much slower, as intended:
    vq1      6/6  57.49s -> 78.48s
    vq2turn  4/4  21.20s -> 54.49s

## Slow-flight run 1 on the official sim — braking worked, and did not help

The changes took effect: BRK is set on the first flight sample and pitch reaches
+29.9 deg, the full BRAKE_TILT_RAD nose-up. It braked as designed and still hit
gate 0. Nick: "It flew super fast and crashed into the first gate."

TWO THINGS THE DATA SUPPORTS:

1. THE CORRIDOR WORKS IN FLIGHT. On the one upright frame before the crash:
       [COR] lat=+5.1px (+0.9deg) look=-2.5px w_near=357 cov=1.00 cyan=2.52% rows=44
   cov=1.00 — every one of 44 sampled rows measured, lane centre 5 px off centre.
   The best corridor reading yet, and from a real flight frame rather than a
   saved one. The attitude gate also behaved: "no corridor" on every subsequent
   line, which is correct, since the aircraft was past 45 deg of roll.

2. THE VERTICAL CHANNEL IS THE PROXIMATE KILLER, AGAIN. el=+37.8 deg at the FIRST
   sighting (rng 4.8), then +51.3, with vz=+4.12 and the aircraft climbing into
   the gate. Acquisition is capped at 18 deg, so it acquired something lower and
   the track walked up — the same walk-up as run 19. The elevation continuity
   gate slows that walk but does not stop it, because a gate genuinely does rise
   in view as you close on it, and a rate limit cannot separate the two cases.

WHAT I CANNOT EXPLAIN: why the ground to gate 0 was covered so quickly while
braking throughout. The plausible reading is that the pad sits close enough to
gate 0 that no braking authority helps, but I have made confident claims about
speed before and been wrong, so this is recorded as unknown rather than acted on.

Assessment: gate-blob navigation has now failed in enough distinct ways that
patching it further is not the best use of runs. The corridor is measured,
continuous, unambiguous, and just returned a perfect reading in flight. The plan
agreed with Nick — corridor as primary steering, gates demoted to a terminal
height cue and progress signal — is the next thing to build, and it removes the
whole class of failure that the elevation walk-up belongs to.

## Corridor-primary steering (MISSION=cornav)

Split into two pieces on purpose: corridor.py does the image processing and needs
numpy; corridor_nav.py holds the control law and deliberately does not, so it is
testable anywhere the rest of the pure-Python suite runs. The interface is three
floats — lane bearing, lookahead bearing, coverage.

The corridor decides WHERE TO GO. The gate is demoted to the vertical channel plus
the progress signal, and braking still overrides pitch, because shedding speed
outranks steering.

Law: yaw and roll BOTH driven by the lane bearing (waiting for the nose to come
round then for forward tilt to push the new way is what made every gate-servo turn
go wide), plus a feed-forward term on the lookahead so a centred lane that BENDS
still turns. That last part is the whole advantage over gate-chasing: the turn is
taken before the next gate is reachable rather than after it has left the frame.
Below min_coverage the lane is refused and the gate servo keeps flying.

14 tests in tests/test_corridor_nav.py, all pure Python: sign conventions both
ways, lookahead feed-forward, coverage gating, immediate hand-back when the
corridor is lost, command limits, and closed-loop convergence (25 deg lane offset
settles inside 0.44 deg).

MISSIONS: race (gate servo, unchanged), corridor (flies like race, only MEASURES
the lane), cornav (steers on it). Kept separate so the gate-servo baseline stays
flyable and a bad outcome costs a run rather than the working configuration.

LIMITATION, STATED PLAINLY: ELODIN CANNOT VALIDATE THIS. elodin_servo has no
camera and no corridor, so both courses exercise the gate servo and say nothing
about cornav. Every previous change was checked on two courses before flying;
this one has unit tests and nothing else. The proper fix is to synthesise a
corridor in the Elodin harness from the true gate positions — the corridor is
essentially the path through them — which would let both courses exercise the same
control law. That is the next piece of work if cornav does not fly well.

## cornav run 1 crashed on an unbound local — and the test was blind to it

    UnboundLocalError: cannot access local variable 'thrust'

The corridor branch returned `thrust` before it was computed. Trivial to fix
(compute it first; the corridor overrides steering only and the gate keeps the
vertical channel).

The reason it SHIPPED is the part worth recording. The smoke test stubbed
detect_corridor to return None, so cmd.have_lane was never true and the branch
containing the bug never executed. The test was green and had no coverage of the
code it existed to cover. A stub that skips the code under test is worse than no
test at all, because it reads as coverage.

Fixing that exposed a second layer: even with the stub returning a valid
observation, the branch stayed unreachable, because CORRIDOR_LOG and CORRIDOR_NAV
are computed at IMPORT time from MISSION. Production sets MISSION via env before
import so that is correct there, but the test assigns C.MISSION afterwards, which
leaves the flags stale. Exactly the late-binding trap that made an earlier test
silently fly yaw_sign=-1 in all four cases.

With both fixed, MISSION=cornav engages LANE on 13 log lines in the smoke test,
so the corridor path is genuinely exercised.

Standing lesson: after stubbing something out, check the stub still lets the code
under test RUN. Grepping the test output for the behaviour (here, "LANE") is the
cheap way to confirm it.

## cornav run 1 — corridor steering WORKS; the vertical channel is now the sole failure

LANE engaged for three consecutive samples and did its job:

    [COR] lat= +0.0px look= +3.7px cov=0.82 rows=36   TRK az=+0.4 el= +2.9  LANE
    [COR] lat= +3.0px look= +7.7px cov=0.66 rows=29   TRK az=+0.3 el=+22.9  LANE BRK
    [COR] lat=+20.2px look=+28.9px cov=1.00 rows=44   TRK az=+3.1 el=+29.8  LANE BRK

Lateral tracking held (0 -> 3 -> 20 px), the lookahead climbed +3.7 -> +28.9
announcing the right-hander, and braking engaged. The corridor primitive works in
closed loop on the official sim, not just on saved frames.

And it isolates the remaining failure exactly. Steering is now the corridor's job
and it is fine; the VERTICAL channel is still gate-driven, and el ran
+2.9 -> +22.9 -> +29.8 -> +53.2 deg with the aircraft climbing over gate 0. Same
walk-up that ended runs 19 and 21. Splitting the responsibilities turned a
confusing multi-cause failure into a single-cause one.

Fix follows from geometry already established: at long range el is small and
genuine (gate 0 reads +14 deg at the gun, 12 m out), while at short range it blows
up whatever the true miss is — and a track walking onto a higher object is
indistinguishable from that. A fixed clamp on the COMMANDED elevation separates
the two without needing a range rule: it barely bites where el is real, and hard
where it is an artefact. max_el_cmd_rad = 0.26 (15 deg).

Test asserts both directions: 53 deg (run 19's actual figure) commands 0.21 rather
than the 0.70 ceiling, while 10 deg still commands a real correction.

Elodin both COMPLETE: vq1 6/6 77.01s, vq2turn 4/4 54.49s.

## Elodin can now test corridor navigation — and cornav halves the vq2turn time

The gap that mattered: elodin_servo has no camera, so MISSION=cornav shipped to
the official sim with unit tests and nothing else, right after four consecutive
well-argued changes had each regressed something these courses caught.

Closed by synthesising the corridor from ground truth. The corridor IS the path
through the gates, so it projects exactly as observe_from_truth projects a gate:
points on the remaining-gate polyline at 4 m and 14 m ahead, converted to
body-relative bearings, with coverage falling off as the lane nears the FOV edge —
which is how real coverage behaves when the corridor leaves the image.

corridor_nav.blend() now holds the corridor/gate-servo combination and BOTH the
controller and the harness call it. They drifted apart once before (elodin_servo
never called gate_passed and projected only the active gate, so Elodin was
validating code the sim could not reach); a shared function makes that impossible
rather than merely unlikely.

RESULTS:
    vq2turn  cornav  4/4 COMPLETE  22.64s   (gate servo: 4/4, 54.49s)
    vq1      cornav  6/6 COMPLETE  64.58s   (gate servo: 6/6, 77.01s)
                     pass times 9.55 / 19.11 / 30.79 / 46.17 / 55.45 / 64.58

cornav passes vq2turn in UNDER HALF the gate servo's time — not by flying faster
but by not wandering: the lane is continuously visible, so the searching that ate
thirty seconds between gates simply does not happen.

TWO TRAPS RECORDED:
  * The first version imported quat_rotate_inv INSIDE the per-cycle bearing
    helper. Elodin went 5014x behind real-time and produced one log line in four
    minutes. Module-level imports only in the control path.
  * The Mac ran out of disk mid-session (26 MB free of 926 GB) and Bash could not
    even create its own output file, which blocked cleanup as well as work. Elodin
    logs accumulate fast; delete /tmp/*.log between sessions.

## Both courses complete under corridor navigation

    course    gate servo        cornav
    vq1       6/6  77.01s       6/6  64.58s
    vq2turn   4/4  54.49s       4/4  22.64s

cornav is faster on BOTH, and it is not flying faster — cruise_tilt is identical.
It is not wandering. The gate servo loses its target after every pass and burns
seconds sweeping for the next one; the corridor is continuously visible, so there
is nothing to search for. On vq2turn that difference is the whole 32 seconds.

This is the first change in the project that improves both courses at once. Every
recent one traded a win on the failing course for a regression on the other, which
is what overfitting to the last log looks like.

Corridor navigation is now verified the way everything before it was: two courses,
full suite, offline, before Nick spends a run.

## cornav run 3 — the corridor is proven; braking INTO the gate is the last bug

Best tracking of the project:
    [COR] lat= -0.3px cov=1.00 cyan=6.66% rows=44   LANE
    [COR] lat= +9.8px cov=0.89 cyan=3.48% rows=39   LANE BRK
Lane centre held to a third of a pixel with every sampled row measuring. Gate 0
passed. Then roll +174.6 one sample later — the same immediate post-pass tumble
seen on every run since 15.

What the attitude says about that tumble: the aircraft went through the gate at
pitch +30.0 then +30.4. That is BRAKE_TILT_RAD at full deflection, i.e. it was
threading a 1.5 m opening at 30 deg nose-up with the tail hanging low. That is a
very good way to catch the bottom of the gate with the rear props, and it explains
why the tumble is immediate and universal rather than gradual.

Braking is correct on approach and wrong at contact. Suppressed inside
brake_inhibit_range = 4.0 m measured (~6 m true): once committed, fly through
level. Test asserts both halves — still brakes on the approach, never inside the
commit range.

Elodin unchanged by the fix, both still COMPLETE:
    vq2turn  4/4  22.64s
    vq1      6/6  64.58s

Worth noting what the corridor bought: for the first time the failure is a single
identifiable event with a physical mechanism, rather than a tangle of impostor
tracks, elevation walk-ups and search thrash. Navigation being solid is what made
the remaining bug legible.

## MISSION=creep — Nick's state machine, built and unit-tested; not yet flying

    SEEK     hold station, sweep for an orange gate
    ALIGN    kill speed, centre the gate in az and el, NO forward drive
    TRANSIT  short forward pulses, attitude level, VISION IGNORED
    PIVOT    hold station, yaw onto the cyan lane, -> SEEK

Why this is the right shape. The continuous blend produced a run of MARGINAL
failures: run 27 braked at pitch +27 deg because the tracked range read 4.3 m
against a 4.0 m inhibit — 0.3 m decided the crash. Phases do not have that
property. And the deeper point: we kept correcting at ranges where corrections
cannot help. Every gate crash came from reacting to a sighting inside the last two
metres, so TRANSIT ignores vision outright.

17 unit tests pass, including the load-bearing ones: only a stopped and centred
aircraft commits; a 23 deg azimuth error or 4 m/s of closure does not; TRANSIT
ignores a 52 deg azimuth and 50 deg elevation sighting entirely.

TWO PULSE BUGS, one found by Elodin and one I should have caught in the test I
wrote:
  1. pulse_on 0.45 s at +0.45 against pulse_off 0.55 s at -0.45 nets -0.045 every
     cycle. The aircraft reversed 75 m down the course. My own test printed "mean
     tilt -0.045" and I read it as "close to zero" — a systematically signed mean
     is a direction, not zero. Impulses now match by construction and the test
     asserts on_s*tilt == off_s*brake directly rather than on a loose tolerance.
  2. With impulses matched it no longer reverses, but it does not TRANSLATE
     either: 100 s in TRANSIT, position still near the start, drifting up to
     z=6.8. Either the phase is thrashing ALIGN<->TRANSIT (transit_timeout_s is
     30 s and should have fired), or drag eats the ~0.22 m of displacement each
     cycle should produce. UNRESOLVED — do not fly this until it is.

race and cornav are untouched and still the flyable configurations; cornav remains
vq1 6/6 64.58s and vq2turn 4/4 22.64s.

## creep: TRANSIT solved, ALIGN now the blocker — NEXT STEP IS SPECIFIC

Phase instrumentation (creep._enter logs every transition with a reason) turned a
mystery into a sequence. Two findings:

1. TRANSIT WAS NEVER BROKEN. It translates at ~0.33 m/s, so 12 m to gate 0 needs
   ~36 s, and transit_timeout_s was 30 — the timeout fired a few seconds before
   arrival and threw it back to ALIGN, forever. "Does not translate" was
   "translating fine and being interrupted". Now 120 s.

2. ALIGN NOW NEVER COMMITS:
       [PHASE] ALIGN -> SEEK at t=25.03 (align timeout az=-0.4 el=+0.1 closure=0.11)
   az, el and closure are ALL well inside tolerance (0.05 rad, 0.06 rad, 0.6 m/s),
   so the blocker is the align_el_rate_tol condition I added. Two attempts at it
   failed: first computing el_rate per control cycle (wrong — el only changes at
   detector rate, so the derivative is zero between frames and spikes on each
   update), then differentiating across sightings as servo.py does. Neither
   changed the outcome, so el_rate may not be the culprit at all.

   THE NEXT DIAGNOSTIC IS TO LOG THE FOUR SUB-CONDITIONS SEPARATELY. `aligned` is
   an AND of four terms and the log only shows three of them; settled_t resets on
   any single flicker, and at Elodin's control rate 0.5 s of settle is hundreds of
   consecutive cycles. One print of (az_ok, el_ok, rate_ok, closure_ok, settled_t)
   will name the term that never holds. Guessing at it has now cost three runs —
   instrument first.

race and cornav remain the flyable configurations and are untouched.

## creep FLIES: align -> commit -> pass -> pivot, working end to end

    [PHASE] ALIGN -> TRANSIT  t= 4.95  (aligned az=-0.6 el=-1.1 closure=0.00)
    [PHASE] TRANSIT -> PIVOT  t=11.45  (gate 1 reached)
    [PHASE] PIVOT -> SEEK     t=13.84
    [PHASE] ALIGN -> TRANSIT  t=18.93  (aligned az=-1.3 el=+0.9 closure=0.00)
    [PHASE] TRANSIT -> PIVOT  t=27.01  (gate 2 reached)
    [PHASE] ALIGN -> TRANSIT  t=32.84  (aligned az=+0.4 el=+1.3 closure=0.00)

Positions confirm real traversal: (0,0) -> (12.8,0.0) -> (18.5,-6.6) ->
(25.0,-11.9), straight down the course and round the right-hander. Nick's
sequence works exactly as described.

FIVE BUGS, each a different cause behind the same symptom, each found by
instrumenting rather than reasoning:
  1. Unequal pulse impulses (0.45*0.45 vs 0.55*0.45) reversed it 75 m. My own
     test printed the -0.045 bias and I read it as "close to zero".
  2. TRANSIT timeout 30 s < the ~36 s the creep needs — interrupted just before
     arrival, every time. "Does not translate" was "being interrupted".
  3. ALIGN never committed: el_rate tolerance 0.05 set from theory when the
     measured noise floor is +-0.14. Logging the four AND-terms separately named
     it in one run, after three guesses had failed.
  4. Pulse period shorter than the attitude loop's settling time — it jittered in
     pitch without translating.
  5. vertical=0 in TRANSIT means hover thrust, which holds vertical VELOCITY not
     height: it sank 3.4 m to the floor and sat there. "Ignore vision" was too
     broad — it must mean ignore STEERING, not stop holding altitude.
  6. Symmetric pulses cancel against drag: forward builds speed, drag caps it,
     the equal reverse pushes it back. x oscillated +-0.5 m for 95 s. COASTING
     instead of braking makes displacement strictly positive while the 50% duty
     cycle still bounds speed.

DISK: the 163 GB that stopped work repeatedly was elodin writing a per-run
telemetry database (betaflight_dbNNN, 2.6-3.7 GB EACH) into the sim directory,
never cleaned up. Fifty runs filled a 926 GB disk. I blamed the sim logs twice
and Downloads once, and had Nick delete caches, a Windows ISO and frame archives,
while `du -sh ~/code/AIGP/elodin` would have answered it the first time.
run_elodin.sh now removes them before and after every run (trap on EXIT).
