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
