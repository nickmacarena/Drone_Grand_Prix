# Architecture

## Purpose

Design of the Drone Grand Prix autonomous flight pipeline. Living reference — update when decisions change, record *why* in JOURNAL.md.

---

## What the sim gives us

This shaped the whole design. Read it first.

- **Track layout** — the sim sends a complete list of gates (id, position NED, orientation, width, height) once at the start, via `ENCAPSULATED_DATA` chunked transfer.
- **Active gate index** — the sim tells us which gate is next via `RACE_STATUS` messages.
- **Position telemetry** — `LOCAL_POSITION_NED` gives us local position and velocity.
- **Attitude telemetry** — `ATTITUDE` gives roll/pitch/yaw (radians) and rates.
- **Collision feedback** — `COLLISION` messages flag gate hits (id 1001) or environment hits (id 1002).

**Control interface — what we learned the hard way:**

The MAVLink spec lists three control modes (position/velocity setpoints, attitude+thrust, motor RPMs), but **only attitude+thrust is actually followed** by this sim. `SET_POSITION_TARGET_LOCAL_NED` is accepted but the drone does not track the setpoint — it just falls.

So we send `SET_ATTITUDE_TARGET` with a target quaternion + thrust at 250 Hz. The sim's inner attitude controller tracks the target. Our job: compute the right attitude quaternion + thrust to fly to each gate.

---

## Pipeline

```
                ┌──────────────────────────────────────┐
                │  mavlink_rx (background thread)      │
                │                                      │
                │  ATTITUDE             → drone_state  │
                │  LOCAL_POSITION_NED   → drone_state  │
                │  HEARTBEAT            → armed/mode   │
                │  ENCAPSULATED_DATA    → track,       │
                │                         race_status  │
                └──────────────┬───────────────────────┘
                               ▼
                       ┌────────────────┐
                       │  SharedState   │
                       └───────┬────────┘
                               ▼
            ┌──────────────────────────────────────┐
            │   controller.update()  (250 Hz)      │
            │                                      │
            │   1. Re-arm if sim disarmed us       │
            │   2. Active gate position → target   │
            │   3. pid.compute_attitude_target()   │
            │      ↓ cascaded loops ↓              │
            │      position → velocity             │
            │      velocity → acceleration         │
            │      acceleration → (q, thrust)      │
            │   4. SET_ATTITUDE_TARGET             │
            └──────────────────────────────────────┘
```

Vision, timesync, heartbeat run as background threads. Vision is wired but unused for VQ1.

---

## Cascaded PID Controller

```
target gate (NED)
    │
    ▼
┌──────────────────────────────────┐
│ POSITION LOOP    (P)             │
│ pos_error → desired_velocity     │  ~10 Hz bandwidth
└──────────────┬───────────────────┘
               ▼
┌──────────────────────────────────┐
│ VELOCITY LOOP    (PD)            │
│ vel_error → desired_acceleration │  ~30 Hz bandwidth
└──────────────┬───────────────────┘
               ▼
┌──────────────────────────────────┐
│ ACCEL → ATTITUDE conversion      │
│ desired_accel + target_yaw →     │  algebraic
│   (q_target, thrust)             │
└──────────────┬───────────────────┘
               ▼
        SET_ATTITUDE_TARGET (250 Hz)
        sim's inner loop tracks quaternion
```

**Why cascaded:** each loop has a clean job. Position loop says "go here." Velocity loop adds damping. Accel→attitude conversion handles gravity properly. Gains are tunable per loop, not magic.

**Why P/PD and not PID:** integral terms accumulate error and add complexity. For a deterministic sim with no steady-state disturbances, P/PD is usually enough. Add I terms only if we observe persistent steady-state error.

---

## System Identification

Before tuning the cascaded controller, we measure key drone parameters with `system_id.py`. This runs as a standalone script — not part of the race pipeline.

What we measure:

1. **Hover thrust** — the thrust value where vertical velocity stabilizes at 0.
2. **Attitude inner-loop response** — when we command pitch=X, how long until the drone reaches X, and is there overshoot?
3. **Thrust → acceleration mapping** — vertical thrust gain (m/s² per unit thrust above hover) and tilt → horizontal acceleration gain.

The measurements live in `measurements.py` as constants. `pid.py` reads them. We re-run system ID when the sim version changes or we suspect drift.

---

## Design Principles

**Pure logic separate from threading.** Pipeline math lives in `pid.py` and `attitude.py` as pure functions — testable without the sim. Threaded plumbing is isolated in the `*_rx`, `*_tx`, `timesync`, `heartbeat` modules.

**Frozen dataclasses for state.** Each field of `SharedState` holds an immutable dataclass replaced atomically by the rx thread. CPython's GIL makes single-field assignment safe — no explicit locks needed.

**Mirror the AIGP example file names where possible.** Teammates can cross-reference the official example.

**Measure before tuning.** Gains derived from measured parameters, not guesses.

---

## Repo Structure

```
Drone_Grand_Prix/
├── main.py              # entry point — connect, run control loop
├── setup.py             # wires components, starts background threads
├── state.py             # frozen dataclasses + SharedState
├── controller.py        # 250 Hz loop, thin — calls into pid.py
├── pid.py               # cascaded PID, pure functions
├── attitude.py          # quaternion + frame math, pure functions
├── measurements.py      # measured drone parameters (from system_id.py)
├── system_id.py         # standalone measurement script (not in race pipeline)
├── mavlink_rx.py        # background MAVLink receiver → SharedState
├── mavlink_tx.py        # MAVLink command senders
├── timesync.py          # background TIMESYNC sender
├── heartbeat.py         # background HEARTBEAT sender (1 Hz)
├── vision_rx.py         # background camera receiver (unused for VQ1)
├── docs/
│   ├── ARCHITECTURE.md
│   ├── JOURNAL.md
│   └── LOCAL_DEV_SETUP.md
└── legacy/              # pre-sim code based on wrong assumptions; do not extend
```

---

## Core Data Types (`state.py`)

**`DroneState`** — position (NED, m), velocity (NED, m/s), attitude (radians). Built from `LOCAL_POSITION_NED` + `ATTITUDE`.

**`Gate`** — id, position (NED, m), orientation (quaternion), width, height. From track data.

**`TrackData`** — tuple of all `Gate`s. Sent once at race start.

**`RaceStatus`** — active gate index, race started/finished flags, last gate time. From `ENCAPSULATED_DATA` race status. `race_started` is true once `sim_boot_time >= race_start_time` (not just when start is *scheduled*).

**`HeartbeatStatus`** — armed flag, base_mode, custom_mode, system_status. From `HEARTBEAT`.

**`SharedState`** — mutable container. Background threads write, control loop reads.

---

## Threading Model

- **Main thread** — runs the control loop, calls `controller.update()` at 250 Hz.
- **`mavlink_rx` thread** — receives all MAVLink messages, updates `SharedState`.
- **`vision_rx` thread** — receives camera frames over UDP 5600, updates `SharedState.latest_frame`. (Wired but unused for VQ1.)
- **`timesync` thread** — sends `TIMESYNC` to the sim at 10 Hz.
- **`heartbeat` thread** — sends `HEARTBEAT` from our side at 1 Hz (MAVLink protocol requirement).

All background threads expose `get_thread_for_join()` for clean shutdown.

---

## Sim-specific protocol notes

- **The sim disarms us at the race-start transition.** Our controller re-arms whenever it sees `armed=False`. Rate-limited so we don't spam.
- **Don't send non-zero attitude/thrust before `race_started=True`.** Early motion = DQ. We hold thrust=0 pre-race.
- **`race_started` is true only when sim time has reached the scheduled start**, not when the start is scheduled. We compare `sim_boot_ms` against `race_start_boot_ms`.

---

## Two-Sim Development Loop (since 2026-06-08)

Hand-tuning against the official sim was a 5-minute-per-iteration loop with
DQ/timing noise. We now develop against the **Elodin practice harness**
(`~/code/AIGP/elodin`, github.com/elodin-sys/ai-grand-prix): real Betaflight
SITL + deterministic physics, runs headless on the Mac in seconds.

```
                      planner.py  (sim-agnostic, lives in THIS repo)
                      (x, y, alt) world frame, normalized effort outputs
                       /                          \
        elodin_solver.py                    controller.py + mavlink_tx.py
        (Elodin adapter)                    (AIGP adapter)
        efforts → RC PWM                    efforts → quaternion + thrust
        gates ← sim/course.py               gates ← ENCAPSULATED_DATA track data
```

- **Development** (logic, gate sequencing, braking, lookahead): hundreds of
  fast iterations in Elodin.
- **Calibration** (hover thrust, tilt→accel, drag): a handful of runs in the
  official sim; constants live in the adapter, not the planner.
- Run our solver in Elodin from the elodin repo root:
  `PYTHONPATH=~/code/AIGP/Drone_Grand_Prix RACE_SOLVER=elodin_solver elodin run sim/main.py`

## Development Phasing

### Phase 1 — Foundations
- ✅ Pipeline, networking, telemetry, gate sequencing
- ✅ Confirm only attitude+thrust is followed by sim
- ✅ Re-arming + race-start handling

### Phase 2 — Planner in Elodin, then port (current)
- ✅ Elodin harness installed, baseline passes 3/3 gates
- Sim-agnostic `planner.py` + `elodin_solver.py` adapter
- Verify 3/3 in Elodin, then port to AIGP adapter + calibrate
- Get through VQ1 reliably

### Phase 3 — VQ2 perception
- Wire vision into the control loop
- Detect gates in camera frames (high-contrast, classical CV likely works)
- Use vision to refine gate positions or detect obstacles

### Phase 4 — Performance
- Trajectory planning across multiple gates (not one-at-a-time targeting)
- Possibly MPC for VQ2 fastest-time scoring

---

## Competition Constraints

- **Control output:** attitude+thrust only (sim ignores position/velocity setpoints)
- **Telemetry inputs:** ATTITUDE, LOCAL_POSITION_NED, ODOMETRY, HIGHRES_IMU, plus custom race/track data
- **No GPS, no absolute positioning, no depth sensor**
- **Monocular FPV camera** via UDP 5600 (JPEG, chunked)
- **Gates must be passed in correct order** — sim provides `active_gate_index`
- **Full 3D course with elevation changes**
- **Max run time:** 8 minutes
- **Sim platform:** Windows only, requires internet (anti-cheat)
- **VQ1:** < 10 gates, focus on completion
- **VQ2:** < 20 gates, visually complex, fastest time wins

---

## Open Questions

- Gate orientation — does it indicate approach direction or gate plane normal?
- How accurate is `LOCAL_POSITION_NED` vs. ground truth? Need to check noise level.
- Does the sim accept attitude commands during the pre-race lobby? Or only after race_started?
- For VQ2, will track data still be provided, or will perception be required to find gates?
