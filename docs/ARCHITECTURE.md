# Architecture

## Purpose

Design of the Drone Grand Prix autonomous flight pipeline. Living reference — update when decisions change, record *why* in JOURNAL.md.

---

## What the sim gives us

This shaped the whole design. Read it first.

- **Track layout** — the sim sends a complete list of gates (id, position NED, orientation, width, height) once at the start, via `ENCAPSULATED_DATA` chunked transfer.
- **Active gate index** — the sim tells us which gate is next via `RACE_STATUS` messages.
- **Position telemetry** — `LOCAL_POSITION_NED` gives us local position and velocity. No GPS needed.
- **Attitude telemetry** — `ATTITUDE` gives roll/pitch/yaw (radians) and rates.
- **Collision feedback** — `COLLISION` messages flag gate hits (id 1001) or environment hits (id 1002).
- **Three control levels** — position/velocity setpoints (`SET_POSITION_TARGET_LOCAL_NED`), attitude rates + thrust (`SET_ATTITUDE_TARGET`), or direct motor RPMs (`SET_ACTUATOR_CONTROL_TARGET`).

**For VQ1, this means we do not need computer vision at all.** Track data + race status + position telemetry is enough. Vision becomes relevant for VQ2 (visually complex environments, obstacles).

---

## Pipeline

```
                ┌──────────────────────────────────────┐
                │  mavlink_rx (background thread)      │
                │                                      │
                │  ATTITUDE          → drone_state     │
                │  LOCAL_POSITION_NED → drone_state    │
                │  ENCAPSULATED_DATA → track_data,     │
                │                      race_status     │
                └──────────────┬───────────────────────┘
                               │
                               ▼
                       ┌────────────────┐
                       │  SharedState   │
                       └───────┬────────┘
                               │
                               ▼
            ┌──────────────────────────────────┐
            │  controller.update()  (250 Hz)   │
            │                                  │
            │  1. Read active gate from        │
            │     race_status                  │
            │  2. Look up gate position in     │
            │     track_data                   │
            │  3. Compute velocity vector      │
            │     from drone to gate           │
            │  4. Send SET_POSITION_TARGET_    │
            │     LOCAL_NED with velocity      │
            └──────────────────────────────────┘
```

Vision and timesync run as background threads but don't feed the VQ1 control loop. Vision populates `SharedState.latest_frame` for future use.

---

## Design Principles

**Mirror the AIGP example structure.** Teammates can reference the official example one-to-one. File names and roles match.

**Pure logic separate from threading.** `controller._compute_velocity()` is a pure function of `SharedState` and is testable. The threaded plumbing (`mavlink_rx`, `vision_rx`, `timesync`) is isolated.

**Frozen dataclasses for state.** Each field of `SharedState` holds an immutable dataclass replaced atomically by the rx thread. CPython's GIL makes single-field assignment safe — no explicit locks needed.

**No premature complexity.** Start with velocity setpoints and trust the sim's stabilized controller. Move to attitude or motor-level control only if we hit a performance ceiling.

---

## Repo Structure

```
Drone_Grand_Prix/
├── main.py                # entry point
├── setup.py               # wires components together (mirrors example)
├── controller.py          # 250 Hz control loop, computes velocity toward active gate
├── mavlink_rx.py          # background MAVLink receiver → SharedState
├── mavlink_tx.py          # MAVLink command senders
├── vision_rx.py           # background camera receiver (stub for VQ1)
├── timesync.py            # background TIMESYNC sender
├── state.py               # data types and SharedState container
├── requirements.txt
├── README.md
├── VQ1_tech_specs.pdf
├── docs/
│   ├── ARCHITECTURE.md
│   ├── JOURNAL.md
│   └── LOCAL_DEV_SETUP.md
├── tests/
└── legacy/                # pre-sim code based on wrong assumptions; do not extend
```

---

## Core Data Types (`state.py`)

**`DroneState`** — position (NED, m), velocity (NED, m/s), attitude (radians). Built from `LOCAL_POSITION_NED` + `ATTITUDE`.

**`Gate`** — id, position (NED, m), orientation (quaternion), width, height. From track data.

**`TrackData`** — tuple of all `Gate`s. Sent once at race start.

**`RaceStatus`** — active gate index, race started/finished flags, last gate time. From `ENCAPSULATED_DATA` race status messages.

**`SharedState`** — mutable container with `drone_state`, `track_data`, `race_status`, `latest_frame`. Background threads write, control loop reads.

---

## Threading Model

- **Main thread** — runs the control loop, calls `controller.update()` at 250 Hz.
- **`mavlink_rx` thread** — receives all MAVLink messages, updates `SharedState`.
- **`vision_rx` thread** — receives camera frames over UDP 5600, updates `SharedState.latest_frame`.
- **`timesync` thread** — sends `TIMESYNC` to the sim at 10 Hz.

All background threads expose `get_thread_for_join()` for clean shutdown.

---

## Development Phasing

### Phase 1 — VQ1 baseline (current)
- Velocity setpoint toward active gate
- No vision
- Goal: complete the VQ1 course

### Phase 2 — VQ1 tuning
- Smarter gate approach (use gate orientation to align before passing)
- Look ahead to next gate to smooth turns
- Handle race start/finish edge cases

### Phase 3 — VQ2 perception
- Wire vision into the control loop
- Detect gates in camera frames
- Use vision to refine gate positions or detect obstacles

### Phase 4 — Performance
- Move from velocity to attitude or motor control if needed for speed
- Tune for fastest time (VQ2 scoring)

---

## Competition Constraints

- **Control output:** velocity/position, attitude rates+thrust, or motor RPMs
- **Telemetry inputs:** ATTITUDE, LOCAL_POSITION_NED, ODOMETRY, HIGHRES_IMU, plus custom race/track data
- **No GPS, no absolute positioning, no depth sensor**
- **Monocular FPV camera** via UDP 5600 (JPEG, chunked)
- **Gates must be passed in correct order** — sim provides `active_gate_index`
- **Full 3D course with elevation changes**
- **Max run time:** 8 minutes
- **No human interaction during runs**
- **Sim platform:** Windows only, requires internet (anti-cheat)
- **VQ1:** < 10 gates, focus on completion
- **VQ2:** < 20 gates, visually complex, fastest time wins

---

## Open Questions

- How accurate is `LOCAL_POSITION_NED` vs. ground truth? If noisy, may need to fuse with ODOMETRY.
- Does the active gate index change as soon as we pass through, or with a delay?
- Gate orientation — does it indicate the approach direction or the gate plane normal?
- For VQ2, will track data still be provided, or will perception be required?
