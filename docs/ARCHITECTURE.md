# Architecture

## Purpose

This document describes the design of the Drone Grand Prix autonomous flight pipeline. It is a living reference — update it when decisions change, and record *why* in JOURNAL.md.

---

## Pipeline Overview

The pipeline is a linear data flow. Each stage transforms inputs into outputs and has no knowledge of the stages around it.

```
camera_frame  ──→ perception  ──→ gate_observations
                                        │
                                        ↓
        drone_state  ──→  planner  ──→  target_waypoint
                                        │
                                        ↓
        drone_state  ──→  controller ──→  attitude_command
                                        │
                                        ↓
                             MAVLink interface ──→ sim
```

Each stage answers one question:
- **Perception:** what do I see?
- **Planner:** where should I go?
- **Controller:** what throttle/roll/pitch/yaw achieves that?

These stages change for different reasons. Perception changes when the vision model improves. The planner changes when racing strategy changes. The controller changes when tuning for speed vs. stability. Keeping them separate means a change in one does not ripple into the others.

---

## Control Interface

The competition simulator accepts **attitude-level commands only**: throttle, roll, pitch, yaw. This is sent via `SET_ATTITUDE_TARGET`. Position setpoints (`SET_POSITION_TARGET_LOCAL_NED`) are not the primary interface.

This means the controller stage is not a thin wrapper — it must implement position→attitude control, most likely PID loops that take a position error and output the roll/pitch/yaw/throttle needed to close that error.

No GPS or absolute positioning is provided. No depth sensor. The only inputs are the FPV camera stream and telemetry (attitude, IMU, odometry).

---

## Design Principles

**Functional over OOP.** Pipeline stages are functions, not classes. Each stage takes inputs and returns outputs with no shared mutable state. This makes stages independently testable, swappable, and easy to reason about.

**Pure logic separate from side effects.** The `pipeline/` layer contains pure functions. The `sim_io/` layer is where side effects live — reading telemetry from MAVSDK, writing commands to the sim. Pipeline stages can be tested without a running simulator.

**Stubs enable parallel development.** Because stages have clean interfaces, any stage can be replaced with a hardcoded stub. This means planning and control can be developed and tuned before perception is working.

**Build vertically, not horizontally.** Complete a thin slice through the full pipeline before deepening any individual stage. A stubbed-but-wired pipeline is more valuable than a polished perception module with nothing downstream.

---

## Repo Structure

```
Drone_Grand_Prix/
├── pipeline/
│   ├── perception.py    # camera frame → gate observations
│   ├── planner.py       # gate observations + drone state → target waypoint
│   └── controller.py    # target waypoint + drone state → attitude command (PID)
├── sim_io/
│   ├── telemetry.py     # MAVSDK streams → DroneState
│   └── commands.py      # attitude command → SET_ATTITUDE_TARGET
├── drone_types.py       # shared data types
├── main.py              # wires pipeline together, runs async loop
├── connect.py           # connectivity diagnostic, not part of pipeline
└── tests/
    ├── test_perception.py
    ├── test_planner.py
    └── test_controller.py
```

**`pipeline/`** contains pure functions with no MAVSDK imports. Testable without the sim.

**`sim_io/`** contains all MAVSDK interaction. This is the only layer that changes when swapping from PX4 SITL to the competition sim.

**`drone_types.py`** defines the contracts between stages. Every stage imports from here. Changing a type here surfaces all the places that need to be updated.

**`main.py`** is glue only — it imports stages, wires them together, and runs the loop. It contains no pipeline logic.

**`tests/`** tests pipeline logic with known inputs and expected outputs. No simulator required.

---

## Core Data Types

Defined in `types.py`. These are the contracts between pipeline stages. All fields are subject to revision — update this section when types change.

**`DroneState`** — telemetry snapshot at a point in time
- position (NED, meters from origin)
- velocity (NED, m/s)
- attitude (roll, pitch, yaw in degrees)
- timestamp

**`GateObservation`** — a gate detected in a single camera frame
- estimated position relative to drone (NED, meters)
- confidence (0.0–1.0)
- gate ID (if determined)

**`Waypoint`** — a target position for the planner to navigate toward
- position (NED, meters from origin)
- heading (yaw, degrees) — optional

**`AttitudeCommand`** — output of the controller, sent to the sim
- throttle (0.0–1.0)
- roll (degrees)
- pitch (degrees)
- yaw (degrees)

---

## The Async Loop

MAVSDK is async. The pipeline stages are synchronous pure functions. The async loop in `main.py` is responsible for:
1. Reading the latest telemetry from MAVSDK into a `DroneState`
2. Reading the latest camera frame
3. Calling the pipeline stages in order
4. Sending the resulting `AttitudeCommand` to the sim via `SET_ATTITUDE_TARGET`

The loop runs at the command rate specified in the tech spec (50–120 Hz).

---

## Development Phasing

### Phase 1 — Vertical slice
Get a stubbed pipeline wired end-to-end:
- Stub perception returns hardcoded gate positions
- Stub planner returns the next hardcoded waypoint
- Controller implements basic PID: position error → attitude command
- Drone flies through hardcoded gate sequence using attitude commands

### Phase 2 — Controller tuning
- Tune PID gains for stable flight
- Test with elevation changes (full 3D)
- Get hover, takeoff, and waypoint-to-waypoint transitions working reliably

### Phase 3 — Perception
- Get camera feed from Gazebo
- Implement gate detection (monocular — no depth sensor available)
- Estimate gate distance from known gate dimensions + apparent size in frame
- Replace stub perception with real observations

### Phase 4 — Planning
- Replace hardcoded waypoints with waypoints derived from gate observations
- Handle gate sequencing (gates must be passed in correct order)
- Handle the case where no gate is visible

### Phase 5 — Integration and competition prep
- Full pipeline running end-to-end
- Swap `sim_io/` layer for competition sim interface (Windows only)
- VQ1 goal: complete the course (< 10 gates), speed is secondary

---

## Competition Constraints

- **Control output:** throttle, roll, pitch, yaw only (SET_ATTITUDE_TARGET)
- **Inputs:** FPV camera stream + telemetry (attitude, IMU, odometry)
- **No GPS, no depth sensor, no absolute position**
- **Monocular vision only**
- **Gates must be passed in correct order**
- **Full 3D course with elevation changes**
- **Max run time:** 8 minutes
- **No human interaction during runs**
- **Competition sim:** Windows only, downloadable, runs locally, requires internet (anti-cheat)
- **VQ1:** < 10 gates, minimal distractions, focus on completion
- **VQ2:** < 20 gates, increased complexity (lighting, obstacles)

---

## Open Questions

- What does the competition sim's camera stream look like? (Separate spec TBD)
- What gate ID scheme does the competition use, if any?
- How does the planner handle loss of gate detection mid-course?
- What is the async pattern for feeding camera frames into the pipeline?
- PID controller structure: cascaded position→velocity→attitude loops, or direct position→attitude?
