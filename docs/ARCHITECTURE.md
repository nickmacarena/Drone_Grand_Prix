# Architecture

## Purpose

This document describes the design of the Drone Grand Prix autonomous flight pipeline. It is a living reference — update it when decisions change, and record *why* in JOURNAL.md.

---

## Pipeline Overview

The pipeline is a linear data flow. Each stage transforms inputs into outputs and has no knowledge of the stages around it.

```
camera_frame  ──→ perception  ──→ gate_observations
odometry      ──┘                       │
                                        ↓
              odometry  ──→  planner  ──→  target_waypoint
                                        │
                                        ↓
              odometry  ──→  controller ──→  position_setpoint
                                        │
                                        ↓
                               MAVLink interface ──→ sim
```

Each stage answers one question:
- **Perception:** what do I see?
- **Planner:** where should I go?
- **Controller:** what command achieves that?

These stages change for different reasons. Perception changes when the vision model improves. The planner changes when racing strategy changes. The controller changes when tuning for speed vs. stability. Keeping them separate means a change in one does not ripple into the others.

---

## Design Principles

**Functional over OOP.** Pipeline stages are functions, not classes. Each stage takes inputs and returns outputs with no shared mutable state. This makes stages independently testable, swappable, and easy to reason about.

**Pure logic separate from side effects.** The `pipeline/` layer contains pure functions. The `io/` layer is where side effects live — reading telemetry from MAVSDK, writing commands to the sim. Pipeline stages can be tested without a running simulator.

**Stubs enable parallel development.** Because stages have clean interfaces, any stage can be replaced with a hardcoded stub. This means planning and control can be developed and tuned before perception is working.

**Build vertically, not horizontally.** Complete a thin slice through the full pipeline before deepening any individual stage. A stubbed-but-wired pipeline is more valuable than a polished perception module with nothing downstream.

---

## Repo Structure

```
Drone_Grand_Prix/
├── pipeline/
│   ├── perception.py    # camera frame → gate observations
│   ├── planner.py       # gate observations + drone state → target waypoint
│   └── controller.py    # target waypoint + drone state → position setpoint
├── io/
│   ├── telemetry.py     # MAVSDK streams → DroneState
│   └── commands.py      # position setpoint → MAVLink commands
├── types.py             # shared data types: DroneState, GateObservation, Waypoint
├── main.py              # wires pipeline together, runs async loop
├── connect.py           # connectivity diagnostic, not part of pipeline
└── tests/
    ├── test_perception.py
    ├── test_planner.py
    └── test_controller.py
```

**`pipeline/`** contains pure functions with no MAVSDK imports. Testable without the sim.

**`io/`** contains all MAVSDK interaction. This is the only layer that changes when swapping from PX4 SITL to the competition sim.

**`types.py`** defines the contracts between stages. Every stage imports from here. Changing a type here surfaces all the places that need to be updated.

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

---

## The Async Loop

MAVSDK is async. The pipeline stages are synchronous pure functions. The async loop in `main.py` is responsible for:
1. Reading the latest telemetry from MAVSDK into a `DroneState`
2. Reading the latest camera frame
3. Calling the pipeline stages in order
4. Sending the resulting setpoint to the sim

The loop runs at the command rate specified in the tech spec (50–120 Hz).

---

## Development Phasing

### Phase 1 — Vertical slice (current)
Get a stubbed pipeline wired end-to-end:
- Stub perception returns hardcoded gate positions
- Stub planner returns the next hardcoded waypoint
- Controller sends `SET_POSITION_TARGET_LOCAL_NED` via MAVSDK offboard
- Drone flies a scripted path through the gates

### Phase 2 — Perception
- Get camera feed from Gazebo
- Implement gate detection on real frames
- Replace stub perception with real observations

### Phase 3 — Planning
- Replace hardcoded waypoints with waypoints derived from gate observations
- Handle sequencing (which gate is next?)
- Handle the case where no gate is visible

### Phase 4 — Tuning and integration
- Tune controller for speed
- Full pipeline running end-to-end on the PX4 SITL course
- Swap io/ layer for competition sim interface

---

## Open Questions

- What does the competition sim's camera stream look like? (Separate spec TBD)
- What gate ID scheme does the competition use, if any?
- How does the planner handle loss of gate detection mid-course?
- What is the async pattern for feeding camera frames into the pipeline?
