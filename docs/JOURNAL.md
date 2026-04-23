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
