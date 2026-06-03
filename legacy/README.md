# Legacy

Pre-simulator code based on assumptions that turned out to be wrong:
- Used MAVSDK (actual sim uses pymavlink)
- Built a PID controller from scratch (actual sim accepts position/velocity setpoints directly)
- Assumed vision-driven gate detection (actual sim provides full track data)

Kept for reference only. **Do not extend.** See repo root for the active codebase.
