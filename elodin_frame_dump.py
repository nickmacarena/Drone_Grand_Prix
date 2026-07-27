"""Stage 3 data capture: dump labeled FPV frames from an Elodin lap.

Control is delegated to the proven elodin_solver, so the drone flies a normal
VQ1-replica lap while we save camera frames plus the pose/track truth needed
to label them. The result is an offline dataset: gate detection can then be
developed and scored with zero sim runs, the same way the synthetic IMU test
validated the estimator.

    tools/run_elodin.sh vq1 /tmp/dump.log 150 elodin_frame_dump

Writes to $FRAME_DIR (default /tmp/aigp_frames):
    frame_0000.png   RGBA 640x360
    labels.jsonl     one JSON object per frame (pose, gates, active index)
"""

import json
import os

import numpy as np
from PIL import Image

from solver.api import RCCommand, SensorUpdate  # elodin repo package
from sim.course import active_course            # elodin repo package

import elodin_solver

FRAME_DIR = os.environ.get("FRAME_DIR", "/tmp/aigp_frames")
CAPTURE_PERIOD_S = float(os.environ.get("CAPTURE_PERIOD_S", "0.4"))

_COURSE, _SPAWN, _SIM_TIME = active_course()
GATES = [list(g.center) for g in _COURSE]

os.makedirs(FRAME_DIR, exist_ok=True)
_labels_path = os.path.join(FRAME_DIR, "labels.jsonl")
_labels = open(_labels_path, "w")
_n = [0]
_next_capture = [0.0]


def reset_state() -> None:
    elodin_solver.reset_state()


def autopilot(update: SensorUpdate) -> RCCommand:
    cmd = elodin_solver.autopilot(update)

    if (update.frame_rgba is not None
            and update.frame_fresh
            and update.t >= _next_capture[0]):
        _next_capture[0] = update.t + CAPTURE_PERIOD_S
        idx = _n[0]
        _n[0] += 1

        arr = np.asarray(update.frame_rgba)
        Image.fromarray(arr, mode="RGBA").save(
            os.path.join(FRAME_DIR, f"frame_{idx:04d}.png"))

        # world_pos is [qx, qy, qz, qw, x, y, z] (Elodin ENU / FLU body)
        wp = [float(v) for v in update.world_pos]
        _labels.write(json.dumps({
            "i": idx,
            "t": round(float(update.t), 3),
            "quat_xyzw": wp[0:4],
            "pos_enu": wp[4:7],
            "next_gate_index": int(update.next_gate_index),
            "gates_enu": GATES,
        }) + "\n")
        _labels.flush()

        if idx % 25 == 0:
            print(f"  [DUMP] {idx} frames  t={update.t:.1f}", flush=True)

    return cmd
