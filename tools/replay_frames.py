"""Replay the detector (and optionally the servo) over captured VQ2 frames.

    python tools/replay_frames.py <frame_dir> [--servo]

Every image-driven failure so far has been findable ONLY on the official sim:
Stage 3a validated the servo against synthetic bearings, which are continuous
by construction, and Elodin's render bridge is dead so no image ever reaches
the servo there. That left a five-minute cycle for vision bugs.

This closes the gap. FRAME_DIR captures a JPEG per detector call plus a JSONL
sidecar with the paired IMU and race status, so a run can be re-flown offline
as many times as we like. Run 6's spin was diagnosed with it in one pass: the
detector was fine, the vertical loop was overshooting.

Needs numpy+Pillow, so run it on the dev machine, not the VM.
"""
import argparse
import json
import math
import pathlib
import sys
import time

import numpy as np
from PIL import Image

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import corridor          # noqa: E402
import detector          # noqa: E402
import servo             # noqa: E402
from bearing import AIGP_CAM, GateObservation, stabilized_bearing  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("frame_dir")
    ap.add_argument("--servo", action="store_true",
                    help="also run the servo, showing accepted/rejected sightings")
    ap.add_argument("--corridor", action="store_true",
                    help="measure the cyan floor corridor instead of gates")
    args = ap.parse_args()

    d = pathlib.Path(args.frame_dir)
    sidecar = d / "labels.jsonl"
    labels = {}
    if sidecar.exists():
        labels = {json.loads(l)["i"]: json.loads(l)
                  for l in sidecar.read_text().splitlines() if l.strip()}

    if args.corridor:
        print(f"{'frm':>4} {'run':>5} {'valid':>6} {'lat_px':>7} {'look':>7} "
              f"{'w_near':>7} {'cov':>5} {'cyan%':>6}")
        print("-" * 54)
        t0, n = time.time(), 0
        for p in sorted(d.glob("frame_*.jpg")):
            i = int(p.stem.split("_")[1])
            lab = labels.get(i, {})
            k = corridor.detect_corridor(
                np.asarray(Image.open(p).convert("RGB")))
            n += 1
            run = str(lab.get("race_started", ""))[:5]
            if not k.valid:
                print(f"{i:>4} {run:>5} {'no':>6} {'':>7} {'':>7} {'':>7} "
                      f"{k.coverage:5.2f} {k.cyan_frac * 100:5.2f}%")
                continue
            look = k.lookahead_px
            lk = f"{look:+7.1f}" if not math.isnan(look) else "    ---"
            print(f"{i:>4} {run:>5} {'YES':>6} {k.lateral_px:+7.1f} {lk} "
                  f"{k.width_near:7.0f} {k.coverage:5.2f} "
                  f"{k.cyan_frac * 100:5.2f}%")
        print(f"\n{n} frames, {(time.time() - t0) / max(n, 1) * 1000:.1f} ms/frame")
        return 0

    cfg, state = servo.ServoConfig(), servo.ServoState()
    hdr = f"{'frm':>4} {'gate':>4} {'run':>5} | {'u':>6} {'v':>6} {'w_px':>6} {'rng':>6} {'conf':>5}"
    if args.servo:
        hdr += f" | {'took':>5} {'yaw':>6} {'vert':>6}"
    print(hdr)
    print("-" * len(hdr))

    t0, n = time.time(), 0
    for p in sorted(d.glob("frame_*.jpg")):
        i = int(p.stem.split("_")[1])
        lab = labels.get(i, {})
        img = np.asarray(Image.open(p).convert("RGB"))
        det = detector.detect_gate(img)
        n += 1
        row = (f"{i:>4} {lab.get('next_gate_index', -1):>4} "
               f"{str(lab.get('race_started', ''))[:5]:>5} |")
        if det is None:
            print(row + "   ---- no detection")
            continue
        rng = det.range_m if hasattr(det, "range_m") else 320.0 * 1.5 / det.width_px
        row += (f" {det.u:6.1f} {det.v:6.1f} {det.width_px:6.1f} "
                f"{rng:6.1f} {det.confidence:5.2f}")
        if args.servo:
            # No attitude in the sidecar beyond raw IMU, so use the gravity
            # direction as a level reference — enough to exercise the gate.
            before = state.rejected
            tgt = servo.Target(az=math.atan2((det.u - 320.0) / 320.0, 1.0),
                              el=math.atan2(-(det.v - 180.0) / 320.0, 1.0),
                              confidence=det.confidence, range_m=rng)
            out = servo.step(state, cfg, i * 0.5, tgt)
            took = "rej" if state.rejected > before else "ok"
            row += f" | {took:>5} {out.yaw_rate:+6.2f} {out.vertical:+6.2f}"
        print(row)

    print(f"\n{n} frames, {(time.time() - t0) / max(n, 1) * 1000:.1f} ms/frame")
    if args.servo:
        print(f"sightings rejected by the continuity gate: {state.rejected}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
