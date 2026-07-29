"""Gate detector: VQ2 camera frame -> GateObservation. numpy + Pillow only.

Real VQ2 imagery (captured 2026-07-27, first official-sim run) turned out to
be far friendlier than the README's "high-fidelity 3D-scanned environments"
suggested: the course is a dark, desaturated indoor hangar and the gates are
saturated orange-red squares. A hue threshold isolates them almost perfectly
with essentially no false positives, so this is classical CV — no ML, no
OpenCV, and cheap enough to run every frame on the VM.

Two useful properties of the target:
  * A gate is a square FRAME, so the centroid of its pixels is the centre of
    its opening — exactly the aim point the servo wants, for free.
  * Apparent width gives monocular range via the known 1.5 m inner opening.

Watch out for: bright white ceiling panels (rectangular, and they photograph
as false positives for any shape-only detector — colour rejects them), and
the cyan guidance lines that thread between gates (a separate hue cluster
near 190 deg; currently ignored, but they are a strong second cue if the gate
colour ever proves insufficient).

    import detector; obs = detector.detect_gate(frame_rgb)

No OpenCV anywhere: opencv-python has no win_arm64 wheels at any version, so
it cannot be installed on the competition VM.
"""

import numpy as np

from bearing import AIGP_CAM, GATE_INNER_W, CameraModel, GateObservation

# Gate colour, measured from real frames: hue wraps through 0 (red-orange).
HUE_LO_DEG = 25.0        # accept hue < this ...
HUE_HI_DEG = 335.0       # ... or hue > this
SAT_MIN = 0.45
VAL_MIN = 0.25

# Connected components run on a downsampled mask for speed; 4x keeps a gate
# at 20 m (~24 px wide) several cells across while cutting work 16-fold.
DOWNSAMPLE = 4
MIN_BLOB_CELLS = 6       # reject speckle
MAX_ASPECT = 4.0         # a gate is roughly square, even seen at an angle


def rgb_to_hsv(a: np.ndarray):
    """Vectorised RGB->HSV. h in degrees, s/v in 0..1."""
    a = a.astype(np.float32) / 255.0
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    mx = a.max(2)
    mn = a.min(2)
    d = mx - mn
    s = np.where(mx > 0, d / np.maximum(mx, 1e-6), 0.0)
    h = np.zeros_like(mx)
    m = d > 1e-6
    i = m & (mx == r); h[i] = ((g - b)[i] / d[i]) % 6.0
    i = m & (mx == g); h[i] = ((b - r)[i] / d[i]) + 2.0
    i = m & (mx == b); h[i] = ((r - g)[i] / d[i]) + 4.0
    return h * 60.0, s, mx


def gate_mask(frame_rgb: np.ndarray) -> np.ndarray:
    h, s, v = rgb_to_hsv(frame_rgb)
    return ((h < HUE_LO_DEG) | (h > HUE_HI_DEG)) & (s > SAT_MIN) & (v > VAL_MIN)


def _downsample_any(mask: np.ndarray, k: int) -> np.ndarray:
    """Block-reduce with OR: a cell is set if any pixel in it is set."""
    hh, ww = mask.shape
    hh -= hh % k
    ww -= ww % k
    return mask[:hh, :ww].reshape(hh // k, k, ww // k, k).any(axis=(1, 3))


def _components(small: np.ndarray):
    """Two-pass connected components (8-connected) with union-find.

    Pure numpy/Python so it runs on the VM; on a 160x90 grid this is well
    under a millisecond, which matters at 30 fps.
    """
    hh, ww = small.shape
    labels = np.zeros((hh, ww), dtype=np.int32)
    parent = [0]

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)

    nxt = 1
    for y in range(hh):
        for x in range(ww):
            if not small[y, x]:
                continue
            neigh = []
            if y > 0:
                for dx in (-1, 0, 1):
                    xx = x + dx
                    if 0 <= xx < ww and labels[y - 1, xx]:
                        neigh.append(labels[y - 1, xx])
            if x > 0 and labels[y, x - 1]:
                neigh.append(labels[y, x - 1])
            if neigh:
                lab = min(neigh)
                labels[y, x] = lab
                for n in neigh:
                    union(lab, n)
            else:
                labels[y, x] = nxt
                parent.append(nxt)
                nxt += 1

    out = {}
    for y in range(hh):
        for x in range(ww):
            l = labels[y, x]
            if l:
                out.setdefault(find(l), []).append((x, y))
    return list(out.values())


def detect_gate(frame_rgb: np.ndarray, cam: CameraModel = AIGP_CAM,
                edge_margin_px: float = 40.0):
    """Best gate sighting in this frame, or None.

    With several gates in view the nearest (largest) is chosen: gates behind
    us are not in a forward camera, so the biggest blob is the one we are
    flying at.
    """
    mask = gate_mask(frame_rgb)
    if not mask.any():
        return None

    small = _downsample_any(mask, DOWNSAMPLE)
    blobs = _components(small)
    if not blobs:
        return None

    best = None
    for cells in blobs:
        if len(cells) < MIN_BLOB_CELLS:
            continue
        xs = [c[0] for c in cells]
        ys = [c[1] for c in cells]
        w = (max(xs) - min(xs) + 1) * DOWNSAMPLE
        h = (max(ys) - min(ys) + 1) * DOWNSAMPLE
        if w <= 0 or h <= 0:
            continue
        aspect = max(w / h, h / w)
        if aspect > MAX_ASPECT:
            continue                      # long thin thing: a guide line, not a gate
        if best is None or w * h > best[0]:
            best = (w * h, xs, ys, w, h)

    if best is None:
        return None
    _, xs, ys, w, h = best

    # Refine on the full-resolution mask inside the blob's bounding box: the
    # centroid of a square frame is the centre of its opening.
    x0 = min(xs) * DOWNSAMPLE
    x1 = min(mask.shape[1], (max(xs) + 1) * DOWNSAMPLE)
    y0 = min(ys) * DOWNSAMPLE
    y1 = min(mask.shape[0], (max(ys) + 1) * DOWNSAMPLE)
    sub = mask[y0:y1, x0:x1]
    ys_f, xs_f = np.nonzero(sub)
    if len(xs_f) == 0:
        return None
    u = float(xs_f.mean() + x0)
    v = float(ys_f.mean() + y0)
    width_px = float(xs_f.max() - xs_f.min() + 1)

    du = min(u, cam.width - 1 - u)
    dv = min(v, cam.height - 1 - v)
    confidence = max(0.0, min(1.0, min(du, dv) / edge_margin_px))

    import math
    x_n = (u - cam.cx) / cam.fx
    y_n = (v - cam.cy) / cam.fy
    return GateObservation(
        az=math.atan2(x_n, 1.0),
        el=math.atan2(-y_n, 1.0),
        u=u, v=v,
        width_px=width_px,
        confidence=confidence,
    )
