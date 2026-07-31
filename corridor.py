"""The cyan floor corridor: a navigation primitive that is always there.

Twenty official runs navigated by orange gate blobs, and every recurring failure
was a consequence of that choice — impostor tracks, the continuity gate eating
itself, ceiling gates, and gate 1 being visible for roughly two metres at the
edge of the frame. The hangar floor carries two bright cyan stripes running the
length of the course, and they do not have any of those problems:

    gate blob                      cyan corridor
    ~84 px wide, often <1%         2-18% of the frame
    appears and vanishes           continuous while upright
    a hangar full of identical     one corridor, nothing to confuse it with
    next gate at the FOV edge      the turn is visible through the current gate

Measured on frame 9 of run 6, taking the midpoint of the leftmost and rightmost
cyan pixel in each image row:

    row    left  right  width  centre
    350      69    573    504     321
    320      94    548    454     321
    280     128    514    386     321
    240     161    480    319     320
    200     196    446    250     321
    170     222    420    198     321

The lane centre reads 320-321 at every row against an image centre of 320: six
independent estimates of "am I aligned with the course", agreeing to a pixel.
Width collapsing 504 -> 198 px is plain perspective, which is what makes an
altitude estimate plausible later — that being the one quantity VQ2 refuses to
give us and the cause of a third of our crashes.

Deliberately NOT taking the row-wise centroid of cyan pixels: it gets dragged
around by the ribbon through the gate and by one rail carrying more pixels than
the other. Leftmost-to-rightmost is what produced the +-1 px stability above.

numpy + Pillow only. opencv has no win_arm64 wheels at any version, so the VM
cannot have it.
"""

import math
from dataclasses import dataclass, field

import numpy as np

# Cyan in HSV. The gates are orange/red (hue >335 or <25) and the guide lines sit
# near 180, so the two are about as far apart in hue as colours get — no shared
# threshold to tune and no chance of one detector stealing the other's pixels.
HUE_LO_DEG = 160.0
HUE_HI_DEG = 215.0
SAT_MIN = 0.35
VAL_MIN = 0.30

# Rows are sampled rather than scanned: 360 rows of numpy indexing is wasteful
# when 40 give the same geometry.
ROW_STEP = 6
ROW_TOP = 96            # above this is mostly ceiling and gantry
MIN_ROW_PIXELS = 6      # cyan pixels in a row before it is worth measuring
MIN_ROW_WIDTH = 25      # px; narrower means we are seeing ONE rail, not two,
                        # and its midpoint would be a lie

# Bands used for steering and for lookahead. Near rows say where we are now;
# far rows say where the course is going. The gap between them is the turn.
NEAR_ROW_MIN = 260
FAR_ROW_MAX = 200

CAM_CX = 320.0
CAM_FX = 320.0


@dataclass(frozen=True)
class CorridorObservation:
    """Corridor geometry from one frame. Angles are raw camera-relative."""
    rows: tuple = field(default=())        # (row, left, right, centre, width)
    centre_near: float = float("nan")      # px, mean lane centre in near rows
    centre_far: float = float("nan")       # px, mean lane centre in far rows
    width_near: float = float("nan")       # px, rail separation in near rows
    coverage: float = 0.0                  # fraction of sampled rows measured
    cyan_frac: float = 0.0                 # fraction of the frame that is cyan

    @property
    def valid(self) -> bool:
        return not math.isnan(self.centre_near)

    @property
    def lateral_px(self) -> float:
        """Where the lane centre sits relative to the camera axis, near field.

        Positive means the course is to our RIGHT, i.e. we have drifted left.
        """
        return self.centre_near - CAM_CX

    @property
    def lateral_rad(self) -> float:
        return math.atan2(self.lateral_px / CAM_FX, 1.0)

    @property
    def lookahead_px(self) -> float:
        """Far lane centre minus near: which way the course bends ahead.

        Positive means the course turns RIGHT. Measured on run 6 approaching
        gate 0: -14 px at frame 11, then +85 px at frame 12 as the right-hander
        came into view — announced while gate 0 was still ahead of us, rather
        than glimpsed at the frame edge after passing it.
        """
        if math.isnan(self.centre_far) or math.isnan(self.centre_near):
            return float("nan")
        return self.centre_far - self.centre_near


def cyan_mask(rgb: np.ndarray) -> np.ndarray:
    """Boolean mask of cyan pixels. Hand-rolled HSV: no cv2 available."""
    a = rgb.astype(np.float32) / 255.0
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    mx = a.max(2)
    mn = a.min(2)
    d = np.maximum(mx - mn, 1e-6)
    sat = np.where(mx > 1e-6, (mx - mn) / np.maximum(mx, 1e-6), 0.0)

    h = np.zeros_like(mx)
    m = mx == r
    h[m] = (((g - b) / d)[m]) % 6.0
    m = mx == g
    h[m] = (((b - r) / d + 2.0)[m])
    m = mx == b
    h[m] = (((r - g) / d + 4.0)[m])
    h *= 60.0
    return (h > HUE_LO_DEG) & (h < HUE_HI_DEG) & (sat > SAT_MIN) & (mx > VAL_MIN)


def detect_corridor(rgb: np.ndarray) -> CorridorObservation:
    """Measure the cyan corridor. Always returns an observation; check .valid."""
    mask = cyan_mask(rgb)
    height, width = mask.shape
    cyan_frac = float(mask.mean())

    rows = []
    sampled = 0
    for y in range(ROW_TOP, height, ROW_STEP):
        sampled += 1
        row = mask[y]
        if row.sum() < MIN_ROW_PIXELS:
            continue
        xs = np.nonzero(row)[0]
        left = float(xs[0])
        right = float(xs[-1])
        if right - left < MIN_ROW_WIDTH:
            continue          # one rail only; its midpoint means nothing
        rows.append((y, left, right, 0.5 * (left + right), right - left))

    if not rows:
        return CorridorObservation(cyan_frac=cyan_frac)

    near = [r for r in rows if r[0] >= NEAR_ROW_MIN]
    far = [r for r in rows if r[0] <= FAR_ROW_MAX]
    mean = lambda vs: float(sum(vs) / len(vs)) if vs else float("nan")

    return CorridorObservation(
        rows=tuple(rows),
        centre_near=mean([r[3] for r in near]),
        centre_far=mean([r[3] for r in far]),
        width_near=mean([r[4] for r in near]),
        coverage=len(rows) / max(sampled, 1),
        cyan_frac=cyan_frac,
    )
