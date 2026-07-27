"""Gate bearing: the one thing vision must deliver, and the camera geometry.

VQ2 gives no position and no gate coordinates, so control cannot fly to a
point — it can only fly toward what it SEES. That reduces the entire vision
requirement to a narrow interface:

    frame  ->  GateObservation(az, el, width_px, confidence)  ->  control

`observe_from_truth()` produces exactly that from known geometry, so the
servoing loop (Stage 3a) can be built and tuned against a perfect detector
while the real image detector (Stage 3b) is developed separately against
authoritative VQ2 frames. Swapping one for the other touches no control code.

Pure Python (math only) — must run on the VM, which has no numpy for 3.14.

Camera basis vectors are body-frame config, so one model serves both sims:
    Elodin FLU (x fwd, y left, z up)  — see ELODIN_CAM
    AIGP   FRD (x fwd, y right, z down) — see AIGP_CAM
Both honour VADR-TS-002 3.8: 640x360, fx=fy=320, cx=320, cy=180, +20 deg
up-tilt. (The spec's prose "VFoV = 90" contradicts its own intrinsics, which
give HFoV = 90 and VFoV = 58.7; the intrinsics win — they are unambiguous.)
"""

import math
from dataclasses import dataclass

from attitude import (
    quat_rotate,
    quat_rotate_inv,
    vec_cross,
    vec_dot,
    vec_norm,
    vec_unit,
)

CAM_WIDTH = 640
CAM_HEIGHT = 360
CAM_FX = 320.0
CAM_FY = 320.0
CAM_CX = 320.0
CAM_CY = 180.0
CAM_TILT_UP_DEG = 20.0
GATE_INNER_W = 1.5  # metres, VADR-TS-002 3.7


@dataclass(frozen=True)
class CameraModel:
    """Camera axes expressed in the BODY frame, plus pinhole intrinsics."""
    fwd: tuple      # optical axis
    right: tuple    # +u in the image
    down: tuple     # +v in the image
    fx: float = CAM_FX
    fy: float = CAM_FY
    cx: float = CAM_CX
    cy: float = CAM_CY
    width: int = CAM_WIDTH
    height: int = CAM_HEIGHT


def _tilted_axes(tilt_deg: float, up_is_plus_z: bool) -> tuple:
    """Camera basis for a forward camera pitched `tilt_deg` upward."""
    t = math.radians(tilt_deg)
    if up_is_plus_z:  # FLU: y is LEFT, z is UP
        fwd = (math.cos(t), 0.0, math.sin(t))
        right = (0.0, -1.0, 0.0)
        down = (math.sin(t), 0.0, -math.cos(t))
    else:             # FRD: y is RIGHT, z is DOWN
        fwd = (math.cos(t), 0.0, -math.sin(t))
        right = (0.0, 1.0, 0.0)
        down = (math.sin(t), 0.0, math.cos(t))
    return CameraModel(fwd=fwd, right=right, down=down)


ELODIN_CAM = _tilted_axes(CAM_TILT_UP_DEG, up_is_plus_z=True)
AIGP_CAM = _tilted_axes(CAM_TILT_UP_DEG, up_is_plus_z=False)


@dataclass(frozen=True)
class GateObservation:
    """What a gate detector reports. Angles are camera-relative, radians."""
    az: float           # + = gate is right of the optical axis
    el: float           # + = gate is above the optical axis
    u: float            # pixel coords, for debugging/overlay
    v: float
    width_px: float     # apparent gate width; range proxy (see range_m)
    confidence: float   # 0..1

    @property
    def range_m(self) -> float:
        """Range implied by apparent width. Crude but monotone, and it is the
        only distance cue a monocular camera has without structure."""
        if self.width_px <= 1e-6:
            return float("inf")
        return CAM_FX * GATE_INNER_W / self.width_px


def observe_from_truth(cam: CameraModel, q_body_to_world, drone_pos, gate_pos,
                       edge_margin_px: float = 40.0):
    """Project a known gate into the camera. Returns None if not visible.

    Stands in for a real detector during Stage 3a. `q_body_to_world` is
    [w,x,y,z]; positions share whatever world frame the caller uses.
    """
    rel_world = (gate_pos[0] - drone_pos[0],
                 gate_pos[1] - drone_pos[1],
                 gate_pos[2] - drone_pos[2])
    dist = vec_norm(rel_world)
    if dist < 1e-6:
        return None

    rel_body = quat_rotate_inv(q_body_to_world, rel_world)

    z = vec_dot(rel_body, cam.fwd)      # depth along optical axis
    if z <= 0.05:
        return None                      # behind the camera
    x = vec_dot(rel_body, cam.right)
    y = vec_dot(rel_body, cam.down)

    u = cam.fx * x / z + cam.cx
    v = cam.fy * y / z + cam.cy
    if not (0.0 <= u < cam.width and 0.0 <= v < cam.height):
        return None                      # outside the frame

    width_px = cam.fx * GATE_INNER_W / dist

    # Confidence fades near the frame edges, where a real detector gets
    # unreliable (partial gate, distortion) — and where servoing is about to
    # lose the target anyway.
    du = min(u, cam.width - 1 - u)
    dv = min(v, cam.height - 1 - v)
    edge = min(du, dv)
    confidence = max(0.0, min(1.0, edge / edge_margin_px))

    return GateObservation(
        az=math.atan2(x, z),
        el=math.atan2(-y, z),   # image +v is down; elevation is up-positive
        u=u, v=v,
        width_px=width_px,
        confidence=confidence,
    )


def dir_body(cam: CameraModel, u: float, v: float):
    """Unit vector toward an image point, in BODY coordinates.

    Depends only on (u, v) and the camera model, so a real image detector
    feeds this exactly like the truth stand-in does.
    """
    x = (u - cam.cx) / cam.fx
    y = (v - cam.cy) / cam.fy
    d = (cam.fwd[0] + x * cam.right[0] + y * cam.down[0],
         cam.fwd[1] + x * cam.right[1] + y * cam.down[1],
         cam.fwd[2] + x * cam.right[2] + y * cam.down[2])
    return vec_unit(d)


def stabilized_bearing(cam: CameraModel, obs: GateObservation,
                       q_body_to_world, up_world):
    """Bearing de-rotated into a LEVEL frame: (az_right, el_up), radians.

    Raw camera az/el are contaminated by the drone's own attitude — a nose-down
    cruise pitch makes a level gate look high, so servoing straight off `el`
    chases your own pitch and climbs away from the gate (observed Stage 3a run
    1: climbed 5 m above gate 0, lost it out of the vertical FOV, never
    recovered). De-rotating with the attitude estimate removes that coupling.

    el_up is 0 when the gate is at our own altitude — a physically meaningful
    setpoint that owes nothing to the camera's mounting tilt.
    az_right is measured from our current heading, positive to the right.
    """
    d_world = quat_rotate(q_body_to_world, dir_body(cam, obs.u, obs.v))

    el_up = math.asin(max(-1.0, min(1.0, vec_dot(d_world, up_world))))

    # Horizontal projections of heading and of the gate direction.
    fwd_world = quat_rotate(q_body_to_world, (1.0, 0.0, 0.0))
    h = vec_unit(_horizontal(fwd_world, up_world))
    g = vec_unit(_horizontal(d_world, up_world))
    if vec_norm(h) < 1e-6 or vec_norm(g) < 1e-6:
        return 0.0, el_up
    az_right = -math.atan2(vec_dot(up_world, vec_cross(h, g)), vec_dot(h, g))
    return az_right, el_up


def _horizontal(v, up_world):
    """Component of v perpendicular to up_world."""
    k = vec_dot(v, up_world)
    return (v[0] - k * up_world[0], v[1] - k * up_world[1], v[2] - k * up_world[2])
