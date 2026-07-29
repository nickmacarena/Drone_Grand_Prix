"""Run the OFFICIAL-SIM VQ2 bring-up sequence inside Elodin.

    tools/run_elodin.sh vq1 /tmp/bringup.log 200 elodin_vq2_bringup

Four official-sim runs have now been spent on bring-up bugs rather than on
flying: sign detection that tumbled the aircraft, a calibration climb that
flew it into the hangar ceiling, and now an unexplained descent between sign
pulses. At five minutes a run that is a bad way to find the fifth one.

This harness drives the SAME `ControllerVQ2` — same calibration, same gyro
sign detection, same vz integration, same servo — from Elodin's IMU, with the
frame conventions injected for FLU/ENU. Elodin's ground truth is used only to
SCORE what the controller does; the controller never sees it.

What it prints that the official sim cannot: true altitude and vertical speed
alongside the controller's own estimates, so a divergence between them is
visible immediately instead of being inferred from a crash.
"""

import math

import numpy as np

from solver.api import RCCommand, SensorUpdate  # elodin repo package

import controller_vq2 as C
from bearing import ELODIN_CAM
from flight_stack import HOVER_CMD, motor_commands
from state import HeartbeatStatus, ImuSample, RaceStatus, SharedState

UP_ENU = (0.0, 0.0, 1.0)
YAW_SIGN_FLU = -1.0

# Elodin's rate loop is ours (flight_stack), so its conventions are already
# consistent: sign detection should come back [+1, +1, +1] here. That is the
# point — it isolates the SEQUENCE from the official sim's sign quirks.


class _FakeConn:
    """Captures what the controller would send over MAVLink."""
    target_system = 1
    target_component = 1

    def __init__(self):
        self.last = None
        self.mav = self

    def set_attitude_target_send(self, *a):
        self.last = a          # (..., mask, q, roll, pitch, yaw, thrust)

    def command_long_send(self, *a):
        pass


_conn = _FakeConn()
_shared = SharedState()
_ctl = C.ControllerVQ2(_conn, _shared, 0,
                       up_world=UP_ENU, yaw_sign=YAW_SIGN_FLU, cam=ELODIN_CAM)
_shared.heartbeat = HeartbeatStatus(armed=True, base_mode=0, custom_mode=0,
                                    system_status=4)

DIRECT_MOTORS = np.array([-1.0, -1.0, -1.0, -1.0])
_t0 = [None]
_log = {"next": 0.0}
RACE_START_S = 1.0          # stand-in for the official sim's race gun


def reset_state() -> None:
    DIRECT_MOTORS[:] = -1.0


def autopilot(update: SensorUpdate) -> RCCommand:
    t = update.t
    if _t0[0] is None:
        _t0[0] = t

    # Drive the controller off Elodin's clock, exactly as it uses time.time().
    C.time.time = lambda: t

    # Elodin's IMU, packaged as the official sim's HIGHRES_IMU. FLU here, so
    # the controller's up_world=(0,0,1) makes the frames consistent.
    g = update.gyro
    a = update.accel
    _shared.imu = ImuSample(
        t_us=int(t * 1e6),
        ax=float(a[0]), ay=float(a[1]), az=float(a[2]),
        gx=float(g[0]), gy=float(g[1]), gz=float(g[2]),
    )
    started = (t - _t0[0]) >= RACE_START_S
    _shared.race_status = RaceStatus(
        active_gate_index=int(update.next_gate_index),
        race_started=started, race_finished=False, last_gate_race_time_s=0.0)
    if update.frame_rgba is not None:
        _shared.latest_frame = np.asarray(update.frame_rgba)[..., :3]

    _ctl.update()

    # Translate the controller's attitude-rate command into motor commands.
    if _conn.last is None:
        DIRECT_MOTORS[:] = HOVER_CMD
        return RCCommand(arm=1000, throttle=1000)
    roll_rate, pitch_rate, yaw_rate, thrust = _conn.last[5:9]

    # The controller closes its own attitude loop and emits BODY RATES, so
    # here we only need a rate follower: hold the commanded rates, pass the
    # commanded thrust through.
    qw, qx, qy, qz = _ctl.est.q
    DIRECT_MOTORS[:] = _rate_follower((qx, qy, qz, qw), g,
                                      roll_rate, pitch_rate, yaw_rate, thrust)

    if t >= _log["next"]:
        _log["next"] += 1.0
        true_alt = float(update.world_pos[6])
        true_vz = float(update.world_vel[5]) if update.world_vel.size > 5 else 0.0
        est_r, est_p, _ = _ctl._est_euler()
        print(f"  [BRINGUP] t={t:5.1f} TRUE alt={true_alt:6.2f} vz={true_vz:+6.2f} | "
              f"EST vz={_ctl.vz_est:+6.2f} err={_ctl.vz_est - true_vz:+6.2f} | "
              f"att=({math.degrees(est_r):+5.1f},{math.degrees(est_p):+5.1f}) "
              f"thr={thrust:.2f} hov="
              f"{_ctl.hover if _ctl.hover else float('nan'):.3f} "
              f"signs={_ctl.rate_sign} settled={_ctl._settled} "
              f"lvl={_ctl._leveled}", flush=True)

    return RCCommand(arm=1000, throttle=1000)


def _rate_follower(quat_xyzw, gyro, rr, pr, yr, thrust):
    """Minimal body-rate follower: torque proportional to rate error."""
    import flight_stack as FS
    w = np.asarray(gyro, dtype=np.float64)
    torque = np.array([
        FS.IXX * FS.KP_RATE * (rr - w[0]),
        FS.IYY * FS.KP_RATE * (pr - w[1]),
        FS.IZZ * FS.KD_YAW * (yr - w[2]),
    ])
    u_roll = torque[0] / (4.0 * FS.F_MAX * FS.ARM)
    u_pitch = torque[1] / (4.0 * FS.F_MAX * FS.ARM)
    u_yaw = torque[2] / (4.0 * FS.F_MAX * FS.K_TORQUE)
    # `thrust` is a normalized 0..1 collective in the official sim's units;
    # HOVER_CMD is Elodin's. Scale so "hover" means hover in both.
    coll = thrust * (HOVER_CMD / max(_ctl.hover or HOVER_CMD, 1e-3))
    m = coll + u_roll * FS.ROLL_MIX + u_pitch * FS.PITCH_MIX + u_yaw * FS.YAW_MIX
    return np.clip(m, 0.0, 1.0)
