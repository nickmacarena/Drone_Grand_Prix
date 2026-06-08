"""Measured drone parameters.

These come from running system_id.py against the live sim.
Last measured: 2026-06-08.
"""

import math


# ─── Hover thrust ─────────────────────────────────────────────────────
# Thrust value (0..1) where vertical velocity stabilizes at zero.
# Measured: tested at thrust=0.30, got accel_up=1.13 m/s² →
#   hover = g * T / (accel_up + g) = 9.81 * 0.30 / (1.13 + 9.81) = 0.269
HOVER_THRUST = 0.269


# ─── Attitude inner-loop response ─────────────────────────────────────
# Measured rise time was 1.3 ms (faster than our 50 Hz sampler — sim's
# attitude inner loop is essentially instantaneous to us). True value
# is < 20 ms. Use a conservative 50 ms for outer-loop bandwidth planning.
ATTITUDE_RISE_TIME_S = 0.05


# ─── Thrust → vertical acceleration ───────────────────────────────────
# Vertical acceleration (m/s²) per unit thrust above hover.
# From physics: accel_per_unit = g / hover = 9.81 / 0.269 = 36.5
VERTICAL_ACCEL_PER_UNIT_THRUST = 36.5


# ─── Tilt → horizontal acceleration ───────────────────────────────────
# Horizontal acceleration (m/s²) per radian of tilt at hover thrust.
# Physics: small-angle approximation gives g at hover thrust.
HORIZONTAL_ACCEL_PER_RAD_TILT = 9.81


# ─── Limits ───────────────────────────────────────────────────────────
MAX_TILT_RAD = math.radians(30)
THRUST_MIN = 0.05
THRUST_MAX = 0.85
