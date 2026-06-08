"""Measured drone parameters.

These come from running system_id.py against the live sim. Initial values
are rough guesses (clearly labeled). Replace with measured values once
system ID has been run.
"""

import math


# ─── Hover thrust ─────────────────────────────────────────────────────
# Thrust value (0..1) where vertical velocity stabilizes at zero.
# GUESS — replace with measurement from system_id.py
HOVER_THRUST = 0.50


# ─── Attitude inner-loop response ─────────────────────────────────────
# Approximate rise time of the sim's attitude controller when given an
# absolute attitude target. Used to choose outer-loop bandwidth.
# GUESS — replace with measurement from system_id.py
ATTITUDE_RISE_TIME_S = 0.20


# ─── Thrust → vertical acceleration ───────────────────────────────────
# Vertical acceleration (m/s²) per unit thrust above hover.
# A drone at 1.5× thrust-to-weight has ~5 m/s² spare at full thrust →
# this would be ~10. Will measure.
# GUESS
VERTICAL_ACCEL_PER_UNIT_THRUST = 10.0


# ─── Tilt → horizontal acceleration ───────────────────────────────────
# Horizontal acceleration (m/s²) per radian of tilt at hover thrust.
# At small angles, this is approximately g (9.81 m/s²) — physics, not
# tuning. Confirmed by measurement.
HORIZONTAL_ACCEL_PER_RAD_TILT = 9.81


# ─── Limits ───────────────────────────────────────────────────────────
MAX_TILT_RAD = math.radians(30)
THRUST_MIN = 0.20
THRUST_MAX = 0.95
