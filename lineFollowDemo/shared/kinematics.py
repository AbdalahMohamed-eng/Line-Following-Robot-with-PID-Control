"""
shared/kinematics.py
====================
Author  : Abdalah
Course  : Siemens Digital Twin Technologies — Spring 2026
Project : Line-Following Robot with PID Control

Purpose
-------
Implements the unicycle (differential-drive) kinematic model and
the Gaussian sensor noise injection used in the plant simulation.

The robot is modelled as a point mass with:
  - Constant linear speed  v  (only angular speed ω is controlled)
  - No wheel slip or mechanical lag

Mathematical model
------------------
Discrete unicycle kinematics — Equations 1-3 (report):

    x_{k+1}   = x_k   + v_k · cos(θ_k) · Δt
    y_{k+1}   = y_k   + v_k · sin(θ_k) · Δt
    θ_{k+1}   = θ_k   + ω_k · Δt

Heading is wrapped to [-π, π] after every update to prevent drift.

Sensor noise — Equations 4-6 (report):

    x̃  = x  + ε_x ,   ε_x ~ N(0, σ²_pos)
    ỹ  = y  + ε_y ,   ε_y ~ N(0, σ²_pos)
    θ̃  = θ  + ε_θ ,   ε_θ ~ N(0, (σ_pos / 10)²)

Usage
-----
    from shared.kinematics import step, add_noise

    x, y, theta = step(x, y, theta, v=0.5, omega=0.1, dt=0.01)
    nx, ny, nt  = add_noise(x, y, theta, sigma_pos=0.02)
"""

import math
import random


# ---------------------------------------------------------------------------
# Kinematic update
# ---------------------------------------------------------------------------

def step(x, y, theta, v, omega, dt):
    """
    Advance the robot pose by one discrete timestep.

    Parameters
    ----------
    x, y  : float  — current position (metres)
    theta : float  — current heading  (radians)
    v     : float  — linear velocity  (m/s)
    omega : float  — angular velocity (rad/s)
    dt    : float  — timestep         (seconds)

    Returns
    -------
    (new_x, new_y, new_theta) : tuple of float
        Updated pose. Heading is wrapped to [-π, π].
    """
    new_x     = x + v * math.cos(theta) * dt
    new_y     = y + v * math.sin(theta) * dt
    new_theta = theta + omega * dt

    # Keep heading inside [-π, π] — prevents numerical drift over long runs
    new_theta = _wrap_angle(new_theta)

    return new_x, new_y, new_theta


def _wrap_angle(angle):
    """
    Wrap an angle to the interval [-π, π].

    Uses atan2(sin, cos) — numerically stable for any magnitude.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


# ---------------------------------------------------------------------------
# Sensor noise
# ---------------------------------------------------------------------------

def add_noise(x, y, theta, sigma_pos):
    """
    Add additive Gaussian noise to the robot pose before it is
    transmitted to the controller (simulates real sensor imperfection).

    Parameters
    ----------
    x, y, theta : float
        True robot pose.
    sigma_pos : float
        Standard deviation of the position noise (metres).
        Set to 0.0 for noise-free operation.
        Typical values: 0.02 (low), 0.05 (moderate), 0.08 (high).

    Returns
    -------
    (noisy_x, noisy_y, noisy_theta) : tuple of float
        Corrupted pose that is sent to other clients.
    """
    if sigma_pos == 0.0:
        # Fast path: skip RNG when noise is disabled
        return x, y, theta

    sigma_theta = sigma_pos / 10.0   # heading noise is 10× smaller

    noisy_x     = x     + random.gauss(0.0, sigma_pos)
    noisy_y     = y     + random.gauss(0.0, sigma_pos)
    noisy_theta = theta + random.gauss(0.0, sigma_theta)

    return noisy_x, noisy_y, noisy_theta
