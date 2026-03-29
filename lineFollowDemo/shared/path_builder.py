"""
shared/path_builder.py
======================
Author  : Abdalah
Course  : Siemens Digital Twin Technologies — Spring 2026
Project : Line-Following Robot with PID Control

Purpose
-------
Builds the reference paths the robot must follow.
Two path types are supported:
  - Straight : a horizontal line at a fixed y-reference
  - Sinusoidal: a smooth wave described by amplitude and spatial frequency

All path functions return a list of (x, y, heading_angle) waypoints.
The heading angle at each waypoint is the tangent direction of the path —
this is what the controller uses to compute heading error.

Usage
-----
    from shared.path_builder import build_path, closest_waypoint

    waypoints = build_path('straight')
    ref_x, ref_y, ref_theta = closest_waypoint(waypoints, robot_x, robot_y)
"""

import math


# ---------------------------------------------------------------------------
# Path configuration constants
# ---------------------------------------------------------------------------

STRAIGHT_Y_REF   = 2.0    # Fixed y-reference for the straight path (metres)
SINUSOIDAL_AMP   = 1.5    # Amplitude A of y_ref(x) = A·sin(f·x)  (metres)
SINUSOIDAL_FREQ  = 0.3    # Spatial frequency f  (rad/m) — 0.3 gentle, 0.6 tight

PATH_X_START     = 0.0    # Path starts at x = 0 m
PATH_X_END       = 35.0   # Path ends   at x = 35 m
PATH_STEP        = 0.01   # Distance between consecutive waypoints (m)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _linspace(start, stop, step):
    """
    Generate a list of x-values from start to stop with the given step.
    Equivalent to numpy.arange but with no external dependency.
    """
    values = []
    x = start
    while x <= stop + 1e-9:
        values.append(x)
        x += step
    return values


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def build_path(path_type, freq=SINUSOIDAL_FREQ):
    """
    Build and return a list of waypoints for the requested path type.

    Parameters
    ----------
    path_type : str
        'straight'   — horizontal line at y = STRAIGHT_Y_REF
        'sinusoidal' — smooth wave  y(x) = A · sin(f · x)

    freq : float, optional
        Spatial frequency for the sinusoidal path (rad/m).
        Ignored when path_type is 'straight'.

    Returns
    -------
    list of (x, y, theta)
        x     : float  — waypoint x-coordinate (m)
        y     : float  — waypoint y-coordinate (m)
        theta : float  — tangent heading angle at this waypoint (rad)

    Raises
    ------
    ValueError
        If path_type is not 'straight' or 'sinusoidal'.
    """
    x_values = _linspace(PATH_X_START, PATH_X_END, PATH_STEP)

    if path_type == 'straight':
        return _build_straight(x_values)

    elif path_type == 'sinusoidal':
        return _build_sinusoidal(x_values, amp=SINUSOIDAL_AMP, freq=freq)

    else:
        raise ValueError(
            f"Unknown path_type '{path_type}'. "
            "Choose 'straight' or 'sinusoidal'."
        )


def _build_straight(x_values):
    """
    Straight horizontal path at y = STRAIGHT_Y_REF.

    Equation 10 (report):
        y_ref(x) = 2.0 m   for all x
        theta_ref = 0       (pointing right)
    """
    waypoints = []
    for x in x_values:
        y     = STRAIGHT_Y_REF
        theta = 0.0           # heading is always horizontal
        waypoints.append((x, y, theta))
    return waypoints


def _build_sinusoidal(x_values, amp, freq):
    """
    Sinusoidal (wave) path.

    Equation 11 (report):
        y_ref(x)     = A · sin(f · x)
        theta_ref(x) = arctan(A · f · cos(f · x))

    The heading angle is derived from the path tangent:
        dy/dx = A · f · cos(f · x)
        theta = arctan(dy/dx)
    """
    waypoints = []
    for x in x_values:
        y     = amp * math.sin(freq * x)
        dydx  = amp * freq * math.cos(freq * x)   # slope of the curve
        theta = math.atan(dydx)                    # tangent heading
        waypoints.append((x, y, theta))
    return waypoints


def closest_waypoint(waypoints, robot_x, robot_y):
    """
    Find the waypoint on the path closest to the robot's current position.

    Uses Euclidean distance. Scans all waypoints so it handles
    non-monotone paths correctly (e.g. sinusoidal crossings).

    Parameters
    ----------
    waypoints : list of (x, y, theta)
        The path returned by build_path().
    robot_x, robot_y : float
        Robot's current position (m).

    Returns
    -------
    (ref_x, ref_y, ref_theta) : tuple of float
        The closest waypoint coordinates and heading.
    """
    best_dist_sq = float('inf')
    best_wp      = waypoints[0]

    for wp in waypoints:
        wx, wy, _ = wp
        dist_sq   = (robot_x - wx) ** 2 + (robot_y - wy) ** 2
        if dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            best_wp      = wp

    return best_wp   # returns (ref_x, ref_y, ref_theta)
