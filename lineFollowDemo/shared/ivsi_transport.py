"""
shared/ivsi_transport.py
========================
Author  : Abdalah
Course  : Siemens Digital Twin Technologies — Spring 2026
Project : Line-Following Robot with PID Control

Purpose
-------
Thin wrapper around the IVSI python2DtEthernet gateway.

Provides two simple functions:
  - pack_doubles()   : convert Python floats → raw bytes for transmission
  - unpack_doubles() : convert raw bytes      → Python floats after reception

This isolates all struct-packing logic in one place.  If the gateway
API changes, only this file needs updating.

The IVSI gateway transmits signals as 64-bit IEEE-754 doubles (8 bytes each),
packed in native byte order using Python's struct module.

Usage
-----
    from shared.ivsi_transport import pack_doubles, unpack_doubles

    # Sending 3 doubles (pose_x, pose_y, theta)
    payload = pack_doubles(pose_x, pose_y, theta)

    # Receiving 3 doubles back
    pose_x, pose_y, theta = unpack_doubles(raw_bytes, count=3)
"""

import struct


# ---------------------------------------------------------------------------
# Format string for one 64-bit double in native byte order
# ---------------------------------------------------------------------------
_DOUBLE_FMT  = '=d'    # '=' = native byte order, 'd' = double (8 bytes)
_DOUBLE_SIZE = struct.calcsize(_DOUBLE_FMT)   # always 8


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def pack_doubles(*values):
    """
    Pack one or more Python floats into a bytes object for transmission.

    Each value is encoded as a 64-bit IEEE-754 double (8 bytes).

    Parameters
    ----------
    *values : float
        Any number of floating-point values to pack, in order.

    Returns
    -------
    bytes
        Raw byte string ready to pass to vsiEthernetPythonGateway.sendEthernetPacket().

    Example
    -------
        payload = pack_doubles(pose_x, pose_y, theta, ref_x, ref_y, ref_theta)
    """
    result = b''
    for v in values:
        result += struct.pack(_DOUBLE_FMT, float(v))
    return result


def unpack_doubles(raw_bytes, count):
    """
    Unpack 'count' doubles from the front of a bytes object.

    Parameters
    ----------
    raw_bytes : bytes
        Raw payload received from the gateway.
    count : int
        Number of doubles to extract.

    Returns
    -------
    tuple of float
        Extracted values in the same order they were packed.

    Raises
    ------
    ValueError
        If raw_bytes is shorter than count × 8 bytes.

    Example
    -------
        pose_x, pose_y, theta, ref_x, ref_y, ref_theta = \
            unpack_doubles(payload, count=6)
    """
    needed = count * _DOUBLE_SIZE
    if len(raw_bytes) < needed:
        raise ValueError(
            f"unpack_doubles: need {needed} bytes for {count} doubles, "
            f"but received only {len(raw_bytes)} bytes."
        )

    fmt    = f'={count}d'
    values = struct.unpack(fmt, raw_bytes[:needed])
    return values   # tuple of floats
