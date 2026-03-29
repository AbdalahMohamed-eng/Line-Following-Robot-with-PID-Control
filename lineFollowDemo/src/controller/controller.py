#!/usr/bin/env python3
"""
lineFollowDemo/src/controller/controller.py
===========================================
Author  : Abdalah
Course  : Siemens Digital Twin Technologies — Spring 2026
Project : Line-Following Robot with PID Control

Client 2 — PID Controller
--------------------------
Receives the robot's noisy pose and the closest reference waypoint
every timestep, computes a steering command using a PID control law,
and sends the velocity commands back to the Plant.

Control strategy
~~~~~~~~~~~~~~~~
Only the angular velocity ω is controlled — linear speed v is constant.

Lateral error (Equation 7):
    e_lat = y_robot − y_ref

    For a straight path this is a direct y-difference.
    For a curved path the perpendicular distance is computed via
    a cross-product with the path tangent direction.

PID control law (Equations 8-9):
    u(k)   = Kp·e(k)  +  Ki·∫e dt  +  Kd·ė(k)
    ω_cmd  = −u(k)

    The negative sign ensures correct steering:
      e > 0  (robot above path)  →  ω < 0  (steer down)
      e < 0  (robot below path)  →  ω > 0  (steer up)

Network connections (python2DtEthernet gateway)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  IP  : 192.168.1.1  |  MAC: 12:34:56:78:9A:BC
  Port 8070 — TCP client  ←  PlantSimulation  (recv pose+ref, send commands)
  Port 8072 — TCP server  →  Visualizer       (send commands + errors)

Signals received from Plant (6 doubles = 48 bytes)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  pose_x, pose_y, theta       — noisy robot pose
  ref_x,  ref_y,  ref_theta   — closest path waypoint

Signals sent to Plant (2 doubles = 16 bytes)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  v_cmd  — constant linear  speed  (m/s)
  w_cmd  — PID-computed angular speed (rad/s)

Signals sent to Visualizer (4 doubles = 32 bytes)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  v_cmd, w_cmd, lateral_error, heading_error

Experiment settings (edit before each run)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  GAIN_SET  : key into GAINS dict below
  PATH_TYPE : must match PlantSimulation.py

Gain table (Table 2 — report):
  'T1': Kp=2.5, Ki=0.0, Kd=0.4  — no integral, high proportional
  'T2': Kp=0.8, Ki=0.0, Kd=0.2  — low gains, slow response
  'T3': Kp=2.5, Ki=0.1, Kd=0.4  — integral added to T1
  'T4': Kp=2.0, Ki=0.1, Kd=0.6  — BEST overall (E1 winner)
  'T5': Kp=1.6, Ki=0.1, Kd=0.4  — sluggish, never fully settles
  'PD': Kp=2.0, Ki=0.0, Kd=0.6  — E4 ablation (no integral)
"""

from __future__ import print_function
import struct
import sys
import argparse
import math

# ---------------------------------------------------------------------------
# IVSI gateway imports
# ---------------------------------------------------------------------------
PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi      as vsi_api
import VsiTcpUdpPythonGateway  as vsi_eth

# ---------------------------------------------------------------------------
# Shared modules
# ---------------------------------------------------------------------------
sys.path.append('../../')
from shared.ivsi_transport import pack_doubles, unpack_doubles


# ===========================================================================
# EXPERIMENT SETTINGS  —  edit these between runs
# ===========================================================================
GAIN_SET   = 'T4'          # Key from GAINS table below
PATH_TYPE  = 'straight'    # 'straight' or 'sinusoidal' — must match Plant

# Linear speed is constant — PID only controls ω
V_CONST    = 0.5           # m/s

# Angular velocity saturation (prevents extreme steering commands)
OMEGA_MAX  = 1.5           # rad/s

# Reference y for straight path (used in lateral error computation)
Y_REF_STRAIGHT = 2.0

# Sinusoidal path parameters — must match PlantSimulation.py
CURVE_AMP  = 1.5
CURVE_FREQ = 0.3

# PID gain lookup table
GAINS = {
    'T1': {'Kp': 2.5, 'Ki': 0.0, 'Kd': 0.4},
    'T2': {'Kp': 0.8, 'Ki': 0.0, 'Kd': 0.2},
    'T3': {'Kp': 2.5, 'Ki': 0.1, 'Kd': 0.4},
    'T4': {'Kp': 2.0, 'Ki': 0.1, 'Kd': 0.6},   # best overall
    'T5': {'Kp': 1.6, 'Ki': 0.1, 'Kd': 0.4},
    'PD': {'Kp': 2.0, 'Ki': 0.0, 'Kd': 0.6},   # E4 — no integral
}

# Network
SRC_MAC      = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC]
SRC_IP       = [192, 168, 1, 1]
PLANT_IP     = [192, 168, 1, 2]
PORT_PLANT   = 8070    # connect to Plant as client
PORT_VIZ     = 8072    # listen for Visualizer as server
COMPONENT_ID = 1
VSI_PORT     = 50102


# ===========================================================================
# Helper functions
# ===========================================================================

def compute_lateral_error(pose_x, pose_y, ref_x, ref_y, ref_theta, path_type):
    """
    Compute the signed lateral error between the robot and the path.

    For a straight path (Equation 7):
        e_lat = y_robot − y_ref

    For a sinusoidal path the perpendicular (cross-product) distance is
    used so the error is correctly signed even when the path curves:
        e_lat = (p − r) × t_hat
    where p is the robot position, r is the reference waypoint, and
    t_hat = (cos θ_ref, sin θ_ref) is the unit tangent of the path.

    Parameters
    ----------
    pose_x, pose_y    : float  — noisy robot position
    ref_x, ref_y      : float  — closest waypoint position
    ref_theta         : float  — tangent heading at the waypoint
    path_type         : str    — 'straight' or 'sinusoidal'

    Returns
    -------
    float  — signed lateral error (metres); positive = robot above path
    """
    if path_type == 'straight':
        return pose_y - ref_y

    # Sinusoidal: perpendicular signed distance using 2-D cross product
    tx = math.cos(ref_theta)   # path tangent x-component
    ty = math.sin(ref_theta)   # path tangent y-component
    dx = pose_x - ref_x
    dy = pose_y - ref_y
    # Cross product magnitude (positive = robot is to the left of the path)
    return -tx * dy + ty * dx


def compute_heading_error(pose_theta, ref_theta):
    """
    Signed angle from the robot's heading to the path tangent.
    Wrapped to [-π, π] so it is always the shortest angular difference.

    Returns
    -------
    float  — heading error (radians)
    """
    diff = pose_theta - ref_theta
    return math.atan2(math.sin(diff), math.cos(diff))


def clamp(value, minimum, maximum):
    """Clamp value to [minimum, maximum]."""
    return max(minimum, min(maximum, value))


# ===========================================================================
# PID state  (persistent across timesteps inside the Controller object)
# ===========================================================================

class PIDState:
    """Holds the memory required by the discrete PID algorithm."""

    def __init__(self):
        self.integral   = 0.0   # accumulated error integral
        self.prev_error = 0.0   # error from the previous timestep

    def reset(self):
        """Reset memory at the start of a new simulation run."""
        self.integral   = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt, kp, ki, kd):
        """
        Compute the PID output for one timestep.

        Parameters
        ----------
        error     : float  — current lateral error e(k)
        dt        : float  — timestep (seconds)
        kp, ki, kd: float  — PID gains

        Returns
        -------
        float  — raw PID output u(k);  ω_cmd = −u(k)
        """
        self.integral += error * dt
        derivative     = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        return kp * error + ki * self.integral + kd * derivative


# ===========================================================================
# Controller IVSI Client
# ===========================================================================

class Controller:
    """
    IVSI Client 2 — computes PID steering commands every timestep.

    Attributes
    ----------
    gains       : dict   — {'Kp': float, 'Ki': float, 'Kd': float}
    pid         : PIDState
    port_plant  : int    — IVSI handle for Plant connection
    port_viz    : int    — IVSI handle for Visualizer connection
    """

    def __init__(self, args):
        self.component_id    = COMPONENT_ID
        self.local_host      = args.server_url
        self.domain          = args.domain
        self.vsi_port        = VSI_PORT
        self.simulation_step = 0
        self.total_sim_time  = 0
        self.stop_requested  = False

        self.gains = GAINS[GAIN_SET]
        self.pid   = PIDState()

        self.port_plant = 0
        self.port_viz   = 0

        print(f"[Controller] GAIN_SET={GAIN_SET}  "
              f"Kp={self.gains['Kp']}  Ki={self.gains['Ki']}  "
              f"Kd={self.gains['Kd']}  PATH={PATH_TYPE}")

    # -----------------------------------------------------------------------
    def _connect_to_fabric(self):
        session = vsi_api.connectToServer(
            self.local_host, self.domain,
            self.vsi_port, self.component_id
        )
        vsi_eth.initialize(session, self.component_id,
                           bytes(SRC_MAC), bytes(SRC_IP))

    def _open_connections(self):
        """Connect to Plant (as client) and open server port for Visualizer."""
        print("[Controller] Connecting to Plant on port", PORT_PLANT)
        self.port_plant = vsi_eth.tcpConnect(bytes(PLANT_IP), PORT_PLANT)

        print("[Controller] Waiting for Visualizer on port", PORT_VIZ)
        self.port_viz = vsi_eth.tcpListen(PORT_VIZ)

        if self.port_plant == 0 or self.port_viz == 0:
            raise RuntimeError("[Controller] Failed to open connections.")
        print("[Controller] All connections established.")

    def _update_vsi_variables(self):
        self.total_sim_time  = vsi_api.getTotalSimulationTime()
        self.simulation_step = vsi_api.getSimulationStep()
        self.stop_requested  = vsi_api.isStopRequested()

    # -----------------------------------------------------------------------
    def _simulation_step(self):
        """
        Execute one control timestep.

        Order:
          1. Receive pose + reference from Plant.
          2. Compute lateral and heading errors.
          3. Run PID → compute ω_cmd.
          4. Send v_cmd + w_cmd to Plant.
          5. Send v_cmd + w_cmd + errors to Visualizer.
        """
        dt = self.simulation_step * 1e-9

        # Step 1 — Receive state from Plant (6 doubles = 48 bytes)
        plant_raw = _receive(self.port_plant)
        if plant_raw is None:
            return

        pose_x, pose_y, pose_theta, ref_x, ref_y, ref_theta = \
            unpack_doubles(plant_raw, count=6)

        # Step 2 — Compute errors
        e_lat  = compute_lateral_error(
            pose_x, pose_y, ref_x, ref_y, ref_theta, PATH_TYPE)
        e_head = compute_heading_error(pose_theta, ref_theta)

        # Step 3 — PID → ω_cmd  (Equations 8-9)
        u      = self.pid.compute(e_lat, dt,
                                  self.gains['Kp'],
                                  self.gains['Ki'],
                                  self.gains['Kd'])
        w_cmd  = clamp(-u, -OMEGA_MAX, OMEGA_MAX)
        v_cmd  = V_CONST

        # Step 4 — Send commands to Plant
        _send(self.port_plant, pack_doubles(v_cmd, w_cmd))

        # Step 5 — Send commands + errors to Visualizer
        viz_raw = _receive(self.port_viz)   # drain any incoming (viz sends nothing)
        _send(self.port_viz, pack_doubles(v_cmd, w_cmd, e_lat, e_head))

    # -----------------------------------------------------------------------
    def main_thread(self):
        """
        Entry point — connects to the FabricServer and runs the control loop.
        """
        self._connect_to_fabric()

        try:
            vsi_api.waitForReset()
            self.pid.reset()
            self._update_vsi_variables()

            if vsi_api.isStopRequested():
                raise StopIteration

            self._open_connections()
            next_time = vsi_api.getSimulationTimeInNs()

            # ── Main control loop ─────────────────────────────────────────
            while vsi_api.getSimulationTimeInNs() < self.total_sim_time:

                self._simulation_step()
                self._update_vsi_variables()

                if vsi_api.isStopRequested():
                    raise StopIteration

                if vsi_eth.isTerminationOnGoing() or vsi_eth.isTerminated():
                    break

                next_time += self.simulation_step
                now        = vsi_api.getSimulationTimeInNs()

                if now >= next_time:
                    continue

                if next_time > self.total_sim_time:
                    vsi_api.advanceSimulation(self.total_sim_time - now)
                    break

                vsi_api.advanceSimulation(next_time - now)

            if vsi_api.getSimulationTimeInNs() < self.total_sim_time:
                vsi_eth.terminate()

        except StopIteration:
            print("[Controller] Stop signal received.")
            vsi_api.advanceSimulation(self.simulation_step + 1)

        except Exception as exc:
            print(f"[Controller] Error: {exc}")
            vsi_api.advanceSimulation(self.simulation_step + 1)

        print("[Controller] Done.")


# ===========================================================================
# Internal helpers (module-level, not class methods)
# ===========================================================================

def _send(port_handle, payload):
    """Send a bytes payload through the IVSI Ethernet gateway."""
    vsi_eth.sendEthernetPacket(port_handle, payload)


def _receive(port_handle):
    """
    Receive an Ethernet packet from the gateway.

    Returns bytes or None if the packet was empty.
    """
    received  = vsi_eth.recvEthernetPacket(port_handle)
    num_bytes = received[3]
    if num_bytes == 0:
        return None
    return bytes(received[2][:num_bytes])


# ===========================================================================
# Entry point
# ===========================================================================

def parse_args():
    parser = argparse.ArgumentParser(description='PID Controller — IVSI Client 2')
    parser.add_argument('--domain',     default='AF_UNIX')
    parser.add_argument('--server-url', default='localhost')
    return parser.parse_args()


if __name__ == '__main__':
    ctrl = Controller(parse_args())
    ctrl.main_thread()
