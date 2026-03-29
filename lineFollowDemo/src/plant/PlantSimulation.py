#!/usr/bin/env python3
"""
lineFollowDemo/src/plant/PlantSimulation.py
===========================================
Author  : Abdalah
Course  : Siemens Digital Twin Technologies — Spring 2026
Project : Line-Following Robot with PID Control

Client 1 — Plant Simulation
----------------------------
Simulates a differential-drive robot moving on a 2-D plane.

Responsibilities
~~~~~~~~~~~~~~~~
1. Build the reference path at startup (straight or sinusoidal).
2. Spawn the robot at a random offset from the path start.
3. Every timestep:
   a. Receive velocity commands (v_cmd, w_cmd) from the Controller.
   b. Update the true robot pose using unicycle kinematics.
   c. Inject Gaussian sensor noise into the pose.
   d. Find the closest waypoint on the reference path.
   e. Send the noisy pose AND the closest waypoint to both
      the Controller and the Visualizer.

Network connections (python2DtEthernet gateway)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  IP  : 192.168.1.2  |  MAC: 12:34:56:78:9A:BA
  Port 8070 — TCP server  →  Controller  (send pose+ref, recv commands)
  Port 8071 — TCP server  →  Visualizer  (send pose+ref only)

Signals sent (8 doubles × 8 bytes = 64 bytes per packet)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  pose_x, pose_y, theta           — noisy robot position and heading
  ref_x,  ref_y,  ref_theta       — closest path waypoint

Signals received from Controller (2 doubles = 16 bytes)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  v_cmd  — linear  velocity command  (m/s)
  w_cmd  — angular velocity command  (rad/s)

Experiment settings (edit before each run)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  PATH_TYPE   : 'straight' for E1/E3   |   'sinusoidal' for E2/E4
  SIGMA_POS   : 0.0 (none) | 0.02 (low) | 0.05 (moderate) | 0.08 (high)
  PATH_FREQ   : 0.3 rad/m  (gentle curve)  |  0.6 rad/m  (tight curve)
"""

from __future__ import print_function
import struct
import sys
import argparse
import random

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
from shared.path_builder  import build_path, closest_waypoint
from shared.kinematics    import step, add_noise
from shared.ivsi_transport import pack_doubles, unpack_doubles


# ===========================================================================
# EXPERIMENT SETTINGS  —  edit these between runs
# ===========================================================================
PATH_TYPE  = 'straight'    # 'straight' (E1, E3)  or  'sinusoidal' (E2, E4)
SIGMA_POS  = 0.0           # position noise σ (m): 0.0 / 0.02 / 0.05 / 0.08
PATH_FREQ  = 0.3           # sinusoidal frequency (rad/m): 0.3 gentle, 0.6 tight
V_INITIAL  = 0.5           # default linear speed before first command arrives

# Spawn: robot starts within ±0.5 m of the path's initial y-value
SPAWN_RANGE = 0.5

# Network
SRC_MAC        = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBA]
SRC_IP         = [192, 168, 1, 2]
PORT_CTRL      = 8070      # controller connects here
PORT_VIZ       = 8071      # visualizer connects here
COMPONENT_ID   = 0
VSI_PORT       = 50101


# ===========================================================================
# Helper functions
# ===========================================================================

def spawn_robot(waypoints):
    """
    Place the robot at x=0 with a random lateral offset from the first
    waypoint.  Heading starts aligned with the path tangent.

    Returns
    -------
    (x, y, theta) : tuple of float  — initial robot pose
    """
    first_x, first_y, first_theta = waypoints[0]
    offset = random.uniform(-SPAWN_RANGE, SPAWN_RANGE)
    return first_x, first_y + offset, first_theta


def decode_command(raw_bytes):
    """
    Unpack v_cmd and w_cmd from a 16-byte payload.

    Returns
    -------
    (v_cmd, w_cmd) : tuple of float
    """
    v_cmd, w_cmd = unpack_doubles(raw_bytes, count=2)
    return v_cmd, w_cmd


def encode_state(noisy_x, noisy_y, noisy_theta, ref_x, ref_y, ref_theta):
    """
    Pack the six state signals into a 48-byte payload for transmission.

    Returns
    -------
    bytes : packed payload
    """
    return pack_doubles(noisy_x, noisy_y, noisy_theta,
                        ref_x,   ref_y,   ref_theta)


def send_to_port(port_handle, payload):
    """Send a bytes payload through the IVSI Ethernet gateway."""
    vsi_eth.sendEthernetPacket(port_handle, payload)


def receive_from_port(port_handle):
    """
    Receive an Ethernet packet from the gateway.

    Returns
    -------
    bytes or None
        Raw payload bytes, or None if the packet was empty.
    """
    received = vsi_eth.recvEthernetPacket(port_handle)
    num_bytes = received[3]
    if num_bytes == 0:
        return None
    return bytes(received[2][:num_bytes])


# ===========================================================================
# PlantSimulation IVSI Client
# ===========================================================================

class PlantSimulation:
    """
    IVSI Client 1 — simulates the robot plant and environment.

    Attributes
    ----------
    waypoints       : list of (x, y, theta)  —  reference path
    robot_x/y/theta : float  —  true robot pose  (not transmitted)
    v_cmd, w_cmd    : float  —  latest velocity commands from controller
    port_ctrl       : int    —  IVSI handle to controller port
    port_viz        : int    —  IVSI handle to visualizer port
    """

    def __init__(self, args):
        self.component_id        = COMPONENT_ID
        self.local_host          = args.server_url
        self.domain              = args.domain
        self.vsi_port            = VSI_PORT
        self.simulation_step     = 0
        self.total_sim_time      = 0
        self.stop_requested      = False

        # Build path and spawn robot
        self.waypoints = build_path(PATH_TYPE, freq=PATH_FREQ)
        self.robot_x, self.robot_y, self.robot_theta = spawn_robot(self.waypoints)

        # Initial velocity commands (robot is stationary until controller connects)
        self.v_cmd = V_INITIAL
        self.w_cmd = 0.0

        # Port handles assigned after TCP connections are established
        self.port_ctrl = 0
        self.port_viz  = 0

        print(f"[Plant] PATH={PATH_TYPE}  SIGMA={SIGMA_POS}  FREQ={PATH_FREQ}")
        print(f"[Plant] Spawn: x={self.robot_x:.3f}  y={self.robot_y:.3f}  "
              f"θ={self.robot_theta:.3f}")

    # -----------------------------------------------------------------------
    def _connect_to_fabric(self):
        """Connect this client to the IVSI FabricServer."""
        session = vsi_api.connectToServer(
            self.local_host, self.domain,
            self.vsi_port, self.component_id
        )
        vsi_eth.initialize(session, self.component_id,
                           bytes(SRC_MAC), bytes(SRC_IP))

    def _open_tcp_ports(self):
        """Open TCP server sockets and wait for the other clients to connect."""
        print("[Plant] Waiting for Controller on port", PORT_CTRL)
        self.port_ctrl = vsi_eth.tcpListen(PORT_CTRL)

        print("[Plant] Waiting for Visualizer  on port", PORT_VIZ)
        self.port_viz  = vsi_eth.tcpListen(PORT_VIZ)

        if self.port_ctrl == 0 or self.port_viz == 0:
            raise RuntimeError("[Plant] Failed to open TCP ports — aborting.")
        print("[Plant] Both clients connected.")

    def _update_vsi_variables(self):
        """Refresh simulation time and stop flag from the FabricServer."""
        self.total_sim_time  = vsi_api.getTotalSimulationTime()
        self.simulation_step = vsi_api.getSimulationStep()
        self.stop_requested  = vsi_api.isStopRequested()

    # -----------------------------------------------------------------------
    def _simulation_step(self):
        """
        Execute one simulation timestep.

        Order of operations:
          1. Compute dt from the VSI simulation step (nanoseconds → seconds).
          2. Integrate kinematics with the most recent velocity commands.
          3. Add sensor noise to produce the observable pose.
          4. Look up the closest reference waypoint.
          5. Receive new commands from the Controller.
          6. Send the observable pose + reference to both clients.
        """
        dt = self.simulation_step * 1e-9   # nanoseconds → seconds

        # Step 1 — Update true robot pose (Equations 1-3)
        self.robot_x, self.robot_y, self.robot_theta = step(
            self.robot_x, self.robot_y, self.robot_theta,
            v=self.v_cmd, omega=self.w_cmd, dt=dt
        )

        # Step 2 — Inject sensor noise (Equations 4-6)
        nx, ny, nt = add_noise(
            self.robot_x, self.robot_y, self.robot_theta,
            sigma_pos=SIGMA_POS
        )

        # Step 3 — Find closest reference waypoint
        ref_x, ref_y, ref_theta = closest_waypoint(
            self.waypoints, self.robot_x, self.robot_y
        )

        # Step 4 — Receive velocity commands from Controller
        raw = receive_from_port(self.port_ctrl)
        if raw is not None:
            self.v_cmd, self.w_cmd = decode_command(raw)

        # Step 5 — Send state to Controller and Visualizer
        payload = encode_state(nx, ny, nt, ref_x, ref_y, ref_theta)
        send_to_port(self.port_ctrl, payload)
        send_to_port(self.port_viz,  payload)

    # -----------------------------------------------------------------------
    def main_thread(self):
        """
        Entry point — connects to the FabricServer and runs the sim loop.
        Called once; blocks until the simulation ends.
        """
        self._connect_to_fabric()

        try:
            vsi_api.waitForReset()
            self._update_vsi_variables()

            if vsi_api.isStopRequested():
                raise StopIteration

            self._open_tcp_ports()
            next_time = vsi_api.getSimulationTimeInNs()

            # ── Main simulation loop ──────────────────────────────────────
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

                remaining = self.total_sim_time - now
                if next_time > self.total_sim_time:
                    vsi_api.advanceSimulation(remaining)
                    break

                vsi_api.advanceSimulation(next_time - now)

            # Clean termination
            if vsi_api.getSimulationTimeInNs() < self.total_sim_time:
                vsi_eth.terminate()

        except StopIteration:
            print("[Plant] Stop signal received.")
            vsi_api.advanceSimulation(self.simulation_step + 1)

        except Exception as exc:
            print(f"[Plant] Error: {exc}")
            vsi_api.advanceSimulation(self.simulation_step + 1)

        print("[Plant] Simulation finished.")


# ===========================================================================
# Entry point
# ===========================================================================

def parse_args():
    """Parse command-line arguments for IVSI domain and server URL."""
    parser = argparse.ArgumentParser(description='Plant Simulation — IVSI Client 1')
    parser.add_argument('--domain',     default='AF_UNIX',
                        help='Socket domain (AF_UNIX or AF_INET)')
    parser.add_argument('--server-url', default='localhost',
                        help='FabricServer hostname')
    return parser.parse_args()


if __name__ == '__main__':
    plant = PlantSimulation(parse_args())
    plant.main_thread()
