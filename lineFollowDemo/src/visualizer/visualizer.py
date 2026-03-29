#!/usr/bin/env python3
"""
lineFollowDemo/src/visualizer/visualizer.py
===========================================
Author  : Abdalah
Course  : Siemens Digital Twin Technologies — Spring 2026
Project : Line-Following Robot with PID Control

Client 3 — Visualizer / Logger
--------------------------------
Pure subscriber — receives all signals every timestep, logs them,
and at the end of the simulation:
  1. Prints KPI results to the terminal.
  2. Saves a trajectory PNG (robot path vs reference).
  3. Saves a lateral-error PNG (error over time with ±0.05 m band).
  4. Appends one row to a KPI CSV file.

No plots or files are created until the simulation actually finishes.

KPI definitions (Equations 12-14 — report)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  OS  = max |e_lat(t)|                          (Overshoot, metres)
  Ts  = first t where |e_lat| ≤ 0.05 m always  (Settling time, seconds)
  SSE = mean(|e_lat|) over last 10 s            (Steady-state error, metres)

Network connections (python2DtEthernet gateway)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  IP  : 192.168.1.3  |  MAC: 12:34:56:78:9A:BB
  Port 8071 — TCP client  ←  PlantSimulation  (recv pose + ref)
  Port 8072 — TCP client  ←  Controller       (recv commands + errors)

Signals received from Plant (6 doubles = 48 bytes)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  pose_x, pose_y, theta, ref_x, ref_y, ref_theta

Signals received from Controller (4 doubles = 32 bytes)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  v_cmd, w_cmd, lateral_error, heading_error

Output files (saved to RESULTS_DIR)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  trajectory.png       — robot path vs reference
  lateral_error.png    — error over time
  kpi_results.csv      — KPI table (appended per run)

Experiment label (edit to match Plant + Controller settings)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  GAIN_SET    : e.g. 'T4'
  PATH_TYPE   : 'straight' or 'sinusoidal'
  NOISE_LEVEL : label string, e.g. '0.02'
"""

from __future__ import print_function
import sys
import os
import math
import argparse
import csv

import matplotlib
matplotlib.use('Agg')          # non-interactive — no display required
import matplotlib.pyplot as plt

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
from shared.ivsi_transport import unpack_doubles


# ===========================================================================
# EXPERIMENT SETTINGS  —  edit these before each run
# ===========================================================================
GAIN_SET    = 'T4'          # Must match controller.py
PATH_TYPE   = 'straight'    # Must match PlantSimulation.py
NOISE_LEVEL = '0.0'         # Label only — used in filenames and CSV

# Where to save output files
RESULTS_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', '..', 'results'
)

# KPI thresholds
SETTLING_THRESHOLD  = 0.05    # ±5 cm band (Equation 13)
SSE_WINDOW_SECONDS  = 10.0    # Last 10 s used for SSE (Equation 14)

# Network
SRC_MAC      = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBB]
SRC_IP       = [192, 168, 1, 3]
PLANT_IP     = [192, 168, 1, 2]
CTRL_IP      = [192, 168, 1, 1]
PORT_PLANT   = 8071    # connect to Plant
PORT_CTRL    = 8072    # connect to Controller
COMPONENT_ID = 2
VSI_PORT     = 50103


# ===========================================================================
# KPI computation functions
# ===========================================================================

def compute_overshoot(lateral_errors):
    """
    Maximum absolute lateral error over the full run.
    Equation 12: OS = max |e_lat(t)|

    Returns float (metres).
    """
    return max(abs(e) for e in lateral_errors)


def compute_settling_time(times, lateral_errors, threshold=SETTLING_THRESHOLD):
    """
    First time instant from which ALL subsequent errors stay within ±threshold.
    Equation 13: Ts = min{t : |e(τ)| ≤ threshold  ∀τ ≥ t}

    Scans forward and uses the complement: find the last timestep where
    the error exceeds the threshold; settling time = next timestep after that.

    Returns float (seconds); equals total sim time if never settled.
    """
    last_violation = -1
    for i, e in enumerate(lateral_errors):
        if abs(e) > threshold:
            last_violation = i

    if last_violation == -1:
        return times[0]                  # settled from the very start
    if last_violation == len(times) - 1:
        return times[-1]                 # never settled

    return times[last_violation + 1]


def compute_sse(times, lateral_errors, window_seconds=SSE_WINDOW_SECONDS):
    """
    Mean absolute lateral error over the last `window_seconds` seconds.
    Equation 14: SSE = (1/N) Σ |e_i|  over last 10 s

    Returns float (metres).
    """
    if not times:
        return 0.0

    cutoff     = times[-1] - window_seconds
    window_err = [abs(e) for t, e in zip(times, lateral_errors) if t >= cutoff]
    return sum(window_err) / len(window_err) if window_err else 0.0


# ===========================================================================
# Plotting functions
# ===========================================================================

def save_trajectory_plot(x_robot, y_robot, x_ref, y_ref, filepath, label):
    """
    Save a trajectory-vs-reference plot to `filepath`.

    Parameters
    ----------
    x_robot, y_robot : list of float  — logged robot positions
    x_ref,   y_ref   : list of float  — logged reference waypoints
    filepath          : str            — output PNG path
    label             : str            — subtitle for the plot
    """
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(x_ref,   y_ref,   'b--', linewidth=1.5, label='Reference path', alpha=0.8)
    ax.plot(x_robot, y_robot, 'r-',  linewidth=1.2, label='Robot trajectory')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Trajectory vs Reference Path\n{label}')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(filepath, dpi=150)
    plt.close(fig)
    print(f"[Visualizer] Saved trajectory  → {filepath}")


def save_lateral_error_plot(times, errors, filepath, label,
                            threshold=SETTLING_THRESHOLD):
    """
    Save a lateral-error-over-time plot with ±threshold lines.

    Parameters
    ----------
    times   : list of float  — simulation time values (s)
    errors  : list of float  — lateral error at each timestep (m)
    filepath: str            — output PNG path
    label   : str            — subtitle
    threshold: float         — settling band (m)
    """
    fig, ax = plt.subplots(figsize=(11, 3.5))
    ax.plot(times, errors, 'g-', linewidth=0.8)
    ax.axhline( threshold, color='gray', linestyle='--',
                linewidth=0.8, alpha=0.7, label=f'±{threshold} m band')
    ax.axhline(-threshold, color='gray', linestyle='--',
                linewidth=0.8, alpha=0.7)
    ax.axhline(0,          color='black', linewidth=0.5, alpha=0.4)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Lateral error (m)')
    ax.set_title(f'Lateral Error over Time\n{label}')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(filepath, dpi=150)
    plt.close(fig)
    print(f"[Visualizer] Saved error plot  → {filepath}")


# ===========================================================================
# KPI reporting
# ===========================================================================

def print_kpi_summary(os_val, ts_val, sse_val, label):
    """Print a formatted KPI table to the terminal."""
    print()
    print("=" * 52)
    print(f"  KPI RESULTS  —  {label}")
    print("=" * 52)
    print(f"  Overshoot      (OS)  :  {os_val:.4f} m")
    print(f"  Settling time  (Ts)  :  {ts_val:.3f} s")
    print(f"  Steady-state   (SSE) :  {sse_val:.4f} m")
    print("=" * 52)
    print()


def append_kpi_csv(filepath, gain_set, path_type, noise,
                   os_val, ts_val, sse_val):
    """
    Append one row to the KPI CSV file.
    Creates the file with a header row if it does not yet exist.
    """
    header     = ['gain_set', 'path_type', 'noise_level',
                  'overshoot_m', 'settling_time_s', 'sse_m']
    write_hdr  = not os.path.exists(filepath)

    with open(filepath, 'a', newline='') as f:
        writer = csv.writer(f)
        if write_hdr:
            writer.writerow(header)
        writer.writerow([
            gain_set, path_type, noise,
            round(os_val,  4),
            round(ts_val,  3),
            round(sse_val, 4),
        ])
    print(f"[Visualizer] KPI appended      → {filepath}")


# ===========================================================================
# Visualizer IVSI Client
# ===========================================================================

class Visualizer:
    """
    IVSI Client 3 — logs all signals and saves results at end of simulation.

    Attributes
    ----------
    log_time          : list[float]  — simulation timestamps (s)
    log_pose_x/y      : list[float]  — robot trajectory
    log_ref_x/y       : list[float]  — reference path
    log_lateral_error : list[float]  — e_lat at each step
    log_heading_error : list[float]  — e_head at each step
    port_plant        : int          — IVSI handle to Plant
    port_ctrl         : int          — IVSI handle to Controller
    results_saved     : bool         — guard so saveResults runs once
    """

    def __init__(self, args):
        self.component_id    = COMPONENT_ID
        self.local_host      = args.server_url
        self.domain          = args.domain
        self.vsi_port        = VSI_PORT
        self.simulation_step = 0
        self.total_sim_time  = 0
        self.stop_requested  = False

        self.port_plant   = 0
        self.port_ctrl    = 0

        self._reset_buffers()
        os.makedirs(RESULTS_DIR, exist_ok=True)

        print(f"[Visualizer] GAIN={GAIN_SET}  PATH={PATH_TYPE}  NOISE={NOISE_LEVEL}")

    # -----------------------------------------------------------------------
    def _reset_buffers(self):
        """Clear all data buffers at the start of a new run."""
        self.log_time          = []
        self.log_pose_x        = []
        self.log_pose_y        = []
        self.log_ref_x         = []
        self.log_ref_y         = []
        self.log_lateral_error = []
        self.log_heading_error = []
        self.results_saved     = False

    def _connect_to_fabric(self):
        session = vsi_api.connectToServer(
            self.local_host, self.domain,
            self.vsi_port, self.component_id
        )
        vsi_eth.initialize(session, self.component_id,
                           bytes(SRC_MAC), bytes(SRC_IP))

    def _open_connections(self):
        """Connect to both Plant and Controller as a TCP client."""
        print("[Visualizer] Connecting to Plant on port", PORT_PLANT)
        self.port_plant = vsi_eth.tcpConnect(bytes(PLANT_IP), PORT_PLANT)

        print("[Visualizer] Connecting to Controller on port", PORT_CTRL)
        self.port_ctrl  = vsi_eth.tcpConnect(bytes(CTRL_IP),  PORT_CTRL)

        if self.port_plant == 0 or self.port_ctrl == 0:
            raise RuntimeError("[Visualizer] Failed to open connections.")
        print("[Visualizer] Connected.")

    def _update_vsi_variables(self):
        self.total_sim_time  = vsi_api.getTotalSimulationTime()
        self.simulation_step = vsi_api.getSimulationStep()
        self.stop_requested  = vsi_api.isStopRequested()

    # -----------------------------------------------------------------------
    def _log_timestep(self):
        """
        Receive all signals for this timestep and append to buffers.

        Signals received:
          From Plant      (48 bytes): pose_x, pose_y, theta, ref_x, ref_y, ref_theta
          From Controller (32 bytes): v_cmd, w_cmd, lateral_error, heading_error
        """
        t = vsi_api.getSimulationTimeInNs() * 1e-9   # nanoseconds → seconds

        # Receive from Plant
        plant_raw = _receive(self.port_plant)
        ctrl_raw  = _receive(self.port_ctrl)

        if plant_raw is None or ctrl_raw is None:
            return

        pose_x, pose_y, _, ref_x, ref_y, _ = unpack_doubles(plant_raw, count=6)
        _, _, e_lat, e_head                 = unpack_doubles(ctrl_raw,  count=4)

        # Append to logs
        self.log_time.append(t)
        self.log_pose_x.append(pose_x)
        self.log_pose_y.append(pose_y)
        self.log_ref_x.append(ref_x)
        self.log_ref_y.append(ref_y)
        self.log_lateral_error.append(e_lat)
        self.log_heading_error.append(e_head)

    def _save_results(self):
        """
        Compute KPIs and save all output files.
        Called exactly once at the end of the simulation.
        """
        if self.results_saved or not self.log_time:
            return
        self.results_saved = True

        # ── Compute KPIs ──────────────────────────────────────────────────
        os_val  = compute_overshoot(self.log_lateral_error)
        ts_val  = compute_settling_time(self.log_time, self.log_lateral_error)
        sse_val = compute_sse(self.log_time, self.log_lateral_error)

        label = (f"Gain={GAIN_SET}  Path={PATH_TYPE}  "
                 f"Noise σ={NOISE_LEVEL}")

        print_kpi_summary(os_val, ts_val, sse_val, label)

        # ── Save plots ────────────────────────────────────────────────────
        tag = f"{GAIN_SET}_{PATH_TYPE}_noise{NOISE_LEVEL}"

        save_trajectory_plot(
            self.log_pose_x, self.log_pose_y,
            self.log_ref_x,  self.log_ref_y,
            filepath=os.path.join(RESULTS_DIR, f'{tag}_trajectory.png'),
            label=label
        )

        save_lateral_error_plot(
            self.log_time, self.log_lateral_error,
            filepath=os.path.join(RESULTS_DIR, f'{tag}_lateral_error.png'),
            label=label
        )

        # ── Append to CSV ─────────────────────────────────────────────────
        append_kpi_csv(
            filepath=os.path.join(RESULTS_DIR, 'kpi_results.csv'),
            gain_set=GAIN_SET, path_type=PATH_TYPE, noise=NOISE_LEVEL,
            os_val=os_val, ts_val=ts_val, sse_val=sse_val
        )

    # -----------------------------------------------------------------------
    def main_thread(self):
        """
        Entry point — connects to the FabricServer and runs the logging loop.
        Saves results when the simulation ends.
        """
        self._connect_to_fabric()

        try:
            vsi_api.waitForReset()
            self._reset_buffers()
            self._update_vsi_variables()

            if vsi_api.isStopRequested():
                raise StopIteration

            self._open_connections()
            next_time = vsi_api.getSimulationTimeInNs()

            # ── Main logging loop ─────────────────────────────────────────
            while vsi_api.getSimulationTimeInNs() < self.total_sim_time:

                self._log_timestep()
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
            print("[Visualizer] Stop signal received.")
            vsi_api.advanceSimulation(self.simulation_step + 1)

        except Exception as exc:
            print(f"[Visualizer] Error: {exc}")
            vsi_api.advanceSimulation(self.simulation_step + 1)

        finally:
            # Always attempt to save — even if an error occurred mid-run
            self._save_results()

        print("[Visualizer] Done.")


# ===========================================================================
# Internal helpers
# ===========================================================================

def _receive(port_handle):
    """Receive a packet; return bytes or None if empty."""
    received  = vsi_eth.recvEthernetPacket(port_handle)
    num_bytes = received[3]
    if num_bytes == 0:
        return None
    return bytes(received[2][:num_bytes])


# ===========================================================================
# Entry point
# ===========================================================================

def parse_args():
    parser = argparse.ArgumentParser(description='Visualizer — IVSI Client 3')
    parser.add_argument('--domain',     default='AF_UNIX')
    parser.add_argument('--server-url', default='localhost')
    return parser.parse_args()


if __name__ == '__main__':
    viz = Visualizer(parse_args())
    viz.main_thread()
