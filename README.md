# Line-Following Robot with PID Control

**Siemens Digital Twin Technologies Course — Spring 2026**  
**Author:** Abdalah Saad
**Presented to:** Dr. Mohamed Abdelsalam and Eng. Mohamed El-Leithy

---

## Project Overview

A digital twin of a differential-drive robot that steers onto a predefined
reference path and tracks it with minimal lateral error.

The system runs on the **Siemens Innexis Virtual System Interconnect (IVSI)**
framework as three independent Python clients connected over a virtual Ethernet
backplane (`python2DtEthernet` gateway).

```
┌──────────────────────┐  pose + ref (TCP 8070)  ┌──────────────────────┐
│  PlantSimulation.py  │ ─────────────────────►  │  controller.py       │
│  IP: 192.168.1.2     │ ◄─────────────────────  │  IP: 192.168.1.1     │
│  Ports: 8070, 8071   │  v_cmd, w_cmd            │  Ports: 8070, 8072   │
└──────────────────────┘                          └──────────────────────┘
          │                                                   │
          │  pose + ref (TCP 8071)          errors (TCP 8072) │
          ▼                                                   ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  visualizer.py  (IP: 192.168.1.3)                                        │
│  Logs all signals → computes KPIs → saves plots + CSV at end of run     │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Repository Structure

```
lineFollowDemo/
│
├── vsiBuildCommands                 ← IVSI topology (run with vsiBuild)
│
├── src/
│   ├── plant/
│   │   └── PlantSimulation.py      ← Client 1: kinematics + noise + path
│   ├── controller/
│   │   └── controller.py           ← Client 2: PID control law
│   └── visualizer/
│       └── visualizer.py           ← Client 3: logger + KPI + plots
│
└── shared/
    ├── path_builder.py             ← builds straight and sinusoidal paths
    ├── kinematics.py               ← unicycle step + noise injection
    └── ivsi_transport.py           ← pack/unpack doubles for IVSI gateway
```

---

## Architecture

### Client 1 — PlantSimulation

Builds the reference path, spawns the robot at a random offset, and runs
the kinematic update every timestep:

```
x_{k+1} = x_k + v·cos(θ_k)·Δt        (Eq. 1)
y_{k+1} = y_k + v·sin(θ_k)·Δt        (Eq. 2)
θ_{k+1} = θ_k + ω_k·Δt               (Eq. 3)
```

Sensor noise is injected before transmission:
```
x̃ = x + N(0, σ²)                     (Eq. 4)
ỹ = y + N(0, σ²)                     (Eq. 5)
θ̃ = θ + N(0, (σ/10)²)               (Eq. 6)
```

### Client 2 — Controller

Computes lateral error every timestep:
```
e_lat = y_robot − y_ref              (Eq. 7, straight path)
```

Runs the PID law:
```
u(k)  = Kp·e(k) + Ki·∫e dt + Kd·ė  (Eq. 8)
ω_cmd = −u(k)                        (Eq. 9)
```

### Client 3 — Visualizer

Logs all signals and computes three KPIs at the end of the simulation:

| KPI | Equation | Definition |
|-----|----------|-----------|
| OS  | Eq. 12   | `max |e_lat(t)|`                             |
| Ts  | Eq. 13   | First t where `|e| ≤ 0.05 m` holds forever  |
| SSE | Eq. 14   | `mean(|e|)` over the last 10 seconds         |

---

## Shared Modules

| Module | Purpose |
|--------|---------|
| `shared/path_builder.py`   | Builds straight and sinusoidal paths; finds closest waypoint |
| `shared/kinematics.py`     | Unicycle `step()` and `add_noise()` functions |
| `shared/ivsi_transport.py` | `pack_doubles()` / `unpack_doubles()` for IVSI byte payloads |

---

## Experiment Settings

### PlantSimulation.py

```python
PATH_TYPE  = 'straight'   # 'straight' (E1, E3)  |  'sinusoidal' (E2, E4)
SIGMA_POS  = 0.0          # noise: 0.0 | 0.02 | 0.05 | 0.08
PATH_FREQ  = 0.3          # sinusoidal frequency (rad/m): 0.3 gentle | 0.6 tight
```

### controller.py

```python
GAIN_SET   = 'T4'         # see gain table below
PATH_TYPE  = 'straight'   # must match PlantSimulation.py
```

**Gain table:**

| Key | Kp  | Ki  | Kd  | Notes |
|-----|-----|-----|-----|-------|
| T1  | 2.5 | 0.0 | 0.4 | High proportional, no integral |
| T2  | 0.8 | 0.0 | 0.2 | Low gains, slow response |
| T3  | 2.5 | 0.1 | 0.4 | Adds integral to T1 |
| T4  | 2.0 | 0.1 | 0.6 | Best overall balance |
| T5  | 1.6 | 0.1 | 0.4 | Sluggish convergence |
| PD  | 2.0 | 0.0 | 0.6 | E4 ablation — no integral |

### visualizer.py

```python
GAIN_SET    = 'T4'        # label used in filenames and CSV
PATH_TYPE   = 'straight'
NOISE_LEVEL = '0.0'       # label string
RESULTS_DIR = '...'       # where to save plots and CSV
```

---

## Prerequisites

- Ubuntu 22.04 on the Siemens Innexis VSI server
- Siemens Innexis VSI 2025.2
- SystemC 2.3.4
- Python 3.10+
- Valid SALT license (`VeloceStratoOS`)

```bash
pip3 install matplotlib --break-system-packages
```

---

## Environment Variables

Add to `~/.bashrc` and run `source ~/.bashrc`:

```bash
export LD_LIBRARY_PATH=/data/tools/systemc/systemc-2.3.4/lib/linux64_el30_gnu103:$LD_LIBRARY_PATH
export INNEXIS_VSI_HOME=/data/tools/pave/vsi_2025.2
export SALT_HOME=/data/tools/pave/vsi_2025.2/common/license/salt_2.4.2
export SALT_LICENSE_SERVER=29000@10.9.8.8
export SALT_PKGINFO_FILE=$SALT_HOME/salt_sdk/lnx64/bin/mgc.pkginfo
export PATH=/data/tools/pave/vsi_2025.2/bin:$PATH
```

---

## Running the Simulation

### Step 1 — Copy shared modules into the workspace

```bash
PROJ=/home/ubuntu/line_follower_dt/vsiBuild/lineFollowerTwin_MA

cp -r shared/  $PROJ/
```

### Step 2 — Copy client files

```bash
cp lineFollowDemo/src/plant/PlantSimulation.py     $PROJ/src/PlantSimulation/
cp lineFollowDemo/src/controller/controller.py     $PROJ/src/controller/
cp lineFollowDemo/src/visualizer/visualizer.py     $PROJ/src/visualizer/
```

### Step 3 — Launch (4 terminals)

**Terminal 1 — FabricServer:**
```bash
cd $PROJ
rm -f dconn-* vsiInputFifo vsiOutputFifo vsiInterruptFifo
mkfifo vsiInputFifo vsiOutputFifo vsiInterruptFifo
(while true; do grep -q "Got all clients" fab.log 2>/dev/null && echo "run" && break; sleep 0.2; done) > vsiInputFifo &
./FabricServer --domain=AF_UNIX --cmdFile=vsiInputFifo 2>&1 | tee fab.log
```

**Terminal 2 — Plant:**
```bash
cd $PROJ && python3 src/PlantSimulation/PlantSimulation.py --domain="AF_UNIX"
```

**Terminal 3 — Controller:**
```bash
cd $PROJ && python3 src/controller/controller.py --domain="AF_UNIX"
```

**Terminal 4 — Visualizer:**
```bash
cd $PROJ && python3 src/visualizer/visualizer.py --domain="AF_UNIX"
```

---

## Output

After the simulation completes, the visualizer saves to `RESULTS_DIR`:

```
results/
├── T4_straight_noise0.0_trajectory.png    ← robot path vs reference
├── T4_straight_noise0.0_lateral_error.png ← error over time
└── kpi_results.csv                        ← KPI table (appended per run)
```

> **Note:** No plots or CSV files are included in this repository.
> All outputs are generated only after running the simulation on the IVSI server.

---

## Experiments

| Exp | PATH_TYPE   | SIGMA_POS          | GAIN_SET       | Purpose              |
|-----|-------------|--------------------|----------------|----------------------|
| E1  | straight    | 0.0                | T1→T2→T3→T4→T5 | Find best gains      |
| E2  | sinusoidal  | 0.0 / 0.02 / 0.06  | T4             | Curved path test     |
| E3  | straight    | 0.0/0.02/0.05/0.08 | T4             | Noise robustness     |
| E4  | sinusoidal  | 0.02               | PD vs T4       | Integral ablation    |
