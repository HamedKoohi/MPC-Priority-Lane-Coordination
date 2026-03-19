# 🚗 Scalable Cooperative Lane-Change Management for Connected Autonomous Vehicles

<div align="center">

[![Paper](https://img.shields.io/badge/Paper-Scientific%20Reports-blue?style=for-the-badge&logo=read-the-docs)](https://doi.org/)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2020b-orange?style=for-the-badge&logo=mathworks)](https://www.mathworks.com/)
[![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)](LICENSE)
[![DAAD](https://img.shields.io/badge/Funded%20by-DAAD%2057610577-yellow?style=for-the-badge)](https://www.daad.de/)

**MPC-Based Decision Coordination for Multi-Vehicle Interactive Lane Changing**

*Hamed Kouhi¹ · Georg Schildbach²*

¹ University of Guilan, Iran &nbsp;|&nbsp; ² University of Lübeck, Germany

</div>

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Key Contributions](#-key-contributions)
- [System Architecture](#-system-architecture)
- [Methodology](#-methodology)
  - [Lane-Change Dynamics (Sigmoid Model)](#lane-change-dynamics-sigmoid-model)
  - [Priority-Based Search Strategy](#priority-based-search-strategy)
  - [MPC Cost Function](#mpc-cost-function)
  - [APF Collision Avoidance](#apf-collision-avoidance)
- [Algorithm Pipeline](#-algorithm-pipeline)
- [Simulation Results](#-simulation-results)
- [Runtime Analysis](#-runtime-analysis)
- [Comparison with Prior Work](#-comparison-with-prior-work)
- [Parameters](#-parameters)
- [Getting Started](#-getting-started)
- [Citation](#-citation)
- [Authors](#-authors)
- [Funding](#-funding)

---

## 🔍 Overview

This repository accompanies the paper:

> **"Scalable Cooperative Lane-Change Management for Connected Autonomous Vehicles Using MPC-Based Decision Coordination"**  
> *Hamed Kouhi, Georg Schildbach — Scientific Reports, 2026*

Connected autonomous vehicles (CAVs) can cooperate via vehicle-to-infrastructure (V2I) communication to achieve traffic-level efficiency that no single vehicle can attain alone. Most existing work addresses lane changes for a **single ego vehicle**. This paper fills the gap by presenting a **centralized framework** that jointly optimizes lane assignments and speed profiles for **all vehicles simultaneously** on a multi-lane highway.

### The Core Problem

Jointly optimizing lane changes for *n* vehicles over a finite horizon is a **mixed-integer optimization problem** whose complexity grows **exponentially** with the number of vehicles — making brute-force global search impossible in real time.

### Our Solution

A **priority-aware search strategy** embedded inside an MPC loop:

- At each time step, score every vehicle using a composite cost function
- Restrict the search to the **top-3 priority vehicles** only
- Evaluate only `2^3 = 8` lane-change combinations instead of the full combinatorial space
- Guarantee collision safety through **Artificial Potential Field (APF)** penalties
- Execute the full loop in **< 2.1 s** for up to 15 vehicles

---

## 🏆 Key Contributions

| Contribution | Description |
|---|---|
| **Multi-vehicle coordination** | First centralized framework to jointly optimize interactive lane changes for *all* vehicles on a highway segment, not just a single ego vehicle |
| **Priority-based search** | Reduces exponential mixed-integer complexity to a fixed 8-combination search, enabling real-time feasibility |
| **Finite-duration lane model** | Replaces the common instantaneous lane-change assumption with a sigmoid-based lateral trajectory that respects acceleration and jerk limits |
| **Deadlock prevention** | Cost function penalizes vehicles lingering in the left lane, preventing deadlocks during simultaneous multi-vehicle maneuvers |
| **Scalable architecture** | Total runtime scales as `O(n_set_L^3)` rather than exponentially with vehicle count |

---

## 🏗 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     V2I Communication                        │
│            (positions, velocities, lane states)              │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                 CENTRALIZED PLANNER                          │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  Stage A: Generate Lane Option Sets (Algorithm 1)    │   │
│  │  → SetL with nset,L possibilities per vehicle        │   │
│  └──────────────────────┬───────────────────────────────┘   │
│                         │                                    │
│  ┌──────────────────────▼───────────────────────────────┐   │
│  │  Stage B: Priority Ranking (Algorithm 2)             │   │
│  │  → Score each vehicle with j1, j2, j3               │   │
│  │  → Select top-3 candidates: O1, O2, O3              │   │
│  └──────────────────────┬───────────────────────────────┘   │
│                         │                                    │
│  ┌──────────────────────▼───────────────────────────────┐   │
│  │  Stage C: Cost Function J (Algorithm 3)              │   │
│  │  → Speed tracking + APF collision + deadlock penalty │   │
│  └──────────────────────┬───────────────────────────────┘   │
│                         │                                    │
│  ┌──────────────────────▼───────────────────────────────┐   │
│  │  Stage D: MPC Optimization (Algorithm 4)             │   │
│  │  → Solve over 8 lane combinations                    │   │
│  │  → Output: U_out (acceleration) + L_vec (new lanes)  │   │
│  └──────────────────────┬───────────────────────────────┘   │
└──────────────────────────┬──────────────────────────────────┘
                           │ Commands broadcast via V2I
           ┌───────────────┼───────────────┐
           ▼               ▼               ▼
        Vehicle 1       Vehicle 2  ...  Vehicle n
     (accel/brake)   (accel/brake)   (accel/brake)
     (lane change?)  (lane change?)  (lane change?)
```

---

## 📐 Methodology

### Lane-Change Dynamics (Sigmoid Model)

Rather than assuming lane changes happen instantaneously — a common simplification in high-level planning — this work models the lateral maneuver as a finite-duration trajectory governed by a **sigmoid function**. The sigmoid is chosen because it naturally produces a smooth S-shaped path that starts and ends with zero slope, eliminating any abrupt steering inputs at the moment the maneuver begins or completes.

The sharpness of the sigmoid — how aggressively the vehicle cuts across lanes — is controlled by a single parameter `k_sig`. This parameter is not freely chosen: it is automatically bounded from above by two physical limits derived from the vehicle dynamics. The first limit comes from the maximum allowable lateral acceleration the passengers can tolerate; the second from the maximum allowable lateral jerk (rate of change of acceleration). The tighter of the two constraints is enforced, ensuring that every generated trajectory is both comfortable and dynamically feasible regardless of lane width or vehicle speed.

**Key properties of the sigmoid lane-change model:**
- ✅ Smooth, continuous transition between lanes with no abrupt steering
- ✅ Zero slope at the start and end of the maneuver
- ✅ Continuous velocity and acceleration profiles throughout
- ✅ Aggressiveness is tunable and automatically bounded by physical limits
- ✅ Satisfies both lateral acceleration and lateral jerk constraints

---

### Priority-Based Search Strategy

At each time step the centralized planner must decide which vehicles should change lanes. Because the total number of lane-change combinations grows exponentially with the number of vehicles, solving the full problem in real time is infeasible. The priority strategy reduces this to a tractable subproblem in two steps.

**Step 1 — Score every vehicle.** Each vehicle receives a scalar priority score built from three weighted components:

- **Speed deficit (`j1`):** How far the vehicle's current speed is from its desired speed. A large gap means the vehicle is being blocked and urgently needs to move to a faster or less crowded lane.
- **Current-lane crowding (`j2`):** How tightly packed the surrounding vehicles are in the vehicle's present lane, measured using the front and rear gaps normalized by the required safe distance. A high score indicates a congested lane that the vehicle should leave.
- **Target-lane availability (`j3`):** How much clear space exists in the adjacent lane, again measured by normalized front and rear gaps. A high score indicates a safe and open lane to move into.

A safety override is applied last: if the adjacent lane does not provide a sufficiently safe gap, the vehicle is marked as ineligible for lane changing regardless of its other scores.

**Step 2 — Select the top three.** Vehicles are ranked by their priority scores in descending order. Only the three highest-priority vehicles — call them `O1`, `O2`, and `O3` — are considered for lane-change optimization at the current time step. All other vehicles maintain their current lane. This reduces the search space from an exponentially large set to at most `2^3 = 8` candidate lane combinations, which can be evaluated exhaustively in real time.

The weights balancing the three score components (`alpha2 = alpha3 = 1`) were determined through systematic simulation experiments, ensuring that neither congestion response nor safety assessment dominates the other.

---

### MPC Cost Function

The MPC optimizer selects acceleration and braking commands for all vehicles by minimizing a composite cost function over a finite prediction horizon. The cost function has five distinct terms, each targeting a specific behavior:

**Speed tracking (`J1`):** Penalizes the squared difference between each vehicle's actual speed and its desired speed, plus a small regularization on the control effort. This is the primary objective — all vehicles should converge to their assigned target speeds.

**Deadlock prevention (`J_antiLock`):** During simultaneous lane changes, vehicles in the left lane can become trapped if their desired speed is lower than their current speed and there is no room to decelerate without collision. This term detects such configurations and applies a penalty that encourages the slower vehicle to yield space, breaking the deadlock before it forms.

**Collision avoidance (`J_col`):** An Artificial Potential Field (APF) penalty that rises steeply as any two vehicles approach within the safe following distance. Described in detail in the next section.

**Lane-change suppression (`J_lane`):** Penalizes deviations of the lane vector from its previous value, discouraging unnecessary or oscillatory lane changes that do not improve traffic flow.

**Jerk minimization (`J_jerk`):** Penalizes large changes in control input between consecutive time steps, directly limiting the longitudinal jerk experienced by passengers and reducing actuator wear.

All five terms are summed with individually tuned weights, giving the designer direct control over the trade-off between speed convergence, comfort, safety margin, and lane stability.

---

### APF Collision Avoidance

The APF collision penalty is evaluated for every pair of vehicles that share a lane or are in the process of merging into the same lane. The key design requirement is that the penalty must be **smooth and differentiable** everywhere so that gradient-based MPC optimizers can handle it reliably — classical APF formulations that use hard walls or discontinuous potentials cause the optimizer to get stuck in local minima.

The approach works as follows. For any two vehicles A (behind) and B (ahead), the safe following distance is first made **speed-dependent**: faster closing speeds require a larger buffer. When the actual gap falls within the danger zone defined by this dynamic safe distance, a smooth repulsive cost is activated. The cost is designed as a rational function of the normalized gap that is bounded between zero and one, rises steeply as the gap approaches zero, and remains differentiable throughout. If the vehicles are already closer than the base safe distance — indicating an imminent collision risk — the penalty is amplified by a large constant factor to make collision avoidance the optimizer's absolute top priority at that step.

This formulation gives the MPC the information it needs to steer away from collisions predictively, over the full planning horizon, rather than reactively at the moment of danger.

---

## 🔄 Algorithm Pipeline

```
REPEAT at each MPC time step t:

  INPUT: Current state X (positions, velocities), Lane vector L_vec

  ┌─ Algorithm 1 ──────────────────────────────────┐
  │  Generate SetL: all future lane combinations   │
  │  for the prediction horizon (nset,L per veh)   │
  └────────────────────────────────────────────────┘

  ┌─ Algorithm 2 ──────────────────────────────────┐
  │  FOR each vehicle i_veh:                       │
  │    Compute j₁, j₂ (Alg 2-1), j₃ (Alg 2-2)   │
  │    J_vec(i_veh) = j₁ + α₂j₂ + α₃j₃           │
  │    Safety check → J_vec = −1 if unsafe         │
  │  Sort J_vec descending                         │
  │  Return {O₁, O₂, O₃}  ← top priority vehicles │
  └────────────────────────────────────────────────┘

  ┌─ Algorithm 4 (Main Loop) ──────────────────────┐
  │  FOR each of nset,L³ lane combinations:        │
  │    Assign tentative lanes to O₁, O₂, O₃       │
  │    Check collision feasibility                 │
  │    IF feasible:                                │
  │      Solve MPC: min J s.t. u_min ≤ u ≤ u_max │
  │      (Alg 3: J₁ + J_antiLock + J_col + …)     │
  │    ELSE: F_val ← 10¹⁵  (infeasible)           │
  │  [J_opt, ind_opt] = min(F_val)                 │
  │  Update L_vec, apply U_out to all vehicles     │
  └────────────────────────────────────────────────┘

  OUTPUT: U_out (accelerations for all n vehicles)
          L_vec (updated lane assignments)
```

---

## 📊 Simulation Results

All simulations run in **MATLAB R2020b** with 6 vehicles on a two-lane one-way highway.

### Scenario Setup

| Vehicle | Initial Speed | Desired Speed | Initial Lane |
|---------|--------------|---------------|--------------|
| 1 | 100 km/h | 80 km/h | Left |
| 2 | 80 km/h | 80 km/h | Left |
| 3 | 80 km/h | 75 km/h | Left |
| 4 | 70 km/h | 75 km/h | Right |
| 5 | 80 km/h | 90 km/h | Left |
| 6 | 70 km/h | 95 km/h | Right |

### Key Outcomes

- ✅ All 6 vehicles converge asymptotically to their desired speeds within 25 s
- ✅ Zero collisions throughout all maneuvers
- ✅ Smooth control inputs — jerk within acceptable limits
- ✅ Interactive, simultaneous lane changes executed without deadlock
- ✅ Left-lane bias resolved by the `J_antiLock` penalty term

> **Speed tracking performance:** actual speeds reach desired values [80, 80, 75, 75, 90, 95] km/h with smooth deceleration and acceleration profiles, respecting bounds `u_min = −20 N` and `u_max = 5 N`.

---

## ⏱ Runtime Analysis

Total runtime scales as:

```
Total Runtime = (n_set_L)^3 * (MPC Runtime for longitudinal control)
              + (Priority Logic Runtime)
```

With `n_set_L = 2` → only **8 MPC evaluations** per time step, regardless of total vehicle count.

### Measured Runtimes

| Case | N_c | Vehicles | Total Runtime (s) | MPC-only (s) | Ratio |
|------|-----|----------|-------------------|--------------|-------|
| 1 | 1 | 6 | 0.60 | 0.070 | ~8.6× |
| 2 | 2 | 6 | 0.90 | 0.110 | ~8.2× |
| 3 | 3 | 6 | 1.30 | 0.155 | ~8.4× |
| 4 | 4 | 6 | 1.70 | 0.200 | ~8.5× |
| 5 | 1 | 9 | 0.90 | 0.110 | ~8.2× |
| 6 | 1 | 12 | 1.72 | 0.210 | ~8.2× |
| 7 | 1 | 15 | 2.10 | 0.250 | ~8.4× |

The consistent ~8x ratio between total and MPC-only runtime confirms the `(n_set_L)^3 = 8` scaling, as opposed to the **exponential** growth of full mixed-integer optimization.

---

## 📊 Comparison with Prior Work

| Method | Architecture | Multi-Vehicle Interaction | Computational Strategy | Scalability | Real-Time |
|--------|-------------|--------------------------|----------------------|-------------|-----------|
| Chae & Yi [36] | Centralized | Limited (ego only) | Deterministic optimization | Moderate | ✅ |
| Zuo [37] | Centralized MPC | None | MPC optimization | Limited | ⚠️ |
| Dixit et al. [38] | Centralized MPC | Limited | Robust MPC | Limited | ⚠️ |
| Xie et al. [32] | Distributed APF | Yes | Local potential fields | High | ✅ |
| Yu et al. [33] | Distributed RL | Yes | Reinforcement learning | High | ⚠️ |
| Chen et al. [27] | Distributed RL | Yes | Multi-agent RL | High | ❌ |
| **Ours** | **Centralized MPC + Priority** | **Explicit global** | **Priority screening + MPC + APF** | **High** | ✅ |

**Unique positioning:** retains the **global view** of a centralized controller while achieving the **low computational burden** typically associated with decentralized or rule-based methods.

---

## ⚙️ Parameters

### Simulation Parameters

| Symbol | Value | Description |
|--------|-------|-------------|
| `T_f` | 25 s | Simulation time |
| `dT` | 0.1 s | Time step |
| `T_pred` | 5 s | MPC prediction horizon |
| `N_C` | 1 | Control horizon |
| `t_span` | 5 s | Control input update time |
| `d` | 3 m | Vehicle length |
| `m` | 1 kg | Vehicle mass (normalized) |
| `x_safe0` | 5 m | Minimum safe following distance |
| `u_min` | -20 N | Maximum braking force |
| `u_max` | 5 N | Maximum acceleration force |

### Controller Weights

| Symbol | Value | Role |
|--------|-------|------|
| `alpha2, alpha3` | 1 | Lane density / target-lane safety balance |
| `w1` | 0.1 | Speed tracking |
| `w3` | 500 | APF collision repulsion |
| `w4` | 1000 | APF collision base |
| `w5` | 20 | Unnecessary lane-change penalty |
| `w6` | 200 | Jerk smoothness |
| `W_antiLock` | 1 | Deadlock prevention |
| `c_col` | 1000 | Near-collision amplification |
| `w_xsafe` | 0.25 | Safe distance speed-dependent scaling |
| `N` | 2 | APF exponent |
| `N2` | 12 | Density exponent |
| `eps1` | 0.001 | APF numerical regularization |

---

## 🚀 Getting Started

### Prerequisites

- MATLAB R2020b or later
- Optimization Toolbox (for `fmincon` or equivalent)
- No external toolboxes required

### Repository Structure

```
├── algorithms/
│   ├── Algorithm1_SetL.m           # Stage A: Generate lane option sets
│   ├── Algorithm2_Priority.m       # Stage B: Priority vehicle selection
│   ├── Algorithm2_1_j2.m           # Calculating current-lane density cost j2
│   ├── Algorithm2_2_j3.m           # Calculating target-lane gap cost j3
│   ├── Algorithm2_3_IndVLC.m       # Tracking vehicles during lane change
│   ├── Algorithm3_Cost.m           # Stage C: MPC cost function J
│   ├── Algorithm3_1_antiLock.m     # Deadlock prevention cost
│   ├── Algorithm3_2_Jcol.m         # APF collision avoidance cost
│   └── Algorithm4_Main.m           # Stage D: Full MPC optimization loop
├── dynamics/
│   ├── sigmoid_lateral.m           # Sigmoid-based lateral trajectory model
│   └── state_space.m               # Longitudinal vehicle dynamics
├── simulation/
│   ├── main_simulation.m           # 6-vehicle CACC scenario entry point
│   └── params.m                    # All simulation and controller parameters
├── results/
│   ├── plot_speeds.m               # Speed convergence plots
│   ├── plot_lanes.m                # Spatial lane-change visualization
│   └── plot_runtime.m              # Runtime analysis plots
└── README.md
```

### Running the Simulation

```matlab
% Clone and navigate to the repository
cd cooperative_lane_change/

% Load parameters
run('simulation/params.m')

% Run the 6-vehicle scenario
run('simulation/main_simulation.m')

% Results are automatically plotted (Figs. 3–9 from the paper)
```

### Customizing the Scenario

```matlab
% In params.m — modify these to test different configurations:
ncar   = 6;                              % Number of vehicles
V0     = [100,80,80,70,80,70];           % Initial speeds (km/h)
Vdes   = [80,80,75,75,90,95];            % Desired speeds (km/h)
P0     = [5,10,25,15,0,-5];             % Initial positions (m)
Lvec   = [1,1,1,0,1,0];                 % Initial lanes (0=left, 1=right)
Nc     = 1;                             % Control horizon steps
nset_L = 2;                             % Lane options per vehicle (nset,L)
```

---

## 📄 Citation

If you use this code or methodology in your research, please cite:

```bibtex
@article{kouhi2026cooperative,
  title   = {Scalable Cooperative Lane-Change Management for Connected
             Autonomous Vehicles Using MPC-Based Decision Coordination},
  author  = {Kouhi, Hamed and Schildbach, Georg},
  journal = {Scientific Reports},
  year    = {2026},
  doi     = {10.xxxx/xxxxxxx}
}
```

---

## 👥 Authors

<table>
  <tr>
    <td align="center">
      <b>Hamed Kouhi</b><br>
      Assistant Professor<br>
      Faculty of Mechanical Engineering<br>
      University of Guilan, Rasht, Iran<br>
      <a href="mailto:hamed.koohi@guilan.ac.ir">hamed.koohi@guilan.ac.ir</a>
    </td>
    <td align="center">
      <b>Georg Schildbach</b><br>
      Full Professor of Mechatronics<br>
      Autonomous Systems Laboratory<br>
      University of Lübeck, Germany<br>
    </td>
  </tr>
</table>

---

## 💰 Funding

This work was supported by the **German Academic Exchange Service (DAAD)** under Grant No. **57610577**.

---

## 📚 Related Work

Key references that this work builds upon or contrasts with:

- **[32]** Xie et al., "Distributed motion planning for safe autonomous vehicle overtaking via APF," *IEEE T-ITS*, 2022
- **[36]** Chae & Yi, "Virtual target-based overtaking decision, motion planning, and control," *IEEE Access*, 2020
- **[38]** Dixit et al., "Trajectory planning for autonomous high-speed overtaking using robust MPC," *IEEE T-ITS*, 2019
- **[43]** Huang et al., "A path planning method for vehicle overtaking using sigmoid functions," *IFAC*, 2019

---

<div align="center">

**⭐ If this work is useful to your research, please consider starring the repository.**

*Connected Autonomous Vehicles · Model Predictive Control · Lane-Change Coordination · Intelligent Transportation Systems*

</div>
