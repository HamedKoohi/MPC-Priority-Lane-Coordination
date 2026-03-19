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
> *Hamed Kouhi, Georg Schildbach — Scientific Reports, 2024*

Connected autonomous vehicles (CAVs) can cooperate via vehicle-to-infrastructure (V2I) communication to achieve traffic-level efficiency that no single vehicle can attain alone. Most existing work addresses lane changes for a **single ego vehicle**. This paper fills the gap by presenting a **centralized framework** that jointly optimizes lane assignments and speed profiles for **all vehicles simultaneously** on a multi-lane highway.

### The Core Problem

Jointly optimizing lane changes for *n* vehicles over a finite horizon is a **mixed-integer optimization problem** whose complexity grows **exponentially** with the number of vehicles — making brute-force global search impossible in real time.

### Our Solution

A **priority-aware search strategy** embedded inside an MPC loop:

- At each time step, score every vehicle using a composite cost function
- Restrict the search to the **top-3 priority vehicles** only
- Evaluate only `2³ = 8` lane-change combinations instead of the full combinatorial space
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
| **Scalable architecture** | Total runtime scales as `O(n_set,L³)` rather than exponentially with vehicle count |

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
│  │  → Score each vehicle with j₁, j₂, j₃               │   │
│  │  → Select top-3 candidates: O₁, O₂, O₃              │   │
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

The lateral position during a lane change is modeled with a **sigmoid function** that naturally enforces smooth transitions:

```
y(t) = d_lat · σ(t)

where  σ(t) = 1 / (1 + exp(−k_sig · (t − t₀)))
```

The sharpness parameter `k_sig` is bounded by both lateral acceleration and jerk limits:

```
k_sig,acc  = sqrt(6√3 · a_y,max / d_lat)
k_sig,jerk = (j_y,max / (C · d_lat))^(1/3)    [C ≈ 0.096]

k_sig = min(k_sig,acc , k_sig,jerk)
```

**Desirable properties of the sigmoid model:**
- ✅ Smooth continuous transition between lanes
- ✅ Zero slope at start and end (no sudden steering inputs)
- ✅ Continuous velocity and acceleration profiles
- ✅ Tunable aggressiveness via `k_sig`
- ✅ Physically feasible — satisfies lateral jerk constraints

---

### Priority-Based Search Strategy

Each vehicle `i_veh` is scored with a composite priority cost:

```
J_vec(i_veh) = j₁ + α₂·j₂ + α₃·j₃
```

| Term | Formula | Meaning |
|------|---------|---------|
| `j₁` | `(Vel(i_veh) − V_des(i_veh))²` | Speed deficit — how urgently this vehicle needs to change lanes |
| `j₂` | `(x_safe,back2 / dist_back2)^N + (x_safe,front2 / dist_front2)^N` | Density/crowding in the **current** lane |
| `j₃` | `(dist_back3 / x_safe,back3)^N + (dist_front3 / x_safe,front3)^N` | Gap availability in the **target** lane |

Weights `α₂ = α₃ = 1` are selected via systematic simulation-based tuning. A safety override sets `J_vec = −1` (no lane change permitted) whenever the target lane gap is insufficient.

The three highest-scoring vehicles `{O₁, O₂, O₃}` become the lane-change candidates for the current time step.

---

### MPC Cost Function

The total cost minimized at each receding-horizon step:

```
J = J₁ + J_antiLock + J_col + w₅‖L_vec − L_vec,0‖² + w₆‖(U_old − U(it))/m‖²
```

| Term | Role |
|------|------|
| `J₁ = w₁‖Vel − V_des‖² + w₂‖U/m‖²` | Speed tracking + input energy |
| `J_antiLock` | Deadlock prevention for left-lane vehicles whose desired speed < actual speed |
| `J_col` | APF-based collision avoidance penalty |
| `w₅‖L_vec − L_vec,0‖²` | Penalizes unnecessary lane changes |
| `w₆‖(U_old − U(it))/m‖²` | Jerk minimization (smoothness) |

---

### APF Collision Avoidance

An Artificial Potential Field penalty `J_c1` between vehicles A and B in the same lane:

```
If |x_B0 − x_A0| ≤ σ·x_safe  and  x_B > x_A0:

  F₁ = ((x_A − x_B) / (x_safe × σ))²
  J_c1 = F₁^N / (F₁^N + ε₁)          ← smooth, bounded repulsion

  If |x_A0 − x_B0| < x_safe:
    J_c1 ← J_c1 × c_col               ← amplified near-collision penalty
```

This formulation produces a **smooth, differentiable** repulsive potential that approaches a hard wall as vehicles close in, without the local-minimum issues of classical APF methods.

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
Total Runtime ≈ (n_set,L)³ × (MPC Runtime for longitudinal control)
              + (Priority Logic Runtime)
```

With `n_set,L = 2` → only **8 MPC evaluations** per time step, regardless of total vehicle count.

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

The consistent ~8× ratio between total and MPC-only runtime confirms the `(n_set,L)³ = 8` scaling, as opposed to the **exponential** growth of full mixed-integer optimization.

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
| `x_safe,0` | 5 m | Minimum safe following distance |
| `u_min` | −20 N | Maximum braking force |
| `u_max` | 5 N | Maximum acceleration force |

### Controller Weights

| Symbol | Value | Role |
|--------|-------|------|
| `α₂, α₃` | 1 | Lane density / target-lane safety balance |
| `w₁` | 0.1 | Speed tracking |
| `w₃` | 500 | APF collision repulsion |
| `w₄` | 1000 | APF collision base |
| `w₅` | 20 | Unnecessary lane-change penalty |
| `w₆` | 200 | Jerk smoothness |
| `W_antiLock` | 1 | Deadlock prevention |
| `c_col` | 1000 | Near-collision amplification |
| `w_xsafe` | 1/4 | Safe distance speed-dependent scaling |
| `N` | 2 | APF exponent |
| `N₂` | 12 | Density exponent |
| `ε₁` | 0.001 | APF numerical regularization |

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
│   ├── Algorithm2_1_j2.m           # Calculating current-lane density cost j₂
│   ├── Algorithm2_2_j3.m           # Calculating target-lane gap cost j₃
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
@article{kouhi2024cooperative,
  title   = {Scalable Cooperative Lane-Change Management for Connected
             Autonomous Vehicles Using MPC-Based Decision Coordination},
  author  = {Kouhi, Hamed and Schildbach, Georg},
  journal = {Scientific Reports},
  year    = {2024},
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
