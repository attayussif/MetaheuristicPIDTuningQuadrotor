# Comparative Analysis of Metaheuristic PID Tuning for Quadrotor Altitude and Attitude Stabilization (MATLAB ODE Framework)

## Overview
This repository contains an academic comparative study of PID gain tuning for a nonlinear quadrotor model using four metaheuristics:

- Genetic Algorithm (GA)
- Particle Swarm Optimization (PSO)
- Differential Evolution (DE)
- Teaching–Learning-Based Optimization (TLBO)

A Linear Quadratic Integral (LQI) controller is also implemented as a baseline intelligent controller for comparison.

The simulation workflow is MATLAB ODE-based (non-Simulink for the main study pipeline), with 100 independent optimization runs and 60 iterations per run for each metaheuristic.

---

## Problem Statement
PID tuning for a nonlinear 6-DOF quadrotor is a nonconvex optimization problem with coupled altitude/attitude dynamics and actuator constraints. This project evaluates whether population-based metaheuristics can produce lower tracking cost and better control performance than baseline tuning strategies under a unified simulation and evaluation protocol.

---

## System Model
- **Plant:** nonlinear quadrotor dynamics in MATLAB (`ode45`), 6-DOF motion.
- **State form:** translational + rotational dynamics with integral error states for PID loops.
- **Control architecture:** cascaded/stacked PID structure for:
  - altitude $$(`z`)$$
  - roll $$(`\phi`)$$
  - pitch $$(`\theta`)$$
  - yaw $$(`\psi`)$$
- **Actuation:** thrust/torque allocation to rotor speeds with saturation limits.
- **Simulation horizon:** $$`t_f = 15 s`.$$

Core model files:
- `quad_dynamics.m`
- `quad_pid_control.m`
- `QuadrotorPerformance.m`
- `quad_params.m`

---

## Controllers Implemented
1. **PID (metaheuristic-tuned)**
   - 12 gains optimized: `[Kp, Ki, Kd]` for each of `z, \phi, \theta, \psi`.
2. **PID baseline via MATLAB `pidtune`**
   - Script: `pidtune_baseline.m`.
3. **LQI baseline controller**
   - Build + simulation scripts in `LQI/`:
     - `build_LQI_controller.m`
     - `LQI_QuadrotorPerformance.m`
     - `run_LQI.m`

---

## Objective (Cost) Function
The scalar fitness `Q` is computed in `QuadrotorPerformance.m` as a time-averaged weighted sum of:

1. **Normalized tracking error terms** for altitude and Euler angles:
   - `z, \phi, \theta, \psi`
2. **Normalized control effort** based on rotor-speed magnitudes.

In compact form:

$$
Q = \frac{1}{T} \int_0^T \left(Q_1(e_z,e_\phi,e_\theta,e_\psi) + \alpha Q_2(U)\right) dt
$$

where the weights and normalization constants are defined in `quad_params.m` (`W_z`, `W_phi`, `W_theta`, `W_psi`, `W_U`, `W_u_scale`, `z_max`, `angle_max`, `U_norm`).

---

## Gain Bounds Used
### GA / PSO / DE/TLBO bounds
- Lower bounds:
  - `z`: `[0.02, 1e-5, 0.02]`
  - $$`\phi, \theta, \psi`: `[1e-4, 1e-6, 1e-4]`$$
- Upper bounds:
  - `z`: `[2.0, 0.2, 2.0]`
  - `\phi`: `[0.3, 0.05, 0.3]`
  - `\theta`: `[0.3, 0.05, 0.3]`
  - `\psi`: `[0.4, 0.05, 0.4]`
---

## Optimization Protocol
For each metaheuristic:
- **Independent runs:** `NRuns = 100`
- **Iterations per run:** `T = 60`
- **Population/swarm size:** `Np = 15`
- **Random seed control:** `rng(i,'twister')` per run index.

Algorithm-specific scripts:
- GA: `GA/GAQuadrotor.m`
- PSO: `PSO/PSOQuadrotor.m`
- DE: `DE/DEQuadrotor.m`
- TLBO: `TLBO/TLBOQuadrotor.m`

Typical outputs per algorithm include:
- best gains (`best_gains_*.mat`)
- run-wise fitness logs (`*_results.mat`)
- summary statistics CSV
- time-history and metrics CSV via `save_metrics_csv.m`
- convergence and trajectory figures.

---

## Statistical Validation
The repository includes one-way ANOVA and post-hoc analysis for final fitness distributions:

- Script: `csvs/ANOVA_test.m`
- Inputs: `bestfitness` arrays from GA/PSO/DE/TLBO result files
- Outputs:
  - ANOVA statistics (`F`, `p`, effect sizes)
  - Tukey-Kramer pairwise table
  - Pairwise Hedges' g + Welch t-test p-values
  - Exported CSV tables (`Q_descriptive_stats.csv`, `Q_tukey.csv`, `Q_pairwise_hedgesg.csv`)

---

## Repository Structure (High-Level)
- `GA/` – GA optimizer and utilities
- `PSO/` – PSO optimizer and utilities
- `DE/` – DE optimizer and utilities
- `TLBO/` – TLBO optimizer and utilities
- `LQI/` – LQI baseline controller implementation
- Root model/utilities:
  - `quad_params.m`, `quad_dynamics.m`, `quad_pid_control.m`, `QuadrotorPerformance.m`
  - `save_metrics_csv.m`
  - `compare_responses.m` (overlay comparisons: altitude/attitude)
  - `pidtune_baseline.m`
- `csvs/` – statistical scripts and exported analysis tables

---

## Software Requirements
- **MATLAB:** **R2025b**
- **Toolboxes:**
  - Control System Toolbox (`pidtune`, `lqr`, `tf`)
  - Statistics and Machine Learning Toolbox (`anova1`, `multcompare`, `ttest2`)

> Main comparative runs use the MATLAB ODE pipeline (not Simulink).

---

## Hardware and Runtime
### Hardware used
- **CPU:** AMD Ryzen 7 7000 series
- **RAM:** 16 GB
- **OS:** Windows
- **MATLAB version:** R2025b

### Expected runtime
Each algorithm runs `100 × 60` optimization iterations, and each objective call evaluates nonlinear ODE dynamics. Full experiments are computationally intensive and can take **hours per algorithm**, depending on machine load and MATLAB configuration.

---

## Reproducibility: Step-by-Step
Run commands from the repository root in MATLAB.

### 1) Add required folders to MATLAB path
```matlab
addpath(genpath(pwd));
```

### 2) Run baseline controllers
```matlab
% PID baseline via pidtune
gains_pid = pidtune_baseline;

% LQI baseline
run('LQI/run_LQI.m');
```

### 3) Run each metaheuristic
```matlab
run('GA/GAQuadrotor.m');
run('PSO/PSOQuadrotor.m');
run('DE/DEQuadrotor.m');
run('TLBO/TLBOQuadrotor.m');
```

### 4) Run statistical analysis (ANOVA)
```matlab
cd csvs
run('ANOVA_test.m')
cd ..
```

### 5) Generate overlay plots (altitude + attitude)
```matlab
run('compare_responses.m');
```

Exports:
- `altitude_comparison.png`
- `roll_comparison.png`
- `pitch_comparison.png`
- `yaw_comparison.png`

### 6) Run LQI comparison only
```matlab
run('LQI/run_LQI.m');
```

---

## Running Individual Algorithms (Quick Commands)
```matlab
addpath(genpath(pwd));
run('GA/GAQuadrotor.m')
run('PSO/PSOQuadrotor.m')
run('DE/DEQuadrotor.m')
run('TLBO/TLBOQuadrotor.m')
```

---

## Notes
- The comparative workflow documented here is based on MATLAB scripts and ODE simulation.


---

## License
License information will be added here.
