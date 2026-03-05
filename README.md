# Comparative Metaheuristic PID Tuning for Nonlinear Quadrotor Control (MATLAB ODE Framework)

## Overview
This repository presents a comparative academic study of PID gain tuning for a nonlinear quadrotor model using:

- Genetic Algorithm (GA)
- Particle Swarm Optimization (PSO)
- Differential Evolution (DE)
- Teaching–Learning-Based Optimization (TLBO)

A Linear Quadratic Integral (LQI) controller is included as an intelligent baseline for comparison.

The main study workflow is MATLAB ODE-based (non-Simulink), using 100 independent runs and 60 iterations per run for each metaheuristic.

---

## Problem Statement
PID tuning for a nonlinear 6-DOF quadrotor is a constrained, nonconvex optimization problem. This work evaluates whether population-based metaheuristics can improve closed-loop tracking and overall control quality relative to baseline methods under a common simulation and evaluation pipeline.

---

## System Model
- **Plant:** nonlinear quadrotor dynamics solved with MATLAB `ode45`.
- **Dynamics:** 6-DOF translational/rotational motion.
- **Control structure:** inner/outer PID loops for:
  - altitude (`z`)
  - roll (`\phi`)
  - pitch (`\theta`)
  - yaw (`\psi`)
- **Actuation:** thrust/torque mapping to rotor speeds with saturation.
- **Simulation horizon:** `t_f = 15 s`.

Core files:
- `quad_dynamics.m`
- `quad_pid_control.m`
- `QuadrotorPerformance.m`
- `quad_params.m`

---

## Controllers Implemented
1. **Metaheuristic-tuned PID**
   - 12 gains: `[Kp, Ki, Kd]` for each channel (`z, \phi, \theta, \psi`).
2. **PID baseline via MATLAB `pidtune`**
   - Script: `pidtune_baseline.m`.
3. **LQI baseline controller**
   - `LQI/build_LQI_controller.m`
   - `LQI/LQI_QuadrotorPerformance.m`
   - `LQI/run_LQI.m`

---

## Objective (Cost) Function
The fitness value `Q` (in `QuadrotorPerformance.m`) is the time-averaged weighted sum of:

1. Normalized tracking errors for `z, \phi, \theta, \psi`
2. Normalized rotor-speed effort

\[
Q = \frac{1}{T} \int_0^T \left(Q_1(e_z,e_\phi,e_\theta,e_\psi) + \alpha Q_2(U)\right) dt
\]

Weights/normalization are set in `quad_params.m` (`W_z`, `W_phi`, `W_theta`, `W_psi`, `W_U`, `W_u_scale`, `z_max`, `angle_max`, `U_norm`).

---

## Gain Bounds Used
### GA / PSO / DE bounds
- Lower:
  - `z`: `[0.02, 1e-5, 0.02]`
  - `\phi, \theta, \psi`: `[1e-4, 1e-6, 1e-4]`
- Upper:
  - `z`: `[2.0, 0.2, 2.0]`
  - `\phi`: `[0.3, 0.05, 0.3]`
  - `\theta`: `[0.3, 0.05, 0.3]`
  - `\psi`: `[0.4, 0.05, 0.4]`

### TLBO bounds
- Lower:
  - `z`: `[0.01, 1e-5, 0.01]`
  - `\phi, \theta, \psi`: `[1e-4, 1e-6, 1e-4]`
- Upper (all channels):
  - `[10.0, 1.0, 5.0]`

---

## Optimization Protocol
For each metaheuristic:
- **Independent runs:** `NRuns = 100`
- **Iterations per run:** `T = 60`
- **Population/swarm size:** `Np = 15`
- **Seed handling:** `rng(i,'twister')` per run

Entry scripts:
- GA: `GA/GAQuadrotor.m`
- PSO: `PSO/PSOQuadrotor.m`
- DE: `DE/DEQuadrotor.m`
- TLBO: `TLBO/TLBOQuadrotor.m`

Generated outputs include best gains, run-level fitness data, convergence history, trajectory/response figures, and CSV exports.

---

## Statistical Validation
Statistical comparison is implemented in:
- `csvs/ANOVA_test.m`

It performs:
- One-way ANOVA on final best-fitness distributions
- Tukey-Kramer post-hoc testing
- Pairwise Hedges' g and Welch t-tests
- CSV exports (`Q_descriptive_stats.csv`, `Q_tukey.csv`, `Q_pairwise_hedgesg.csv`)

---

## Repository Structure (High-Level)
- `GA/`, `PSO/`, `DE/`, `TLBO/` — optimization scripts and helpers
- `LQI/` — LQI baseline implementation
- Root-level simulation/control utilities (`quad_*.m`, `QuadrotorPerformance.m`, `save_metrics_csv.m`)
- `compare_responses.m` — overlay plots for altitude and attitude comparisons
- `csvs/` — ANOVA/statistical scripts and exported tables

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
- Simulink artifacts currently present in the repository are legacy outputs and will be removed during repository cleanup.

---

## License
License information will be added here.
