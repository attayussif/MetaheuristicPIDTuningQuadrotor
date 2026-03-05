# Comparative Metaheuristic PID Tuning for Nonlinear Quadrotor Control (MATLAB ODE Framework)

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
  - altitude (`z`)
  - roll (`\phi`)
  - pitch (`\theta`)
  - yaw (`\psi`)
- **Actuation:** thrust/torque allocation to rotor speeds with saturation limits.
- **Simulation horizon:** `t_f = 15 s`.

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

\[
Q = \frac{1}{T} \int_0^T \left(Q_1(e_z,e_\phi,e_\theta,e_\psi) + \alpha Q_2(U)\right) dt
\]

where the weights and normalization constants are defined in `quad_params.m` (`W_z`, `W_phi`, `W_theta`, `W_psi`, `W_U`, `W_u_scale`, `z_max`, `angle_max`, `U_norm`).

---

## Gain Bounds Used
### GA / PSO / DE bounds
- Lower bounds:
  - `z`: `[0.02, 1e-5, 0.02]`
  - `\phi, \theta, \psi`: `[1e-4, 1e-6, 1e-4]`
- Upper bounds:
  - `z`: `[2.0, 0.2, 2.0]`
  - `\phi`: `[0.3, 0.05, 0.3]`
  - `\theta`: `[0.3, 0.05, 0.3]`
  - `\psi`: `[0.4, 0.05, 0.4]`

### TLBO bounds
- Lower bounds:
  - `z`: `[0.01, 1e-5, 0.01]`
  - `\phi, \theta, \psi`: `[1e-4, 1e-6, 1e-4]`
- Upper bounds (all channels):
  - `[10.0, 1.0, 5.0]`

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
- **MATLAB:** recommended R2021b or newer (tested workflow is MATLAB-script based).
- **Required toolboxes:**
  - Control System Toolbox (for `pidtune`, `lqr`, `tf`)
  - Statistics and Machine Learning Toolbox (for `anova1`, `multcompare`, `ttest2`)

> Note: Main optimization and plant simulation are ODE-script based (no Simulink required for the primary pipeline).

---

## Hardware and Runtime
### Hardware used in study
Please fill in your exact test machine for reproducibility in publications:
- CPU: `<your CPU model>`
- RAM: `<your RAM>`
- OS: `<your OS>`
- MATLAB version: `<your MATLAB release>`

### Expected runtime
Each algorithm executes `100 × 60` optimization iterations, and each objective evaluation runs nonlinear ODE simulation. Runtime therefore depends strongly on CPU speed and MATLAB settings. On a typical desktop/laptop, full multi-run experiments may take **hours** per algorithm.

---

## Reproducibility: Step-by-Step
Run commands from the repository root in MATLAB.

### 1) Initialize baseline(s)
```matlab
% PID baseline from pidtune
gains_pid = pidtune_baseline;

% LQI baseline
run('LQI/run_LQI.m');
```

### 2) Run each metaheuristic
```matlab
% Genetic Algorithm
run('GA/GAQuadrotor.m');

% Particle Swarm Optimization
run('PSO/PSOQuadrotor.m');

% Differential Evolution
run('DE/DEQuadrotor.m');

% Teaching-Learning-Based Optimization
run('TLBO/TLBOQuadrotor.m');
```

### 3) Run statistical validation (ANOVA)
```matlab
cd csvs
run('ANOVA_test.m')
cd ..
```

### 4) Generate overlay comparison plots (altitude + attitude)
```matlab
run('compare_responses.m');
```

This exports:
- `altitude_comparison.png`
- `roll_comparison.png`
- `pitch_comparison.png`
- `yaw_comparison.png`

### 5) Run LQI-only comparison/evaluation
```matlab
run('LQI/run_LQI.m');
```

This exports LQI time history and metrics CSV files.

---

## Running Individual Algorithms (Quick Commands)
```matlab
run('GA/GAQuadrotor.m')
run('PSO/PSOQuadrotor.m')
run('DE/DEQuadrotor.m')
run('TLBO/TLBOQuadrotor.m')
```

---

## Notes
- The core comparative framework is implemented with MATLAB ODE scripts.
- Some legacy files/folders may still reference Simulink artifacts from earlier development; they are not required for the ODE-based comparative workflow described above.

---

## Citation (BibTeX Placeholder)
```bibtex
@misc{quadrotor_metaheuristic_pid_study,
  title        = {Comparative Metaheuristic PID Tuning for Nonlinear Quadrotor Control},
  author       = {Author(s) Placeholder},
  year         = {2026},
  howpublished = {GitHub repository},
  note         = {Add DOI / URL / publication details}
}
```

---

## License
License information will be added here.
