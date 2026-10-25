# Discrete-Time Model Predictive Control (MPC) of an Inverted Pendulum Cart-Pole System

A complete MATLAB implementation of a discrete-time Model Predictive Control (MPC) framework designed to stabilize an inherently unstable inverted pendulum on a moving cart. The project covers linearized state-space derivation, recursive prediction matrix formulation, quadratic optimization (`quadprog`), setpoint trajectory tracking, actuator saturation limits, and side-by-side performance benchmarking.


---

## 📌 Project Overview

The primary objective is to balance an inverted pendulum upright (theta = 0 rad) while precisely moving the cart chassis to a target displacement (x_ref = 0.5 m) under strict physical constraints on input motor force (|u| <= 10 N).

### Key Features
* **Linearized Dynamics & Discretization**: Continuous-time physical model linearized around equilibrium and discretized using Zero-Order Hold (ZOH).
* **Explicit Prediction Matrices**: Manual recursive matrix formulation (M and Phi) projecting states and inputs over a finite horizon (N = 10).
* **Quadratic Programming**: Formulated as 1/2 U^T H U + f^T U solved at each time step via MATLAB's `quadprog`.
* **Actuator Saturation**: Explicit handling of lower and upper control bounds (lb, ub).
* **System Evaluation**: Automated extraction of peak metrics (maximum pole angle and peak control force) alongside custom trajectory and error plotting.
* **Constrained vs. Unconstrained Benchmarking**: Quantifies trade-offs in response time and transient overshoot when physical hardware caps are enforced.
* **Real-Time Visualization**: Dynamic handle-graphics MATLAB animation of the cart-pole system.

---
## ⚙️ System Parameters

| Parameter | Symbol | Value | Unit | Description |
| :--- | :---: | :---: | :---: | :--- |
| **Cart Mass** | *M_c* | 1.0 | kg | Mass of the cart base |
| **Pendulum Mass** | *m* | 0.2 | kg | Point mass of pendulum bob |
| **Pendulum Length** | *l* | 0.5 | m | Center of mass length |
| **Friction** | *k* | 0.1 | N·s/m | Cart surface damping coefficient |
| **Sampling Time** | *T_s* | 0.05 | s | Discrete controller sample time (50 ms) |
| **Prediction Horizon** | *N* | 10 | steps | Look-ahead window size |
---
## 📁 Repository Structure & Workflow

The repository follows a clean, chronological development structure from initial model derivation to final benchmarking:

| File | Module / Stage | Objective |
| :--- | :--- | :--- |
| `Day1_systemModeling.m` | **System Modeling & Linearization** | Defines physical parameters, forms continuous state space, verifies rank controllability, and discretizes via `c2d`. |
| `Day2_PredictionMatrices.m` | **Prediction Matrices** | Builds free-response matrix M and forced-response matrix Phi recursively. |
| `Day3_BuildingMPC.m` | **QP Formulation & Sanity Check** | Formulates state penalty Q, input penalty R, and terminal Riccati cost into symmetric H and f matrices for open-loop prediction testing. |
| `Day4_MPCsimulation.m` | **Closed-Loop Simulation** | Implements the receding-horizon loop over 100 simulation steps, applying only the first control action at each time step. |
| `Day5_stabilization.m` | **Setpoint Position Tracking** | Modifies reference to translate the cart to x = 0.5 m while maintaining pendulum equilibrium. |
| `Day6_RealWorldLimits.m` | **Actuator Saturation Limits** | Introduces lower and upper control bounds to force optimization within hardware constraints. |
| `Day7_DynamicVisualization.m` | **Real-Time Graphics** | Draws and animates the cart chassis and pendulum rod dynamically from recorded state history. |
| `Day8_EvaluateSystem.m` | **System Evaluation & Metrics** | Plots tracking error, position response, and pole angles while printing peak angle and maximum control force constraints. |
| `Day9_ConstrainedvsUnconstrained.m` | **Constrained vs. Unconstrained MPC** | Runs side-by-side performance benchmarking comparing unconstrained vs. constrained tracking error, control force saturation, and pole angle transients. |

---

## 📊 Results & Analysis

From the `Day8_EvaluateSystem.m` system evaluation and performance study:

1. **Tracking Performance**: The cart successfully tracks the target setpoint (x_ref = 0.5 m) while maintaining precise stabilization of the inverted pendulum at theta = 0 rad.
2. **Quantitative Metrics**: The evaluation script automatically extracts and outputs critical performance boundaries, confirming the maximum observed pole angle and peak control force under closed-loop operation.
---

## 🚀 How to Run

1. Clone this repository:
   
```bash
   git clone [https://github.com/begumbalci/cartpole-mpc-matlab.git](https://github.com/begumbalci/cartpole-mpc-matlab.git)
   cd cartpole-mpc-matlab
