# MayurBC_Drone_Altitude_Stabilization
# Drone Altitude Stabilization using PID Control
### CONTROL CRAFT Hackathon — Problem Statement 1

**Team:** Mayur BC, Kishore KY
**Institution:** BNM Institute of Technology (BNMIT)

---

## Problem Statement

Design a PID controller to stabilize a drone at a target altitude of **1 metre**, despite external wind disturbances. The system must meet the following performance specifications:

| Metric | Specification |
|---|---|
| Overshoot | < 10% |
| Settling Time | < 3 seconds |
| Steady-State Error | ≈ 0 |
| Disturbance Rejection | Recover to 1m after wind hit at t = 5s |

---

## Plant Model

The drone's vertical dynamics are modeled as a second-order transfer function:

```
        1
G(s) = ────────────
       s² + 2s + 5
```

This represents how the drone's altitude responds to a thrust input. The system is underdamped (tends to oscillate) and without a controller, it settles at only 0.2m — an 80% steady-state error.

---

## Approach

### 1. Open-Loop Analysis
First, the plant was simulated without any controller to understand its natural behavior. The drone failed to reach the target — confirming the need for a controller.

### 2. PID Controller Design
A PID controller was designed using MATLAB's `pidtune` function:

```
C(s) = Kp + Ki/s + Kd·s
```

**Tuned Gains:**
| Gain | Value | Role |
|---|---|---|
| Kp | 10.2543 | Drives drone toward target quickly |
| Ki | 13.1422 | Eliminates steady-state error; handles disturbance recovery |
| Kd | 1.9336 | Adds damping, suppresses overshoot |

### 3. Closed-Loop System
The closed-loop transfer function is:

```
         C(s)·G(s)
T(s) = ─────────────────
        1 + C(s)·G(s)
```

### 4. Disturbance Modeling
A wind disturbance of magnitude -0.3 is injected at t = 5s at the plant input. The disturbance transfer function is:

```
          G(s)
T_d(s) = ─────────────────
          1 + C(s)·G(s)
```

Total response = reference tracking response + disturbance response

---

## Results

### Step Response Metrics (All Specs Passed ✓)

| Metric | Result | Spec | Status |
|---|---|---|---|
| Overshoot | 3.35% | < 10% | ✅ PASS |
| Settling Time | 2.48 s | < 3 s | ✅ PASS |
| Steady-State Error | ≈ 0 | ≈ 0 | ✅ PASS |
| Rise Time | 0.47 s | — | ✅ Fast |

### Disturbance Response
- Wind disturbance of -0.3 injected at t = 5s
- Drone dips slightly below 1m
- Integral term (Ki) corrects the error automatically
- Drone returns to exactly 1m — zero steady-state error after disturbance

---

## Simulation Outputs

| Figure | Description |
|---|---|
| Figure 1 | Open-loop step response — drone reaches only 0.2m (no controller) |
| Figure 2 | Closed-loop step response — drone reaches 1m with PID controller |
| Figure 3 | Disturbance response — drone recovers to 1m after wind hit at t=5s |
| Animation | Live visual simulation showing drone altitude with PID gains overlay |

---

## Dependencies

- MATLAB R2024b
- Simulink R2024b
- Control System Toolbox
- Simulink 3D Animation (for visual simulation)
- Aerospace Blockset (for Parrot Mambo drone model)

---

## How to Run

1. Clone this repository
2. Open MATLAB R2024b
3. Navigate to the project folder in MATLAB
4. Open and run `drone_pid_simulation.m`
5. All figures and the performance report will be generated automatically

```matlab
% Quick start — run this in MATLAB Command Window
run('drone_pid_simulation.m')
```

The script will:
- Print the full performance report in the Command Window
- Open Figure 1: Open-loop response
- Open Figure 2: Closed-loop PID response
- Open Figure 3: Disturbance response
- Launch the live animation window

---



*CONTROL CRAFT Hackathon | BNMIT | 2025*
