# Control Barrier Function (CBF) Multi-Agent Obstacle Avoidance

A MATLAB implementation of Control Barrier Functions for safe multi-robot navigation in the Robotarium platform. This project demonstrates CBF-based collision avoidance between robots while ensuring goal convergence.

## Overview

Control Barrier Functions (CBFs) provide safety guarantees in control systems by constraining control inputs to maintain system safety. This implementation focuses on:
- Multi-robot collision avoidance
- Boundary constraint satisfaction  
- Real-time safe navigation with deadlock resolution
- Visualization of CBF behavior for analysis

## Control Barrier Functions Theory

### Mathematical Foundation

A **Control Barrier Function** $h(x)$ defines a safety set $\mathcal{S} = \{x : h(x) \geq 0\}$. To ensure the system remains safe (stays in $\mathcal{S}$), we require:

$$\dot{h}(x,u) \geq -\gamma h(x)$$

where:
- $x$ is the system state
- $u$ is the control input  
- $\gamma > 0$ is the CBF parameter controlling conservativeness
- $\dot{h}(x,u) = \nabla h(x)^T f(x,u)$ is the time derivative of the barrier function

### Safety Constraints

For robot $i$ at position $p_i$, we define barrier functions for:

**Robot-Robot Avoidance:**
```matlab
h_robot = ||p_i - p_j|| - d_safe
```
where $d_{safe}$ is the minimum safe distance.

**Boundary Constraints:**
```matlab  
h_boundary = p_i - boundary_limit - margin
```

The CBF constraint becomes:
$$\nabla h(x)^T u \geq -\gamma h(x)$$

### Control Optimization

The safe control is found by solving:
$$\min_{u} ||u - u_{desired}||^2$$
$$\text{subject to: } A_{cbf} u \geq b_{cbf}$$

where $A_{cbf}$ and $b_{cbf}$ encode the CBF constraints.

## Implementation Details

### Symmetric CBF for Multi-Robot Systems

To handle robot-robot interactions fairly, each robot uses **symmetric weights**:

```matlab
% Each robot takes 50% responsibility for avoidance
weight = 0.5;
A_cbf = [A_cbf; weight * gradient'];
b_cbf = [b_cbf; -gamma * h_robot * weight];
```

This prevents one robot from being overly passive while ensuring coordinated avoidance.

### CBF Activation

CBF constraints are only activated when robots are within an **activation distance**:

```matlab
if h_robot < activation_distance && distance > 0
    % Apply CBF constraint
    gradient = (pos - robot_pos) / distance;
    A_cbf = [A_cbf; gradient'];
    b_cbf = [b_cbf; -gamma * h_robot];
end
```

This reduces computational load and prevents unnecessary constraint activation.

## Key Parameters

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `gamma_cbf` | CBF aggressiveness (higher = more conservative) | 2.0 |
| `safety_margin_robots` | Minimum distance between robots (m) | 0.3 |
| `safety_margin_boundaries` | Minimum distance to arena boundaries (m) | 0.2 |
| `activation_distance` | Distance to activate CBF constraints (m) | 0.8 |
| `max_linear_velocity` | Maximum robot speed (m/s) | 0.1 |

### CBF Parameter Effects

- **γ ↑**: More conservative avoidance, larger safety margins
- **γ ↓**: More aggressive behavior, tighter trajectories
- **activation_distance ↑**: Earlier constraint activation, smoother avoidance
- **safety_margin ↑**: Larger safe zones, more conservative behavior

## File Descriptions

### Core Implementation Files

- **`cbf_obstacle_avoidance.m`**: Offline simulation with full visualization
  - Complete CBF implementation with real-time plotting
  - Performance analysis and trajectory visualization
  - Ideal for parameter tuning and algorithm understanding

- **`cbf_robotarium_online.m`**: Online Robotarium deployment
  - Optimized for real hardware execution  
  - Arena floor projections for live visualization
  - Data collection for post-analysis

- **`plot_cbf_trajectory_data.m`**: Post-experiment visualization
  - Recreates Figure 2 style plots from collected data
  - Performance metrics and efficiency analysis

### Helper Functions

```matlab
function [A_cbf, b_cbf, cbf_active] = setupCBFConstraints(pos, other_robots, boundaries, ...)
% Sets up CBF constraint matrices for QP solver

function u_safe = solveCBFQP(u_desired, A_cbf, b_cbf, v_max)  
% Solves the CBF-QP optimization problem
```

## Deadlock Resolution

### The Deadlock Problem

CBF can create **deadlock situations** when:
- Multiple robots approach each other simultaneously
- CBF constraints become overly restrictive
- All robots reduce velocity to near-zero

### Perturbation-Based Solution

We implement a **random perturbation** mechanism:

```matlab
% Detect deadlock: low average velocity over time window
if avg_velocity < deadlock_movement_threshold && pose_idx > 10
    deadlock_timer(i) = deadlock_timer(i) + sampleTime;
    
    % Apply random perturbation after threshold time
    if deadlock_timer(i) > deadlock_detection_time
        angle = atan2(u_safe(2), u_safe(1)) + (rand()-0.5)*pi;
        perturbation = magnitude * [cos(angle); sin(angle)];
        u_safe = u_safe + perturbation;
    end
end
```

**Key Features:**
- Scales naturally to N robots (no coordination required)
- Temporary intervention (1 second perturbation)
- Cooldown period prevents oscillations
- Minimal impact on normal CBF behavior

## Visualization Features

### Arena Floor Projections

The online implementation projects visual elements onto the physical arena:

- **Safety Circles**: Semi-transparent circles showing `safety_margin_robots`
- **Detection Ranges**: Dashed circles appearing when CBF is active  
- **Trajectory Trails**: Fading paths showing recent robot movement
- **Start/Goal Markers**: Circles (start) and pentagons (goals)

### Performance Control

```matlab
% Speed vs visualization trade-off
enable_visualization = true;     % Master switch
update_frequency = 5;           % Update every N iterations
show_trajectories = true;       % Individual feature toggles
```

## Usage Instructions

### Basic Simulation

```matlab
% Run offline simulation with full visualization
cbf_obstacle_avoidance

% Run online Robotarium version  
cbf_robotarium_online

% Visualize collected data
plot_cbf_trajectory_data
```

### Parameter Tuning

1. **For more conservative behavior**: Increase `gamma_cbf`, increase `safety_margin_robots`
2. **For faster execution**: Decrease `update_frequency`, set `enable_visualization = false`  
3. **For deadlock issues**: Adjust `deadlock_detection_time`, `perturbation_magnitude`

### Data Collection

The online version automatically saves trajectory data:
```matlab
save('CBF_Trajectory_Data.mat', 'final_poses', 'final_velocities', 'final_cbf_active', ...);
```

## Mathematical Notation Summary

| Symbol | Description |
|--------|-------------|
| $h(x)$ | Barrier function defining safety set |
| $\mathcal{S} = \{x : h(x) \geq 0\}$ | Safety set |
| $\gamma$ | CBF parameter (aggressiveness) |
| $u_{desired}$ | Unconstrained desired control |
| $u_{safe}$ | CBF-constrained safe control |
| $\nabla h(x)$ | Gradient of barrier function |
| $d_{safe}$ | Minimum safe inter-robot distance |

## Algorithm Flow

```
1. Get robot positions and goals
2. Compute desired velocity (toward goal)  
3. Set up CBF constraints:
   - Robot-robot avoidance constraints
   - Boundary avoidance constraints
4. Solve QP: min ||u - u_desired||² subject to CBF constraints
5. Apply deadlock resolution if needed
6. Convert to unicycle control (v, ω)
7. Send to robots and repeat
```

This implementation provides a foundation for safe multi-robot systems with theoretical guarantees and practical deadlock handling.