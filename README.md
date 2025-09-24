# UAV Suspension System

A Python simulation tool that optimizes suspension parameters for a 4-wheeled vehicle using Monte Carlo methods and parallel processing. The system finds optimal spring stiffness and damping coefficients to maximize vehicle stability while avoiding failure conditions.

## Overview

This simulation models a vehicle's dynamic behavior under various suspension configurations. The vehicle is represented as a rigid body with:
- **Center of gravity motion** (vertical translation)
- **Roll motion** (rotation about longitudinal axis) 
- **Pitch motion** (rotation about lateral axis)

Each wheel has an independent spring-damper system that only provides upward force when compressed (one-way springs). The simulation uses differential equations to model the complete vehicle dynamics.

## Key Features

- **Monte Carlo optimization** - Tests thousands of parameter combinations
- **Parallel processing** - Utilizes multiple CPU cores for faster computation
- **Realistic physics** - Models spring/damper forces, rotational inertia, and ground contact
- **Failure detection** - Identifies spring bottoming and excessive force conditions
- **Statistical analysis** - Provides comprehensive success/failure statistics

## Installation

### Requirements
```bash
pip install numpy scipy tqdm
```

### Dependencies
- **numpy**: Numerical computations and array operations
- **scipy**: ODE integration using `odeint`
- **tqdm**: Progress bars for long-running simulations

## Usage


This runs the default optimization with:
- 3,000 different parameter combinations
- 300 simulation runs per combination
- Automatic parallel processing

### Custom Parameters
Modify the main function call to adjust simulation parameters:

```python
# Example: Faster run for testing
all_results = monte_carlo_simulation(
    num_iterations=100,        # Test fewer combinations
    num_runs_per_combination=50  # Fewer runs per combination
)
```

## Vehicle Parameters

The simulation uses the following vehicle configuration:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `m` | 140 kg | Vehicle mass |
| `L` | 0.5 m | Spring equilibrium length |
| `r` | 0.2 m | Wheel radius |
| `w` | 0.6 m | Vehicle width (wheel track) |
| `l` | 1.5 m | Vehicle length (wheelbase) |

## Optimization Parameters

The Monte Carlo search explores these ranges:

| Parameter | Range | Units | Description |
|-----------|-------|-------|-------------|
| `k` | 1,000 - 10,000 | N/m | Spring stiffness coefficient |
| `c` | 100 - 1,000 | N⋅s/m | Damping coefficient |

## Simulation Details

### Initial Conditions
Each simulation run starts with randomized initial conditions:
- **Roll/Pitch angles**: ±0.2 radians
- **Angular velocities**: ±0.1 rad/s  
- **Initial height**: 1.9-2.1 meters above ground

### Failure Conditions
A simulation run fails if:
1. **Spring bottoms out**: Any spring compresses below 0.2m
2. **Excessive force**: Force exceeds `343.35 + 0.3 × k` Newtons

### Success Metric
The optimizer maximizes the **success-to-failure ratio** for each parameter combination. Higher ratios indicate more reliable suspension designs.

## Output

The program provides detailed results including:

```
MONTE CARLO OPTIMIZATION RESULTS
================================================================================
Optimal k: 9450.341139436745, Optimal c: 290.27196314437174, Success/Fail Ratio: 299.0 --> (Author Note: 99.67% Success Rate For Optimal Parameters)
Average Minimum Spring Length for Optimal k, c: 0.23750406220323705
Average Maximum Force for Optimal k, c: 2875.8868902306353
Total Number of Successful Runs: 71988
Total Number of Failed Runs: 825312
================================================================================
```

## Performance

Runtime depends on your system specifications:
- **CPU cores**: More cores = faster parallel processing

### Optimization Tips
- Start with fewer iterations for testing: `num_iterations=100`
- Use fewer runs per combination for quick results: `num_runs_per_combination=10`
- Monitor CPU usage - the simulation will utilize all available cores

## Physics Model

### Equations of Motion
The system solves a coupled set of ODEs describing:
- **Vertical motion**: `m × z_CG'' = ΣF_wheels - m × g + drag_term`
- **Roll motion**: `I_roll × α'' = Σ(torques from wheel forces)`  
- **Pitch motion**: `I_pitch × β'' = Σ(torques from wheel forces)`

### Spring Forces
Each wheel force follows: `F = -k × (compression) - c × (compression_rate)`

Forces only activate when springs are compressed (length ≤ equilibrium length).

## Extending the Code

### Adding New Failure Conditions
```python
# Example: Add maximum angle constraint
max_angle = np.max([np.max(np.abs(alpha_t)), np.max(np.abs(beta_t))])
if max_angle > 0.3:  # 0.3 radians ≈ 17 degrees
    return "failed", min(min_lengths), max_angle
```

### Modifying Vehicle Parameters
Simply change the constants at the top of the file:
```python
m = 200      # Heavier vehicle
w = 0.8      # Wider track
l = 2.0      # Longer wheelbase
```

### Custom Optimization Objectives
Replace the success/failure ratio with other metrics:
```python
# Example: Minimize average maximum force
optimal_kc = min(max_forces_for_combinations, 
                key=max_forces_for_combinations.get)
```

## Troubleshooting

### Common Issues
- **Long runtime**: Reduce `num_iterations` and `num_runs_per_combination`
- **Memory errors**: Close other applications or reduce simulation size
- **No optimal solution found**: All combinations failed - check failure thresholds

### Debug Mode
Add print statements to track progress:
```python
print(f"Testing k={k:.1f}, c={c:.1f}")
```

## License

This project is not open source. Please do not use or modify this code.
