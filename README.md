# UAV Suspension System

Monte Carlo optimization of landing-gear suspension parameters for a 4-wheeled paramotor-based UAV. Sweeps thousands of `(k, c)` spring/damper combinations under randomized touchdown conditions to find a design that maximizes successful landings while staying inside structural and attitude limits.

The repo contains both the original Python prototype (`MonteCarlo.py`) and a faster, physics-corrected C++ port (`monte_carlo.cpp`) that produced the figures in the accompanying AIAA paper.

## Contents

| File | Description |
|------|-------------|
| `monte_carlo.cpp` | Production C++ implementation (OpenMP, RK4, CSV + plot) |
| `MonteCarlo.py` | Original Python prototype |
| `matplotlibcpp.h` | Header-only matplotlib bindings used by the C++ build |
| `monte_carlo_results.csv` | Latest C++ run output |
| `iter_final.png` | Success-count scatter over the `(k, c)` plane |
| `Parth_Mhaske___Research_Paper_AIAA.pdf` | Full write-up |

## Physics Model

The UAV is modeled as a rigid body with three DOF — vertical CG translation, roll, and pitch — and four independent spring-damper legs. Each leg only applies force when compressed past its free length (one-way springs). Vertical drag uses `|v|·v` so it opposes motion in both directions (the C++ port fixes a sign bug in the original Python `v²` term).

### Equations of motion

```
m · z_CG'' = ΣF_wheels − m·g − DRAG_CONST · |z_CG'| · z_CG'
I_x · α''  = (w/2) · (F_FL − F_FR + F_BL − F_BR)
I_y · β''  = (l/2) · (F_FL − F_BL + F_FR − F_BR)
F_wheel    = −k · (length − L_free) − c · (compression_rate)   if length ≤ L_free else 0
```

Thin-plate moments of inertia: `I_x = (1/12)·m·w²`, `I_y = (1/12)·m·l²`.

### Vehicle parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `m` | 140 kg | UAV mass |
| `L_free` | 0.5 m | Spring free length |
| `L_solid` | 0.2 m | Spring solid length (0.3 m usable travel) |
| `r` | 0.2 m | Wheel radius |
| `w` | 0.6 m | Track width |
| `l` | 1.5 m | Wheelbase |
| `DRAG_CONST` | 32.17 | `0.5 · ρ · C_P · A_P` (parachute) |

### Touchdown sampling (C++)

Rather than simulating free-fall, the C++ version starts each trial at first wheel contact and samples touchdown velocity directly from `U(4.0, 7.5) m/s` — covering aggressive-flare landings up through gust-augmented terminal descent. Initial roll/pitch are sampled from `U(−0.2, 0.2) rad` and angular rates from `U(−0.1, 0.1) rad/s`.

### Failure modes (C++)

A trial fails if any of the following occurs during the 2-second integration window:

1. **Bottoming** — any spring compresses below `L_solid`
2. **Force overshoot** — peak leg force exceeds `m·g/4 + travel·k`
3. **Attitude divergence** — `|α|` or `|β|` exceeds 0.2 rad at any time
4. **Settling** — `|z_CG'|` at `t = T_final` exceeds 0.3 m/s (still ringing)

The Python prototype only checked modes 1 and 2 and re-rolled initial conditions on attitude divergence, which silently exempted underdamped configs from the realism check.

## Building and Running

### C++ (recommended)

```bash
# Get the matplotlib-cpp header
wget https://raw.githubusercontent.com/lava/matplotlib-cpp/master/matplotlibcpp.h

# Build
g++ -std=c++17 -O3 -fopenmp monte_carlo.cpp \
    $(python3-config --includes) \
    $(python3-config --ldflags --embed) \
    -o monte_carlo

# Run
./monte_carlo
```

Requires Python 3 with `matplotlib` and `pandas` installed for plotting. On headless systems the program automatically switches matplotlib to the `Agg` backend.

Outputs:
- `monte_carlo_results.csv` — per-`(k, c)` success/fail counts and force/length stats
- `monte_carlo.png` — success-count scatter over the parameter plane
- Console summary with optimal `(k, c)`, success rate, and tiebreaker stats

### Python

```bash
pip install numpy scipy tqdm matplotlib
python MonteCarlo.py
```

## Optimization Setup

| | Python | C++ |
|---|---|---|
| Combinations | 3,000 | 5,000 |
| Trials per combo | 300 | 1,000 |
| `k` range | 1,000 – 10,000 N/m | 5,000 – 30,000 N/m |
| `c` range | 100 – 1,000 N·s/m | 100 – 1,500 N·s/m |
| Selection | Max success/fail ratio | Max success count, ties broken by smallest `avg_max_force` |
| Parallelism | `ProcessPoolExecutor` | OpenMP `parallel for` |

The C++ selection criterion fixes a bias in the Python version: success/fail ratio silently excluded any cell with zero failures (division by zero), throwing out the best-performing parameter combinations. The force-margin tiebreaker prefers designs with more headroom below the structural limit.

## Tuning

The main constants live near the top of `monte_carlo.cpp`:

```cpp
constexpr int    N_COMBINATIONS     = 5000;
constexpr int    N_TRIALS_PER_COMBO = 1000;
constexpr double V_TD_MIN = 4.0, V_TD_MAX = 7.5;
constexpr double T_FINAL = 2.0;
constexpr int    N_STEPS = 700;
```

For a quick smoke test, drop `N_COMBINATIONS` to ~200 and `N_TRIALS_PER_COMBO` to ~100.

## Reference

See `Parth_Mhaske___Research_Paper_AIAA.pdf` for the full derivation, parameter rationale, and discussion of results.

## License

This project is not open source. Please do not use or modify this code.
