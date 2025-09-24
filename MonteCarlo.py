import numpy as np
from scipy.integrate import odeint
import random
from tqdm import tqdm
from concurrent.futures import ProcessPoolExecutor, as_completed

# --------------------------------------------------------------------
# Constants for the physical system
# --------------------------------------------------------------------
m = 140       # Mass of the vehicle (kg)
L = 0.5       # Equilibrium length of the spring (m)
r = 0.2       # Radius of the wheels (m)
w = 0.6       # Width between left/right wheels (m)
l = 1.5       # Length between front/rear wheels (m)
g = 9.81      # Gravitational acceleration (m/s^2)

# Globals to track ground contact state at four corners
n, q, s, p = 0, 0, 0, 0  # FL, FR, BL, BR contact flags


# --------------------------------------------------------------------
# Getter/setter utility functions for contact state (used inside ODE)
# --------------------------------------------------------------------
def get_n(): return n
def get_q(): return q
def get_s(): return s
def get_p(): return p

def set_n(value): global n; n = value
def set_q(value): global q; q = value
def set_s(value): global s; s = value
def set_p(value): global p; p = value


# --------------------------------------------------------------------
# Compute spring lengths and velocities
# --------------------------------------------------------------------
def calculate_lengths(z_CG, alpha, beta, w, l, r):
    """Compute spring lengths for all 4 corners of the suspension."""
    L_FL = z_CG - r + (w / 2) * alpha + (l / 2) * beta
    L_FR = z_CG - r - (w / 2) * alpha + (l / 2) * beta
    L_BL = z_CG - r + (w / 2) * alpha - (l / 2) * beta
    L_BR = z_CG - r - (w / 2) * alpha - (l / 2) * beta
    return L_FL, L_FR, L_BL, L_BR

def calculate_velocities(z_CG_p, alpha_p, beta_p, w, l):
    """Compute velocity of spring compression for each wheel."""
    L_FL_p = z_CG_p + (w / 2) * alpha_p + (l / 2) * beta_p
    L_FR_p = z_CG_p - (w / 2) * alpha_p + (l / 2) * beta_p
    L_BL_p = z_CG_p + (w / 2) * alpha_p - (l / 2) * beta_p
    L_BR_p = z_CG_p - (w / 2) * alpha_p - (l / 2) * beta_p
    return L_FL_p, L_FR_p, L_BL_p, L_BR_p


# --------------------------------------------------------------------
# Main ODE system: defines the dynamics of the vehicle
# --------------------------------------------------------------------
def rightSideODE(x, t, k, c, m, L, r, g, w, l):
    alpha, alpha_p, beta, beta_p, z_CG, z_CG_p = x

    # Get spring lengths and velocities
    L_FL, L_FR, L_BL, L_BR = calculate_lengths(z_CG, alpha, beta, w, l, r)
    L_FL_p, L_FR_p, L_BL_p, L_BR_p = calculate_velocities(z_CG_p, alpha_p, beta_p, w, l)

    # Compute spring-damper forces only if in contact (compression)
    F_FL = -k * (L_FL - L) - c * L_FL_p if L_FL <= L else 0
    F_FR = -k * (L_FR - L) - c * L_FR_p if L_FR <= L else 0
    F_BL = -k * (L_BL - L) - c * L_BL_p if L_BL <= L else 0
    F_BR = -k * (L_BR - L) - c * L_BR_p if L_BR <= L else 0

    # Update contact flags
    set_n(1 if L_FL <= L else 0)
    set_q(1 if L_FR <= L else 0)
    set_s(1 if L_BL <= L else 0)
    set_p(1 if L_BR <= L else 0)

    # Vertical motion of center of gravity (z_CG'')
    z_CG_pp = (1/m) * (get_n() * F_FL + get_q() * F_FR + get_s() * F_BL + get_p() * F_BR + 32.17389 * z_CG_p**2) - g

    # Angular accelerations (alpha'', beta'')
    alpha_pp = ((w / 2) * (get_n() * F_FL - get_q() * F_FR + get_s() * F_BL - get_p() * F_BR)) / ((1 / 12) * m * w**2)
    beta_pp  = ((l / 2) * (get_n() * F_FL - get_s() * F_BL + get_q() * F_FR - get_p() * F_BR)) / ((1 / 12) * m * l**2)

    return [alpha_p, alpha_pp, beta_p, beta_pp, z_CG_p, z_CG_pp]


# --------------------------------------------------------------------
# Single simulation for a given (k, c) pair
# --------------------------------------------------------------------
def run_simulation(k, c):
    while True:
        # Random initial conditions within bounds
        alpha = random.uniform(-0.2, 0.2)
        beta = random.uniform(-0.2, 0.2)
        alpha_p = random.uniform(-0.1, 0.1)
        beta_p = random.uniform(-0.1, 0.1)
        height = random.uniform(1.9, 2.1)

        x0 = [alpha, alpha_p, beta, beta_p, L + r + height, 0]
        time = np.linspace(0, 3, 1000)

        # Solve ODE system
        solution = odeint(rightSideODE, x0, time, args=(k, c, m, L, r, g, w, l), atol=1e-8, rtol=1e-8)

        alpha_t, beta_t = solution[:, 0], solution[:, 2]
        if np.any(alpha_t < -0.2) or np.any(alpha_t > 0.2) or np.any(beta_t < -0.2) or np.any(beta_t > 0.2):
            continue  # Retry if rotational displacements exceed bounds
        break  # Valid simulation

    z_CG_t, alpha_t, beta_t = solution[:, 4], solution[:, 0], solution[:, 2]
    z_CG_p_t, alpha_p_t, beta_p_t = solution[:, 5], solution[:, 1], solution[:, 3]

    # Get length time series
    L_FL, L_FR, L_BL, L_BR = calculate_lengths(z_CG_t, alpha_t, beta_t, w, l, r)

    # Record time when each spring first compresses past equilibrium
    activation_times = {'L_FL': [], 'L_FR': [], 'L_BL': [], 'L_BR': []}
    for i in range(len(time) - 1):
        if L_FL[i] > L and L_FL[i + 1] <= L: activation_times['L_FL'].append(time[i])
        if L_FR[i] > L and L_FR[i + 1] <= L: activation_times['L_FR'].append(time[i])
        if L_BL[i] > L and L_BL[i + 1] <= L: activation_times['L_BL'].append(time[i])
        if L_BR[i] > L and L_BR[i + 1] <= L: activation_times['L_BR'].append(time[i])

    # Forces at each corner
    F_FL = -k * (z_CG_t + (1/2)*(w * alpha_t - l * beta_t) - L - r) - c * ((1/2) * w * alpha_p_t - (1/2) * l * beta_p_t + z_CG_p_t)
    F_FR = -k * (z_CG_t + (1/2)*(-w * alpha_t - l * beta_t) - L - r) - c * ((1/2) * w * alpha_p_t - (1/2) * l * beta_p_t + z_CG_p_t)
    F_BL = -k * (z_CG_t + (1/2)*(w * alpha_t + l * beta_t) - L - r) - c * ((1/2) * w * alpha_p_t + (1/2) * l * beta_p_t + z_CG_p_t)
    F_BR = -k * (z_CG_t + (1/2)*(-w * alpha_t + l * beta_t) - L - r) - c * (-(1/2) * w * alpha_p_t + (1/2) * l * beta_p_t + z_CG_p_t)

    # Record min/max force after first activation
    min_max_forces = { 'F_FL': {}, 'F_FR': {}, 'F_BL': {}, 'F_BR': {} }
    activation_map = {'L_FL': 'F_FL', 'L_FR': 'F_FR', 'L_BL': 'F_BL', 'L_BR': 'F_BR'}

    for label, times in activation_times.items():
        if times:
            idx = np.where(time >= times[0])[0][0]
            forces = locals()[activation_map[label]][idx:]
            min_max_forces[activation_map[label]]['min'] = np.min(forces)
            min_max_forces[activation_map[label]]['max'] = np.max(forces)

    # Failure if any spring compresses too much
    if np.min([np.min(L_FL), np.min(L_FR), np.min(L_BL), np.min(L_BR)]) < 0.2:
        return "failed", np.min([L_FL, L_FR, L_BL, L_BR]), None

    # Failure if any force exceeds structural limit
    force_limit = 343.35 + 0.3 * k
    max_forces = [min_max_forces[k]['max'] for k in min_max_forces if min_max_forces[k]]
    if any(f > force_limit for f in max_forces):
        return "failed", np.min([L_FL, L_FR, L_BL, L_BR]), np.max(max_forces)

    return "successful", np.min([L_FL, L_FR, L_BL, L_BR]), np.max(max_forces)


# --------------------------------------------------------------------
# Run multiple simulations for given k and c (parallel-safe)
# --------------------------------------------------------------------
def run_simulation_in_parallel(k, c, num_runs):
    success_count, fail_count = 0, 0
    min_lengths, max_forces = [], []

    for _ in range(num_runs):
        result, min_length, max_force = run_simulation(k, c)
        if result == "successful":
            success_count += 1
        else:
            fail_count += 1
        if min_length is not None:
            min_lengths.append(min_length)
        if max_force is not None:
            max_forces.append(max_force)

    return k, c, success_count, fail_count, min_lengths, max_forces


# --------------------------------------------------------------------
# Monte Carlo sweep over k and c to find optimal parameters
# --------------------------------------------------------------------
def monte_carlo_simulation(num_iterations=1000, num_runs_per_combination=10):
    success_fail_ratios = {}
    min_lengths_record = {}
    max_forces_record = {}
    run_counts = {}
    all_results = []

    tasks = []
    with ProcessPoolExecutor() as executor:
        for _ in tqdm(range(num_iterations)):
            k = random.uniform(1000, 10000)
            c = random.uniform(100, 1000)
            task = executor.submit(run_simulation_in_parallel, k, c, num_runs_per_combination)
            tasks.append(task)

        for future in tqdm(as_completed(tasks), total=len(tasks)):
            k, c, success_count, fail_count, min_lengths, max_forces = future.result()
            if fail_count > 0:
                ratio = success_count / fail_count
                success_fail_ratios[(k, c)] = ratio
                min_lengths_record[(k, c)] = np.mean(min_lengths)
                max_forces_record[(k, c)] = np.mean(max_forces)
                run_counts[(k, c)] = (success_count, fail_count)
                all_results.append((k, c, success_count, fail_count))

    # Identify the best-performing (k, c)
    best_kc = max(success_fail_ratios, key=success_fail_ratios.get)
    print(f"Optimal k: {best_kc[0]}, Optimal c: {best_kc[1]}, Success/Fail Ratio: {success_fail_ratios[best_kc]}")
    print(f"Average Min Length: {min_lengths_record[best_kc]}")
    print(f"Average Max Force: {max_forces_record[best_kc]}")
    print(f"Total Successes: {run_counts[best_kc][0]}, Total Fails: {run_counts[best_kc][1]}")
    print(f"Overall Successes: {sum(r[2] for r in all_results)}, Fails: {sum(r[3] for r in all_results)}")

    return all_results


# --------------------------------------------------------------------
# Visualization of simulation results
# --------------------------------------------------------------------
import matplotlib.pyplot as plt

def plot_results(all_results, plot_type='scatter', figsize=(15, 10), **kwargs):
    k_values, c_values, success_counts, _ = zip(*all_results)

    plt.figure(figsize=figsize)
    if plot_type == 'scatter':
        plt.scatter(k_values, c_values, c=success_counts, cmap='viridis', s=20, alpha=0.75, **kwargs)
        plt.colorbar(label='Number of Successful Runs')
    elif plot_type == 'heatmap':
        plt.hexbin(k_values, c_values, gridsize=50, cmap='viridis', C=success_counts, reduce_C_function=np.mean)
        plt.colorbar(label='Mean Number of Successful Runs')

    plt.xlabel('Spring Constant (k)')
    plt.ylabel('Damping Coefficient (c)')
    plt.title('Monte Carlo Simulation: Success Counts')
    plt.show()


# --------------------------------------------------------------------
# Run the Monte Carlo simulation
# --------------------------------------------------------------------
all_results = monte_carlo_simulation(num_iterations=3000, num_runs_per_combination=300)
plot_results(all_results)
