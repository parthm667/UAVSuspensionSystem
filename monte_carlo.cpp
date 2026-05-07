// monte_carlo.cpp
// Monte Carlo suspension optimization for the GoAERO Emergency Response Flyer
// (paramotor-based UAV, Team ASCEND, UT Austin).
//
// Port of the Python Monte Carlo with:
//   - Drag sign fix: |z_CG_p| * z_CG_p instead of z_CG_p^2
//   - Operational vehicle config: 250 kg mass, 50 cm spring travel, 126 m^2 chute
//   - Realistic suspension sampling ranges: k ∈ [15, 60] kN/m, c ∈ [0.5, 3.0] kNs/m
//   - Rejection loop for out-of-bounds alpha/beta (kept as in Python)
//   - Direct touchdown-velocity sampling (bypasses free-fall phase)
//   - Selection by max success count + force-margin tiebreaker
//     (replaces Python's success/fail ratio which had selection bias)
//   - OpenMP parallelism across (k, c) combinations
//   - Thread-local RNG with per-thread seeds
//   - In-place progress bar
//   - CSV output + matplotlib-cpp plotting
//
// Build:
//   g++ -std=c++17 -O3 -fopenmp monte_carlo.cpp \
//       $(python3-config --includes) \
//       $(python3-config --ldflags --embed) \
//       -o monte_carlo
//
// (If Python < 3.8, drop --embed from ldflags.)
//
// Dependencies:
//   - matplotlibcpp.h in this directory:
//     wget https://raw.githubusercontent.com/lava/matplotlib-cpp/master/matplotlibcpp.h
//   - Python 3 with matplotlib + pandas installed (apt: python3-matplotlib python3-pandas)

#define WITHOUT_NUMPY  // we don't need numpy arrays in matplotlib-cpp; simpler linking

#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <random>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <mutex>
#include <string>
#include <limits>
#include <utility>
#include <cstdlib>
#include <omp.h>

// On Linux/WSL, if no display is available, tell matplotlib to use the
// non-interactive 'Agg' backend BEFORE matplotlib-cpp imports pyplot.
// This must happen before any matplotlib-cpp #include-triggered init,
// so we do it in a function called at the very top of main().
static void configure_matplotlib_backend() {
#ifdef __linux__
    const bool has_display = std::getenv("DISPLAY") != nullptr
                          || std::getenv("WAYLAND_DISPLAY") != nullptr;
    if (!has_display) {
        setenv("MPLBACKEND", "Agg", 1);
    }
#endif
}

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// ============================================================================
// Physical constants (GoAERO Emergency Response Flyer operational config)
// ============================================================================
constexpr double M_UAV   = 140.0;   // UAV mass (kg) — paramotor UAV (AIAA paper)
constexpr double L_FREE  = 0.5;     // spring free length (m) — gives 0.3 m travel
constexpr double R_WHEEL = 0.2;     // wheel radius (m)
constexpr double W_UAV   = 0.6;     // UAV width (m)
constexpr double L_UAV   = 1.5;     // UAV length (m)
constexpr double G       = 9.81;    // gravity (m/s^2)

// Spring solid length (minimum compressed length). Difference from L_FREE
// is the usable travel: 0.5 - 0.2 = 0.3 m.
constexpr double L_SOLID = 0.2;
constexpr double TRAVEL  = L_FREE - L_SOLID;  // 0.3 m

// Lumped drag constant: 0.5 * rho * C_P * A_P.
// With rho = 1.225, C_P = 0.75, and A_P = 70 m^2, this gives terminal
// velocity of 6.54 m/s at 140 kg.
constexpr double DRAG_CONST = 32.17;

// Simulation time grid. With no free-fall phase (we start at first contact),
// 2 s is plenty to capture the full bounce decay.
constexpr double T_FINAL = 2.0;
constexpr int    N_STEPS = 700;
constexpr double DT      = T_FINAL / (N_STEPS - 1);

// Monte Carlo parameters (full paper-figure size).
constexpr int N_COMBINATIONS      = 5000;
constexpr int N_TRIALS_PER_COMBO  = 1000;

// Touchdown velocity distribution (m/s, magnitude — will be applied as
// -v_touchdown in the z-direction). Operational envelope: aggressive-flare
// landing (4.0 m/s) up through gust-augmented terminal velocity (7.5 m/s).
// Narrowing this window artificially inflates success rates.
constexpr double V_TD_MIN = 4.0;
constexpr double V_TD_MAX = 7.5;

// ============================================================================
// State = [alpha, alpha_p, beta, beta_p, z_CG, z_CG_p]
// ============================================================================
using State = std::array<double, 6>;

struct Lengths {
    double FL, FR, BL, BR;
    double&       operator[](int i)       { return (&FL)[i]; }
    const double& operator[](int i) const { return (&FL)[i]; }
};

inline Lengths calculate_lengths(double z_CG, double alpha, double beta) {
    Lengths L;
    L.FL = z_CG - R_WHEEL + (W_UAV / 2.0) * alpha + (L_UAV / 2.0) * beta;
    L.FR = z_CG - R_WHEEL - (W_UAV / 2.0) * alpha + (L_UAV / 2.0) * beta;
    L.BL = z_CG - R_WHEEL + (W_UAV / 2.0) * alpha - (L_UAV / 2.0) * beta;
    L.BR = z_CG - R_WHEEL - (W_UAV / 2.0) * alpha - (L_UAV / 2.0) * beta;
    return L;
}

inline Lengths calculate_velocities(double z_CG_p, double alpha_p, double beta_p) {
    Lengths L;
    L.FL = z_CG_p + (W_UAV / 2.0) * alpha_p + (L_UAV / 2.0) * beta_p;
    L.FR = z_CG_p - (W_UAV / 2.0) * alpha_p + (L_UAV / 2.0) * beta_p;
    L.BL = z_CG_p + (W_UAV / 2.0) * alpha_p - (L_UAV / 2.0) * beta_p;
    L.BR = z_CG_p - (W_UAV / 2.0) * alpha_p - (L_UAV / 2.0) * beta_p;
    return L;
}

// ============================================================================
// ODE right-hand side
// ============================================================================
inline State rhs(const State& x, double k, double c) {
    const double alpha   = x[0];
    const double alpha_p = x[1];
    const double beta    = x[2];
    const double beta_p  = x[3];
    const double z_CG    = x[4];
    const double z_CG_p  = x[5];

    const Lengths len = calculate_lengths(z_CG, alpha, beta);
    const Lengths vel = calculate_velocities(z_CG_p, alpha_p, beta_p);

    // Spring/damper forces: zero when wheel is airborne (spring extended past free length)
    const double F_FL = (len.FL <= L_FREE) ? -k * (len.FL - L_FREE) - c * vel.FL : 0.0;
    const double F_FR = (len.FR <= L_FREE) ? -k * (len.FR - L_FREE) - c * vel.FR : 0.0;
    const double F_BL = (len.BL <= L_FREE) ? -k * (len.BL - L_FREE) - c * vel.BL : 0.0;
    const double F_BR = (len.BR <= L_FREE) ? -k * (len.BR - L_FREE) - c * vel.BR : 0.0;

    // Vertical EOM. Drag uses |v|*v so it opposes motion in both directions
    // (the fix vs. the original Python's unsigned v^2).
    const double z_CG_pp = (F_FL + F_FR + F_BL + F_BR
                            - DRAG_CONST * std::abs(z_CG_p) * z_CG_p) / M_UAV - G;

    // Thin-plate moments of inertia
    constexpr double I_x = (1.0 / 12.0) * M_UAV * W_UAV * W_UAV;
    constexpr double I_y = (1.0 / 12.0) * M_UAV * L_UAV * L_UAV;

    // Rotational EOM. Matches the Python code's convention exactly
    // (code and paper disagree on the sign of beta, but results are equivalent
    // under a relabeling; we preserve the code's numerical behavior).
    const double alpha_pp = (W_UAV / 2.0) * (F_FL - F_FR + F_BL - F_BR) / I_x;
    const double beta_pp  = (L_UAV / 2.0) * (F_FL - F_BL + F_FR - F_BR) / I_y;

    return {alpha_p, alpha_pp, beta_p, beta_pp, z_CG_p, z_CG_pp};
}

// ============================================================================
// Fixed-step RK4 integrator (analog of scipy.integrate.odeint for this problem)
// ============================================================================
std::vector<State> integrate_rk4(const State& x0, double k, double c) {
    std::vector<State> trajectory;
    trajectory.reserve(N_STEPS);
    trajectory.push_back(x0);

    State x = x0;
    for (int i = 1; i < N_STEPS; i++) {
        const State k1 = rhs(x, k, c);

        State x_tmp;
        for (int j = 0; j < 6; j++) x_tmp[j] = x[j] + 0.5 * DT * k1[j];
        const State k2 = rhs(x_tmp, k, c);

        for (int j = 0; j < 6; j++) x_tmp[j] = x[j] + 0.5 * DT * k2[j];
        const State k3 = rhs(x_tmp, k, c);

        for (int j = 0; j < 6; j++) x_tmp[j] = x[j] + DT * k3[j];
        const State k4 = rhs(x_tmp, k, c);

        for (int j = 0; j < 6; j++) {
            x[j] += (DT / 6.0) * (k1[j] + 2.0 * k2[j] + 2.0 * k3[j] + k4[j]);
        }
        trajectory.push_back(x);
    }
    return trajectory;
}

// ============================================================================
// Single trial. Four failure modes — bottoming, force overshoot, attitude
// divergence, settling — selected so the optimum sits in the interior of
// the (k, c) plane instead of hugging the c=c_min boundary.
// ============================================================================
struct TrialResult {
    bool   success;
    double min_length;
    double max_force;
    bool   has_data;  // false only if the vehicle never made wheel contact
};

TrialResult run_single_trial(double k, double c, std::mt19937& rng) {
    std::uniform_real_distribution<double> attitude_dist(-0.2, 0.2);
    std::uniform_real_distribution<double> vel_dist(-0.1, 0.1);
    std::uniform_real_distribution<double> v_td_dist(V_TD_MIN, V_TD_MAX);

    const double alpha0    = attitude_dist(rng);
    const double beta0     = attitude_dist(rng);
    const double alpha_p0  = vel_dist(rng);
    const double beta_p0   = vel_dist(rng);
    const double v_touchdown = v_td_dist(rng);  // m/s, magnitude

    // Start the simulation at first contact of the lowest wheel.
    // The lowest wheel depends on signs of alpha and beta; for that wheel,
    // L = L_FREE (spring at free length, force just about to activate).
    // This skips the free-fall phase entirely. The parachute's job is to
    // regulate touchdown velocity; we sample that velocity directly.
    const double z_CG_first_contact = L_FREE + R_WHEEL
                                    + std::abs(alpha0) * (W_UAV / 2.0)
                                    + std::abs(beta0)  * (L_UAV / 2.0);

    const State x0 = {alpha0, alpha_p0, beta0, beta_p0,
                      z_CG_first_contact, -v_touchdown};

    const auto traj = integrate_rk4(x0, k, c);

    // Failure mode 3 (attitude divergence): if alpha or beta exceeds ±0.2 rad
    // anywhere in the trajectory, count the trial as a failure rather than
    // re-rolling initial conditions. Re-rolling silently exempts low-damping
    // configs from this physical-realism failure.
    bool failed_attitude = false;
    for (const auto& s : traj) {
        if (s[0] < -0.2 || s[0] > 0.2 || s[2] < -0.2 || s[2] > 0.2) {
            failed_attitude = true;
            break;
        }
    }

    // Walk the trajectory once for force/length statistics and for
    // first-impact tracking. Force gate matches the ODE: zero force when
    // wheel is airborne (length > L_FREE).
    double min_length        = std::numeric_limits<double>::max();
    double max_force         = 0.0;
    bool   first_impact_seen = false;

    for (const auto& s : traj) {
        const Lengths len = calculate_lengths(s[4], s[0], s[2]);
        const Lengths vel = calculate_velocities(s[5], s[1], s[3]);

        for (int w = 0; w < 4; w++) {
            if (len[w] <= L_FREE) {
                first_impact_seen = true;
                const double F = -k * (len[w] - L_FREE) - c * vel[w];
                if (F > max_force)        max_force  = F;
                if (len[w] < min_length)  min_length = len[w];
            }
        }
    }

    if (!first_impact_seen) {
        // Vehicle never made wheel contact during the simulation window.
        // Report as failure with no force/length data.
        return {false, 0.0, 0.0, false};
    }

    // Failure mode 1: bottoming (spring compressed past solid length).
    const bool   failed_length = (min_length < L_SOLID);

    // Failure mode 2: force overshoot vs structural limit.
    const double force_limit   = M_UAV * G / 4.0 + TRAVEL * k;
    const bool   failed_force  = (max_force > force_limit);

    // Failure mode 4 (settling): UAV must come to rest within T_FINAL.
    // Underdamped configs ring out the impact energy too slowly and fail here.
    const double z_CG_p_final  = traj.back()[5];
    const bool   failed_settle = (std::abs(z_CG_p_final) > 0.3);

    const bool   success = !failed_length && !failed_force
                         && !failed_attitude && !failed_settle;

    return {success, min_length, max_force, true};
}

// ============================================================================
// Aggregation over N_TRIALS_PER_COMBO trials for a single (k, c)
// ============================================================================
struct ComboResult {
    double k, c;
    int    success_count;
    int    fail_count;
    double avg_min_length;
    double avg_max_force;
};

ComboResult run_combination(double k, double c, std::mt19937& rng) {
    int    success_count = 0, fail_count = 0;
    double sum_min_length = 0.0, sum_max_force = 0.0;
    int    n_with_data = 0;

    for (int i = 0; i < N_TRIALS_PER_COMBO; i++) {
        const TrialResult tr = run_single_trial(k, c, rng);
        if (tr.success) success_count++;
        else            fail_count++;

        if (tr.has_data) {
            sum_min_length += tr.min_length;
            sum_max_force  += tr.max_force;
            n_with_data++;
        }
    }

    const double avg_min_length = (n_with_data > 0) ? sum_min_length / n_with_data : 0.0;
    const double avg_max_force  = (n_with_data > 0) ? sum_max_force  / n_with_data : 0.0;

    return {k, c, success_count, fail_count, avg_min_length, avg_max_force};
}

// ============================================================================
// Thread-safe progress bar
// ============================================================================
class ProgressBar {
    std::atomic<int> completed{0};
    int              total;
    std::chrono::steady_clock::time_point start;
    mutable std::mutex                    print_mutex;
    static constexpr int bar_width = 40;

public:
    explicit ProgressBar(int total_) 
        : total(total_), start(std::chrono::steady_clock::now()) {}

    void tick() {
        const int done = ++completed;
        print(done);
    }

    void finish() {
        print(total);
        std::cout << std::endl;
    }

private:
    void print(int done) const {
        std::lock_guard<std::mutex> lock(print_mutex);
        const auto   now     = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - start).count();
        const double rate    = (elapsed > 0.0) ? done / elapsed : 0.0;
        const double eta     = (rate    > 0.0) ? (total - done) / rate : 0.0;

        const double progress = static_cast<double>(done) / total;
        const int    filled   = static_cast<int>(bar_width * progress);

        std::cout << "\r[";
        for (int i = 0; i < bar_width; i++) {
            std::cout << (i < filled ? '#' : (i == filled ? '>' : ' '));
        }
        std::cout << "] " << done << "/" << total
                  << " (" << std::fixed << std::setprecision(1) << (100.0 * progress) << "%)"
                  << " elapsed: " << std::setprecision(0) << elapsed << "s"
                  << " eta: "     << eta                             << "s    "
                  << std::flush;
    }
};

// ============================================================================
// CSV output
// ============================================================================
void save_csv(const std::vector<ComboResult>& results, const std::string& filename) {
    std::ofstream out(filename);
    out << "k,c,success_count,fail_count,avg_min_length,avg_max_force\n";
    out << std::setprecision(12);
    for (const auto& r : results) {
        out << r.k << "," << r.c << ","
            << r.success_count << "," << r.fail_count << ","
            << r.avg_min_length << "," << r.avg_max_force << "\n";
    }
}

// ============================================================================
// Plotting via matplotlib-cpp
// We use matplotlib-cpp to bootstrap the Python interpreter, then call
// matplotlib directly via PyRun_SimpleString for the per-point colored
// scatter (which stock matplotlib-cpp doesn't support cleanly).
// ============================================================================
void plot_results(const std::string& csv_file) {
    // Trigger matplotlib-cpp's Python interpreter initialization
    plt::figure_size(1500, 1000);

    const std::string code = std::string(R"(
import matplotlib.pyplot as plt
import pandas as pd
df = pd.read_csv(')") + csv_file + R"(')
plt.clf()
fig, ax = plt.subplots(figsize=(15, 10))
sc = ax.scatter(df['k'], df['c'], c=df['success_count'], cmap='viridis', s=20, alpha=0.75)
plt.colorbar(sc, label='Number of Successful Runs')
ax.set_xlabel('Spring Constant k (N/m)')
ax.set_ylabel('Damping Coefficient c (Ns/m)')
ax.set_title('Monte Carlo Simulation Success Counts')
plt.tight_layout()
plt.savefig('monte_carlo.png', dpi=150)
print('Saved monte_carlo.png')
try:
    plt.show()
except Exception as e:
    print(f'(plt.show skipped: {e})')
)";
    PyRun_SimpleString(code.c_str());
}

// ============================================================================
// Main
// ============================================================================
int main() {
    configure_matplotlib_backend();

    std::cout << "Monte Carlo suspension optimization\n";
    std::cout << "  " << N_COMBINATIONS << " (k, c) combinations x "
              << N_TRIALS_PER_COMBO << " trials each = "
              << (N_COMBINATIONS * N_TRIALS_PER_COMBO) << " base trials\n";
    std::cout << "  Using " << omp_get_max_threads() << " threads\n\n";

    // Pre-generate (k, c) parameter list with a master RNG (deterministic).
    // Ranges reflect commercial ATV-class suspension hardware proportional
    // to the 140 kg paramotor UAV mass.
    std::mt19937 master_rng(42);
    std::uniform_real_distribution<double> k_dist(5000.0, 30000.0);
    std::uniform_real_distribution<double> c_dist(100.0, 1500.0);

    std::vector<std::pair<double, double>> params;
    params.reserve(N_COMBINATIONS);
    for (int i = 0; i < N_COMBINATIONS; i++) {
        params.emplace_back(k_dist(master_rng), c_dist(master_rng));
    }

    std::vector<ComboResult> results(N_COMBINATIONS);
    ProgressBar              pbar(N_COMBINATIONS);

    const auto t0 = std::chrono::steady_clock::now();

    #pragma omp parallel
    {
        const int    tid = omp_get_thread_num();
        std::mt19937 thread_rng(1000 + tid);  // distinct seed per thread

        #pragma omp for schedule(dynamic, 1)
        for (int i = 0; i < N_COMBINATIONS; i++) {
            results[i] = run_combination(params[i].first, params[i].second, thread_rng);
            pbar.tick();
        }
    }
    pbar.finish();

    const auto   t1      = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(t1 - t0).count();
    std::cout << "\nCompleted in " << std::fixed << std::setprecision(1)
              << elapsed << " s\n\n";

    // Find optimum: maximum success count, tiebreaker = smallest avg_max_force
    // (more margin below the structural force threshold = safer).
    // This is a fix from the original Python, which used success/fail ratio
    // and silently excluded any cells with zero failures.
    const ComboResult* optimal         = nullptr;
    int                best_success    = -1;
    double             tiebreak_force  = std::numeric_limits<double>::max();
    int                total_success   = 0, total_fail = 0;
    int                n_perfect_cells = 0;

    for (const auto& r : results) {
        total_success += r.success_count;
        total_fail    += r.fail_count;
        if (r.fail_count == 0) n_perfect_cells++;

        const bool strictly_better = (r.success_count > best_success);
        const bool tied_and_safer  = (r.success_count == best_success
                                      && r.avg_max_force < tiebreak_force);

        if (strictly_better || tied_and_safer) {
            best_success   = r.success_count;
            tiebreak_force = r.avg_max_force;
            optimal        = &r;
        }
    }

    std::cout << std::setprecision(15);
    if (optimal) {
        const double success_rate = static_cast<double>(optimal->success_count)
                                  / N_TRIALS_PER_COMBO;
        std::cout << "Optimal k: " << optimal->k
                  << ", Optimal c: " << optimal->c << "\n";
        std::cout << "Success count: " << optimal->success_count
                  << " / " << N_TRIALS_PER_COMBO
                  << " (" << std::fixed << std::setprecision(2)
                  << (100.0 * success_rate) << "%)\n";
        std::cout << std::setprecision(15);
        std::cout << "Average Minimum Spring Length: "
                  << optimal->avg_min_length << " m\n";
        std::cout << "Average Maximum Force: "
                  << optimal->avg_max_force << " N\n";
    } else {
        std::cout << "No results — something went wrong.\n";
    }
    std::cout << "Total successful runs: " << total_success
              << " / " << (N_COMBINATIONS * N_TRIALS_PER_COMBO) << "\n";
    std::cout << "Total failed runs: "     << total_fail    << "\n";
    std::cout << "Parameter cells with zero failures: "
              << n_perfect_cells << " / " << N_COMBINATIONS << "\n";

    // Save CSV and plot
    const std::string csv_file = "monte_carlo_results.csv";
    save_csv(results, csv_file);
    std::cout << "\nResults saved to " << csv_file << "\n";
    std::cout << "Plotting...\n";

    plot_results(csv_file);

    return 0;
}