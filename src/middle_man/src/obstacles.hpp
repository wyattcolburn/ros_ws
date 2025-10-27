#ifndef OBSTACLES_H
#define OBSTACLES_H
#include "local_goal.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <utility>
#include <vector>
// parameters from config.yaml

const int MAX_OBSTACLES = 100;

class Obstacle {
  public:
    double center_x, center_y, radius;
};
// function prototypes
std::pair<Obstacle, Obstacle> create_obstacle(double current_lg_x, double current_lg_y, double next_lg_x,
                                              double next_lg_y, float OFFSET, float RADIUS);
struct ObstacleManager {

    Obstacle obstacle_array[MAX_OBSTACLES];
    bool is_active[MAX_OBSTACLES];
    float RADIUS;
    int NUM_VALID_OBSTACLES;
    float OFFSET;
    int obstacle_count; // num of obstacles current in array
    ObstacleManager() : RADIUS(0.4f), NUM_VALID_OBSTACLES(20), OFFSET(1.0f), obstacle_count(0) {
        for (int i = 0; i < MAX_OBSTACLES; i++) {
            is_active[i] = false;
        }
    }

    void set_params(float radius, int num_valid, float offset) {
        // init values from config.yaml
        RADIUS = radius;
        NUM_VALID_OBSTACLES = num_valid;
        OFFSET = offset;
    }
    // function for creating obstacles
    int add_obstacle(const Obstacle &obs) {
        if (obstacle_count >= MAX_OBSTACLES)
            return -1;

        for (int i = 0; i < MAX_OBSTACLES; i++) {

            if (!is_active[i]) {
                obstacle_array[i] = obs;
                is_active[i] = true;
                obstacle_count++;
                return 1;
            }
        }
        return -1; // no space
    }

    void local_goals_to_obs(Local_Goal_Manager &local_manager_) {
        // does not work
        std::vector<Local_Goal> local_goal_vec;
        local_goal_vec = local_manager_.getLocalGoalVector();

        for (int lg_counter = 0; lg_counter < static_cast<int>(local_goal_vec.size() - 1); lg_counter++) {

            if (lg_counter + 1 > local_goal_vec.size()) {
                return;
            }
            Local_Goal current_lg = local_goal_vec[lg_counter];
            Local_Goal next_lg = local_goal_vec[lg_counter + 1];

            std::pair<Obstacle, Obstacle> obs_pair = create_obstacle(current_lg.x_point, current_lg.y_point,
                                                                     next_lg.x_point, next_lg.y_point, OFFSET, RADIUS);
            add_obstacle(obs_pair.first);
            add_obstacle(obs_pair.second);

            std::cout << "Obstacle at " << obs_pair.first.center_x << "  " << obs_pair.first.center_y << std::endl;
        }
        return;
    }
    void update_obstacles(Local_Goal_Manager &local_manager_) {
        if (obstacle_count <= 0)
            return;

        // Current segment index (which local goal we're on)
        int g = local_manager_.get_local_goal_counter();

        // Map segment index -> obstacle index range (2 obstacles per segment)
        int start = 2 * g; // first obstacle for this segment
        int end = std::min(start + NUM_VALID_OBSTACLES - 1, obstacle_count - 1);

        // Log (obstacle indices, not goal indices)
        // std::cout << "goal/segment index: " << g << "  -> active obstacle window: [" << start << ", " << end << "]
        // "
        // << obstacle_count << std::endl;

        for (int i = 0; i < obstacle_count; ++i) {
            is_active[i] = (i >= start && i <= end);
        }
    }

    void update_obstacles_sliding(Local_Goal_Manager &local_manager_) {

        if (obstacle_count <= 0)
            return;

        int g = local_manager_.get_local_goal_counter();
        int start_offset = NUM_VALID_OBSTACLES / 4;
        int start = 2 * g - start_offset;
        int end = std::min(start + NUM_VALID_OBSTACLES - 1, obstacle_count - 1);

        for (int i = 0; i < obstacle_count; ++i) {
            is_active[i] = (i >= start && i <= end);
        }
    }
    // int g = local_manager_.get_local_goal_counter();
    // int start_offset = NUM_VALID_OBSTACLES / 4;
    // int start = std::max(0, 2 * g - start_offset);
    // start = std::min(start, std::max(0, obstacle_count - 1));
    // int end = std::min(start + NUM_VALID_OBSTACLES - 1, obstacle_count - 1);
    //
    // for (int i = 0; i < obstacle_count; ++i) {
    //     is_active[i] = (i >= start && i <= end);
    // }

    // Get array of active obstacles for passing to functions
    const Obstacle *get_active_obstacles(int &out_count) {
        static Obstacle active_obs[MAX_OBSTACLES];
        out_count = 0;

        for (int i = 0; i < MAX_OBSTACLES; i++) {
            if (is_active[i]) {
                active_obs[out_count++] = obstacle_array[i];
            }
        }

        return active_obs;
    }

    void clean_data() {
        // new path so need to make new obstacles
        for (int i = 0; i < MAX_OBSTACLES; i++) {
            is_active[i] = false;
        }
        obstacle_count = 0;
        memset(obstacle_array, 0, sizeof(obstacle_array));
        return;
    }
};

#endif
//
//
// #ifndef OBSTACLES_HPP
// #define OBSTACLES_HPP
//
// #include "local_goal.hpp"
//
// #include <algorithm>
// #include <cmath>
// #include <cstring>
// #include <limits>
// #include <random>
// #include <utility>
// #include <vector>
//
// // ========================= config =========================
// #ifndef MAX_OBSTACLES
// #define MAX_OBSTACLES 100
// #endif
//
// // ========================= types ==========================
// struct Obstacle {
//     double center_x{0.0};
//     double center_y{0.0};
//     double radius{0.0};
// };
//
// // Simple symmetric pair (your old runtime behavior)
// inline std::pair<Obstacle, Obstacle> create_obstacle(double current_lg_x, double current_lg_y, double next_lg_x,
//                                                      double next_lg_y, float OFFSET, float RADIUS) {
//     double dx = next_lg_x - current_lg_x;
//     double dy = next_lg_y - current_lg_y;
//     double L = std::hypot(dx, dy);
//     if (L < 1e-12) {
//         return {Obstacle{current_lg_x, current_lg_y, RADIUS}, Obstacle{current_lg_x, current_lg_y, RADIUS}};
//     }
//     dx /= L;
//     dy /= L;
//     const double px = -dy, py = dx; // left-hand normal
//     const double mx = 0.5 * (current_lg_x + next_lg_x);
//     const double my = 0.5 * (current_lg_y + next_lg_y);
//     Obstacle in{mx + OFFSET * px, my + OFFSET * py, RADIUS};
//     Obstacle out{mx - OFFSET * px, my - OFFSET * py, RADIUS};
//     return {in, out};
// }
//
// // ================= generator params (matches training) =================
// struct ObstacleGenParams {
//     // Required by your class __init__(..., OFFSET, RADIUS, ...)
//     float OFFSET{1.0f}; // baseline half-width
//     float RADIUS{0.4f}; // base obstacle radius
//
//     // realism knobs from training
//     float offset_lo{0.85f}, offset_hi{1.30f};           // width multiplier per segment
//     float radius_lo{0.90f}, radius_hi{1.15f};           // radius multiplier per obstacle
//     float jitter_std{0.05f};                            // XY noise [m]
//     float along_std{0.04f};                             // along-track slide [m]
//     float drop_prob{0.08f};                             // drop obstacle (gaps)
//     float single_side_prob{0.12f};                      // sometimes only one side
//     float pinch_prob{0.06f};                            // extra narrowing event
//     float pinch_scale_lo{0.45f}, pinch_scale_hi{0.75f}; // pinch multiplier for width
//     float clutter_prob{0.04f};                          // add free obstacle near corridor
//     float clutter_rad{1.20f};                           // max radius from midpoint
//
//     // curvature → skew (asymmetric widening/shrinking)
//     float skew_s_max{0.50f}; // max fractional skew 0..1
//     float skew_gain{4.0f};   // curvature→skew growth
//     float widen_out{0.70f};  // outside widening multiplier vs inside shrinking
//     float min_inside{0.40f}; // floor for inside (fraction of BASE)
//     float max_outmul{1.80f}; // cap for outside (multiple of BASE)
//
//     // clearance from path (kept from earlier training snippet)
//     float clearance_scale{1.20f};  // ≥ 1.1–1.2 × radius
//     float clearance_buffer{0.03f}; // [m]
//
//     // clean_mode behavior from your code
//     bool clean_mode{true}; // default matches your __init__(..., clean_mode=True)
//     // determinism
//     uint32_t rng_seed{42}; // seed from your training snippet
// };
//
// // RNG helper
// struct Rand {
//     std::mt19937 gen;
//     std::uniform_real_distribution<double> U{0.0, 1.0};
//     std::normal_distribution<double> N{0.0, 1.0};
//     explicit Rand(uint32_t seed = 42) : gen(seed) {}
//     inline double uniform() { return U(gen); }
//     inline bool coin(double p) { return U(gen) < p; }
//     inline double normal(double std) { return N(gen) * std; }
// };
//
// // ================= helpers: curvature & polyline distance ==============
// inline double signed_curvature(const std::vector<Local_Goal> &gp, int i) {
//     if (gp.empty())
//         return 0.0;
//     const int n = static_cast<int>(gp.size());
//     const int i0 = std::max(0, i - 1);
//     const int i1 = std::clamp(i, 0, n - 1);
//     const int i2 = std::min(n - 1, i + 1);
//
//     const double x0 = gp[i0].x_point, y0 = gp[i0].y_point;
//     const double x1 = gp[i1].x_point, y1 = gp[i1].y_point;
//     const double x2 = gp[i2].x_point, y2 = gp[i2].y_point;
//
//     double ax = x1 - x0, ay = y1 - y0;
//     double bx = x2 - x1, by = y2 - y1;
//     const double a = std::hypot(ax, ay);
//     const double b = std::hypot(bx, by);
//     if (a < 1e-9 || b < 1e-9)
//         return 0.0;
//
//     ax /= a;
//     ay /= a;
//     bx /= b;
//     by /= b;
//
//     const double vx = ax + bx;
//     const double vy = ay + by;
//     const double v = std::hypot(vx, vy);
//     if (v < 1e-9)
//         return 0.0;
//
//     const double cross = (ax * by - ay * bx); // +left, -right
//     return 2.0 * cross / v;
// }
//
// inline double min_distance_to_polyline(double px, double py, const std::vector<Local_Goal> &gp) {
//     double best = std::numeric_limits<double>::infinity();
//     for (size_t i = 0; i + 1 < gp.size(); ++i) {
//         const double x1 = gp[i].x_point, y1 = gp[i].y_point;
//         const double x2 = gp[i + 1].x_point, y2 = gp[i + 1].y_point;
//         const double vx = x2 - x1, vy = y2 - y1;
//         const double L2 = vx * vx + vy * vy;
//         double t = (L2 > 1e-12) ? ((px - x1) * vx + (py - y1) * vy) / L2 : 0.0;
//         t = std::clamp(t, 0.0, 1.0);
//         const double cx = x1 + t * vx;
//         const double cy = y1 + t * vy;
//         best = std::min(best, std::hypot(px - cx, py - cy));
//     }
//     return best;
// }
//
// // ===================== training-style creation =========================
// inline std::vector<Obstacle> create_obstacles_like_training(const Local_Goal &current_lg, const Local_Goal &next_lg,
//                                                             const std::vector<Local_Goal> &global_path,
//                                                             int seg_index, // segment index for curvature
//                                                             const ObstacleGenParams &Pin, Rand &R) {
//     // Apply clean_mode overrides like your Python __init__
//     ObstacleGenParams P = Pin;
//     if (P.clean_mode) {
//         P.offset_lo = P.offset_hi = 1.0f;
//         P.radius_lo = P.radius_hi = 1.0f;
//         P.jitter_std = 0.0f;
//         P.along_std = 0.0f;
//         P.drop_prob = 0.0f;
//         P.single_side_prob = 0.0f;
//         P.pinch_prob = 0.0f;
//         P.clutter_prob = 0.0f;
//         // also reduce curvature skew (your code sets these, not zero)
//         P.skew_s_max = 0.3f;
//         P.widen_out = 0.5f;
//     }
//
//     std::vector<Obstacle> out;
//
//     // Segment frame
//     double x1 = current_lg.x_point, y1 = current_lg.y_point;
//     double x2 = next_lg.x_point, y2 = next_lg.y_point;
//     double dx = x2 - x1, dy = y2 - y1;
//     double L = std::hypot(dx, dy);
//     if (L < 1e-9)
//         return out;
//
//     dx /= L;
//     dy /= L;
//     const double px = -dy, py = dx; // left normal
//     const double mx = 0.5 * (x1 + x2);
//     const double my = 0.5 * (y1 + y2);
//
//     // Width & pinch
//     const double width_mul = P.offset_lo + (P.offset_hi - P.offset_lo) * R.uniform();
//     double BASE = P.OFFSET * width_mul;
//
//     if (R.coin(P.pinch_prob)) {
//         const double pinch = P.pinch_scale_lo + (P.pinch_scale_hi - P.pinch_scale_lo) * R.uniform();
//         BASE *= pinch; // narrow corridor event
//     }
//
//     // Curvature → skew
//     const double kappa = signed_curvature(global_path, std::clamp(seg_index, 1, (int)global_path.size() - 2));
//     const double skew = P.skew_s_max * (1.0 - std::exp(-P.skew_gain * std::fabs(kappa)));
//
//     // Inside/outside offsets
//     const double off_in = std::max((double)P.min_inside * BASE, BASE * (1.0 - skew));
//     const double off_out = std::min((double)P.max_outmul * BASE, BASE * (1.0 + P.widen_out * skew));
//
//     const int inside_s = (kappa > 0.0) ? +1 : -1; // +left
//     const int outside_s = -inside_s;
//
//     // Radii
//     auto sample_radius = [&]() -> double {
//         const double f = P.radius_lo + (P.radius_hi - P.radius_lo) * R.uniform();
//         return P.RADIUS * f;
//     };
//     const double rad_in = sample_radius();
//     const double rad_out = sample_radius();
//
//     // Jitter/along
//     auto jittered = [&](double bx, double by) -> std::pair<double, double> {
//         const double jx = R.normal(P.jitter_std);
//         const double jy = R.normal(P.jitter_std);
//         const double along = R.normal(P.along_std);
//         return {bx + jx + along * dx, by + jy + along * dy};
//     };
//
//     auto [cx_in0, cy_in0] = jittered(mx + inside_s * off_in * px, my + inside_s * off_in * py);
//     auto [cx_out0, cy_out0] = jittered(mx + outside_s * off_out * px, my + outside_s * off_out * py);
//
//     // Clearance enforcement (push outward along normal if too close)
//     auto enforce_clearance = [&](double cx, double cy, double radius) -> std::pair<double, double> {
//         const double min_clear = std::max(radius * 1.10, (double)P.RADIUS * P.clearance_scale) + P.clearance_buffer;
//         const double d = min_distance_to_polyline(cx, cy, global_path);
//         const double need = min_clear - d;
//         if (need > 1e-9) {
//             const double s = ((px * (cx - mx) + py * (cy - my)) >= 0.0) ? +1.0 : -1.0;
//             cx += s * need * px;
//             cy += s * need * py;
//         }
//         return {cx, cy};
//     };
//
//     std::tie(cx_in0, cy_in0) = enforce_clearance(cx_in0, cy_in0, rad_in);
//     std::tie(cx_out0, cy_out0) = enforce_clearance(cx_out0, cy_out0, rad_out);
//
//     // Single-side option
//     bool single_side = R.coin(P.single_side_prob);
//
//     // Dropouts per side (if not single-side override)
//     bool keep_in = true;
//     bool keep_out = true;
//
//     if (single_side) {
//         // keep exactly one side
//         if (R.coin(0.5))
//             keep_out = false;
//         else
//             keep_in = false;
//     } else {
//         // independent dropout like training
//         if (R.coin(P.drop_prob * 0.5))
//             keep_in = false;
//         if (R.coin(P.drop_prob))
//             keep_out = false;
//     }
//
//     if (keep_in)
//         out.push_back(Obstacle{cx_in0, cy_in0, rad_in});
//     if (keep_out)
//         out.push_back(Obstacle{cx_out0, cy_out0, rad_out});
//
//     // Occasional clutter near corridor midpoint
//     if (R.coin(P.clutter_prob)) {
//         static constexpr double kTwoPi = 6.28318530717958647692;
//         const double ang = kTwoPi * R.uniform();
//         const double rad = P.clutter_rad * R.uniform(); // within radius
//         const double rx = mx + rad * std::cos(ang);
//         const double ry = my + rad * std::sin(ang);
//         out.push_back(Obstacle{rx, ry, sample_radius()});
//     }
//
//     return out;
// }
//
// // ============================= manager ================================
// struct ObstacleManager {
//     // storage
//     Obstacle obstacle_array[MAX_OBSTACLES]{};
//     bool is_active[MAX_OBSTACLES]{};
//     int obstacle_count{0};
//
//     // segment→obstacle index bookkeeping (robust to variable counts)
//     std::vector<int> seg_start; // starting index of each segment's obstacles
//     std::vector<int> seg_count; // how many obstacles that segment produced
//
//     // public params for legacy API
//     float RADIUS{0.40f};
//     int NUM_VALID_OBSTACLES{20}; // window size in obstacle count
//     float OFFSET{1.0f};
//
//     // generator params & RNG
//     ObstacleGenParams gen_params{};
//     Rand rng{gen_params.rng_seed};
//
//     ObstacleManager() { std::fill(std::begin(is_active), std::end(is_active), false); }
//
//     void set_params(float radius, int num_valid, float offset) {
//         RADIUS = radius;
//         NUM_VALID_OBSTACLES = std::max(0, num_valid);
//         OFFSET = offset;
//         gen_params.RADIUS = RADIUS;
//         gen_params.OFFSET = OFFSET;
//     }
//     void set_generator_params(const ObstacleGenParams &p) {
//         gen_params = p;
//         // mirror core into legacy fields
//         RADIUS = gen_params.RADIUS;
//         OFFSET = gen_params.OFFSET;
//         rng = Rand(gen_params.rng_seed);
//     }
//
//     // ------------ building ------------
//     int add_obstacle(const Obstacle &obs) {
//         if (obstacle_count >= MAX_OBSTACLES)
//             return -1;
//         obstacle_array[obstacle_count] = obs;
//         is_active[obstacle_count] = false; // window decides activation
//         ++obstacle_count;
//         return 1;
//     }
//
//     // Old symmetric behavior
//     void local_goals_to_obs_symmetric(Local_Goal_Manager &local_manager_) {
//         clean_data();
//         std::vector<Local_Goal> gp = local_manager_.getLocalGoalVector();
//         const int N = static_cast<int>(gp.size());
//         if (N < 2)
//             return;
//
//         seg_start.clear();
//         seg_count.clear();
//         for (int i = 0; i < N - 1 && obstacle_count < MAX_OBSTACLES; ++i) {
//             const auto &cur = gp[i];
//             const auto &nxt = gp[i + 1];
//             int start_idx = obstacle_count;
//             auto pair = create_obstacle(cur.x_point, cur.y_point, nxt.x_point, nxt.y_point, OFFSET, RADIUS);
//             if (obstacle_count < MAX_OBSTACLES)
//                 add_obstacle(pair.first);
//             if (obstacle_count < MAX_OBSTACLES)
//                 add_obstacle(pair.second);
//             seg_start.push_back(start_idx);
//             seg_count.push_back(std::min(2, MAX_OBSTACLES - start_idx));
//         }
//     }
//
//     // Training-style behavior (matches your Python)
//     void local_goals_to_obs_like_training(Local_Goal_Manager &local_manager_) {
//         clean_data();
//         std::vector<Local_Goal> gp = local_manager_.getLocalGoalVector();
//         const int N = static_cast<int>(gp.size());
//         if (N < 2)
//             return;
//
//         rng = Rand(gen_params.rng_seed); // reset for determinism (optional)
//         seg_start.clear();
//         seg_count.clear();
//         seg_start.reserve(N - 1);
//         seg_count.reserve(N - 1);
//
//         for (int i = 0; i < N - 1 && obstacle_count < MAX_OBSTACLES; ++i) {
//             int start_idx = obstacle_count;
//
//             auto obs = create_obstacles_like_training(gp[i], gp[i + 1], gp, std::max(1, i), gen_params, rng);
//             for (const auto &o : obs) {
//                 if (obstacle_count >= MAX_OBSTACLES)
//                     break;
//                 add_obstacle(o);
//             }
//             seg_start.push_back(start_idx);
//             seg_count.push_back(obstacle_count - start_idx);
//         }
//     }
//
//     // ------------ windowing ------------
//     // Fixed window: start at current segment's first obstacle, take NUM_VALID_OBSTACLES
//     void update_obstacles(Local_Goal_Manager &local_manager_) {
//         if (obstacle_count <= 0)
//             return;
//         const int g = std::max(0, local_manager_.get_local_goal_counter());
//         if (seg_start.empty()) { // fallback: linear range
//             int start = std::clamp(2 * g, 0, std::max(0, obstacle_count - 1));
//             int end = std::min(start + std::max(1, NUM_VALID_OBSTACLES) - 1, obstacle_count - 1);
//             apply_window(start, end);
//             return;
//         }
//         const int sidx = std::clamp(g, 0, (int)seg_start.size() - 1);
//         int start = seg_start[sidx];
//         int end = std::min(start + std::max(1, NUM_VALID_OBSTACLES) - 1, obstacle_count - 1);
//         apply_window(start, end);
//     }
//
//     // Sliding: look back by ~NUM_VALID_OBSTACLES/4 worth of obstacles (segment-aware)
//     void update_obstacles_sliding(Local_Goal_Manager &local_manager_) {
//         if (obstacle_count <= 0)
//             return;
//
//         const int g = std::max(0, local_manager_.get_local_goal_counter());
//         if (seg_start.empty()) {
//             int start = std::clamp(2 * g - NUM_VALID_OBSTACLES / 4, 0, std::max(0, obstacle_count - 1));
//             int end = std::min(start + std::max(1, NUM_VALID_OBSTACLES) - 1, obstacle_count - 1);
//             apply_window(start, end);
//             return;
//         }
//
//         // Estimate obstacles/segment to translate obstacle window to segment window
//         const double avg_per_seg = std::max(1.0, (double)obstacle_count / std::max(1, (int)seg_start.size()));
//         const int seg_lookback = std::max(0, (int)std::round((NUM_VALID_OBSTACLES / 4.0) / avg_per_seg));
//         const int start_seg = std::clamp(g - seg_lookback, 0, (int)seg_start.size() - 1);
//         int start = seg_start[start_seg];
//         int end = std::min(start + std::max(1, NUM_VALID_OBSTACLES) - 1, obstacle_count - 1);
//         apply_window(start, end);
//     }
//
//     // ------------ access & maintenance ------------
//     const Obstacle *get_active_obstacles(int &out_count) {
//         static Obstacle active_obs[MAX_OBSTACLES];
//         out_count = 0;
//         for (int i = 0; i < obstacle_count; ++i) {
//             if (is_active[i]) {
//                 active_obs[out_count++] = obstacle_array[i];
//             }
//         }
//         return active_obs;
//     }
//
//     void clean_data() {
//         std::fill(std::begin(is_active), std::end(is_active), false);
//         obstacle_count = 0;
//         std::memset(obstacle_array, 0, sizeof(obstacle_array));
//         seg_start.clear();
//         seg_count.clear();
//     }
//
//   private:
//     void apply_window(int start, int end) {
//         for (int i = 0; i < obstacle_count; ++i) {
//             is_active[i] = (i >= start && i <= end);
//         }
//         for (int i = obstacle_count; i < MAX_OBSTACLES; ++i) {
//             is_active[i] = false;
//         }
//     }
// };
//
// #endif // OBSTACLES_HPP
