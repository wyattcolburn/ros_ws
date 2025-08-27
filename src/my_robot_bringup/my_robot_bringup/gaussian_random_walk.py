#
# """
# This node creates random walks to facilitate training.
# Small improvements:
#  - Prevent persistent circling by steering toward a near-zero angular target that changes every ~1–2 s.
#  - Smoothly slew toward that target and add tiny zero-mean jitter.
#  - Keep linear speed near-constant with slight variability.
# """
#
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import numpy as np
# from scipy.stats import truncnorm
#
# # Bounds
# v_min = 0.0
# v_max = 0.4
# w_min = -1.4
# w_max = 1.4
#
#
# def truncated_gaussian(min_val, max_val, mu, sigma):
#     # Sample from a truncated normal distribution within [min_val, max_val]
#     a, b = (min_val - mu) / sigma, (max_val - mu) / sigma
#     return float(truncnorm.rvs(a, b, loc=mu, scale=sigma))
#
#
# def prev_based_gaussian(min_val, max_val, prev_mu, sigma):
#     # Same idea as your original helper (kept for compatibility if you want it)
#     a, b = (min_val - prev_mu) / sigma, (max_val - prev_mu) / sigma
#     return float(truncnorm.rvs(a, b, loc=prev_mu, scale=sigma))
#
#
# class randomMotor(Node):
#     def __init__(self):
#         super().__init__('random_motor_node')
#         self.get_logger().info('Random motor node started')
#
#         self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.timer_period = 0.05  # 20 Hz
#         self.timer = self.create_timer(
#             self.timer_period, self.publish_random_vel)
#
#         # --- Angular state (minimal additions) ---
#         self.prev_mu = 0.0                  # current angular command (rad/s)
#         self.omega_set = 0.0                # small, near-zero target (rad/s)
#         self.target_ticks = 0               # countdown until we resample target
#         self.target_ticks_min = int(1.0 / self.timer_period)   # ~1 s
#         self.target_ticks_max = int(2.0 / self.timer_period)   # ~2 s
#         self.omega_bound = 0.55             # keep turns gentle: [-0.35, 0.35]
#
#         self.slew = 0.12                    # per-tick blend toward target
#         self.decay = 0.08                   # mild pull-to-zero each tick
#         self.noise_sigma = 0.04             # tiny angular jitter
#
#         # (Kept from your original; not strictly required now)
#         self.angular_counter = 0
#         self.angular_limit = 5
#
#         self.prev_linear = 0.2      # Initialize linear velocity state
#         self.prev_angular = 0.0
#
#     def publish_random_vel(self):
#         msg = Twist()
#
#         # Add occasional challenge modes (20% of the time)
#         if np.random.random() < 0.2:
#             # Challenge mode: More extreme, less predictable movements
#             target_linear = truncated_gaussian(
#                 0.05, v_max, 0.15, 0.12)  # Wider range
#             target_angular = truncated_gaussian(
#                 w_min, w_max, 0.0, 1.0)  # More extreme turns
#
#             # More responsive (less smooth) for challenge scenarios
#             linear_smoothing = 0.35
#             angular_smoothing = 0.45
#
#             # More noise for unpredictability
#             linear_noise = 0.08
#             angular_noise = 0.15
#
#         else:
#             # Your current successful approach (80% of the time)
#             target_linear = truncated_gaussian(v_min, v_max, 0.2, 0.1)
#             target_angular = truncated_gaussian(w_min, w_max, 0.0, 0.6)
#
#             linear_smoothing = 0.25
#             angular_smoothing = 0.30
#             linear_noise = 0.05
#             angular_noise = 0.10
#
#         # Apply smoothing
#         self.prev_linear += linear_smoothing * \
#             (target_linear - self.prev_linear)
#         self.prev_angular += angular_smoothing * \
#             (target_angular - self.prev_angular)
#
#         # Add noise
#         self.prev_linear += np.random.normal(0, linear_noise)
#         self.prev_angular += np.random.normal(0, angular_noise)
#
#         msg.linear.x = np.clip(self.prev_linear, v_min, v_max)
#         msg.angular.z = np.clip(self.prev_angular, w_min, w_max)
#
#         self.publisher_.publish(msg)
#         self.get_logger().info(
#             f'Publishing: v={msg.linear.x:.3f}, w={msg.angular.z:.3f}')
#     # def publish_random_vel(self):
#     #     msg = Twist()
#     #
#     #     # Reduce smoothing and increase noise from original
#     #     target_linear = truncated_gaussian(v_min, v_max, 0.2, 0.1)
#     #     target_angular = truncated_gaussian(
#     #         w_min, w_max, 0.0, 0.6)  # Higher variance
#     #
#     #     # Less aggressive smoothing
#     #     self.prev_linear += 0.25 * \
#     #         (target_linear - self.prev_linear)    # vs 0.12
#     #     self.prev_angular += 0.30 * \
#     #         (target_angular - self.prev_angular)  # vs 0.12
#     #
#     #     # More noise
#     #     self.prev_linear += np.random.normal(0, 0.05)   # vs 0.04
#     #     self.prev_angular += np.random.normal(0, 0.10)  # vs 0.04
#     #
#     #     msg.linear.x = np.clip(self.prev_linear, v_min, v_max)
#     #     msg.angular.z = np.clip(self.prev_angular, w_min, w_max)
#     #
#     #     self.publisher_.publish(msg)
#     #
#     # def publish_random_vel(self):
#     #     msg = Twist()
#     #
#     #     # --- Linear speed: near-constant with small variability (your original idea) ---
#     #     # Mean ~0.2 m/s, sigma ~0.08, clipped to [0, 0.4]
#     #     msg.linear.x = truncated_gaussian(v_min, v_max, 0.2, 0.08)
#     #
#     #     # --- Angular speed: small changes, no circles ---
#     #     # Occasionally resample a small near-zero target (every ~1–2 s)
#     #     if self.target_ticks <= 0:
#     #         # Normal around 0, clipped to gentle range
#     #         self.omega_set = float(np:clip(np.random.normal(
#     #             0.0, 0.25), -self.omega_bound, self.omega_bound))
#     #         self.target_ticks = np.random.randint(
#     #             self.target_ticks_min, self.target_ticks_max + 1)
#     #     self.target_ticks -= 1
#     #
#     #     # Optional: keep your older refresh path (low influence now)
#     #     if self.angular_counter >= self.angular_limit:
#     #         self.angular_counter = 0
#     #         # Re-sample around current value (kept from your code)
#     #         self.prev_mu = prev_based_gaussian(
#     #             w_min, w_max, self.prev_mu, 0.10)
#     #     self.angular_counter += 1
#     #
#     #     # Slew toward target + mild decay + tiny noise (prevents constant ω and big circles)
#     #     self.prev_mu += self.slew * (self.omega_set - self.prev_mu)
#     #     self.prev_mu = (1.0 - self.decay) * self.prev_mu + \
#     #         float(np.random.normal(0.0, self.noise_sigma))
#     #     self.prev_mu = float(np.clip(self.prev_mu, w_min, w_max))
#     #
#     #     msg.angular.z = self.prev_mu
#     #
#     #     self.publisher_.publish(msg)
#         # Uncomment if you want to watch values:
#         # self.get_logger().info(f'v={msg.linear.x:.2f}, omega={msg.angular.z:.2f}, target={self.omega_set:.2f}')
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = randomMotor()
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
from scipy.stats import truncnorm

# Bounds
v_min = 0.0
v_max = 0.4
w_min = -1.4
w_max = 1.4


def truncated_gaussian(min_val, max_val, mu, sigma):
    a, b = (min_val - mu) / sigma, (max_val - mu) / sigma
    return float(truncnorm.rvs(a, b, loc=mu, scale=sigma))


class randomMotor(Node):
    def __init__(self):
        super().__init__('random_motor_node')
        self.get_logger().info('Random motor node started')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(
            self.timer_period, self.publish_random_vel)

        # ---------- Mostly-straight baseline ----------
        self.DRIFT_DECAY = 0.10            # pull-to-zero each tick
        self.DRIFT_NOISE = 0.015           # tiny angular jitter
        self.DRIFT_SMOOTH = 0.18           # smoothing toward drift target
        self.DRIFT_TARGET_SIGMA = 0.08     # near-zero drift target

        # ---------- Arc episodes (left/right) ----------
        # Start frequency & duration (keeps most time straight)
        self.ARC_START_PROB = 0.10         # chance to start when off cooldown
        self.ARC_LEN_MIN = int(0.8 / self.timer_period)   # 0.8 s
        self.ARC_LEN_MAX = int(2.4 / self.timer_period)   # 2.4 s
        # Peak angular speed during arc (moderate, real-looking arcs)
        self.ARC_OMEGA_MU = 0.55           # rad/s nominal peak
        self.ARC_OMEGA_SIG = 0.12
        self.ARC_OMEGA_MIN = 0.35          # ensure it actually arcs
        # Mild curvature wobble inside arc to avoid perfect circles
        self.ARC_INTERNAL_NOISE = 0.03
        # Prefer alternation L<->R so it doesn't drift forever
        self.ALTERNATE_WEIGHT = 0.65       # prob. to pick opposite of last

        # ---------- Cooldown between arcs ----------
        self.COOLDOWN_MIN = int(2.0 / self.timer_period)   # 2 s
        self.COOLDOWN_MAX = int(4.0 / self.timer_period)   # 4 s
        self.cooldown_ticks = 0

        # ---------- Linear velocity ----------
        self.V_BASE_MU = 0.22
        self.V_BASE_SIG = 0.05
        self.V_ARC_FACTOR = 0.90    # slight slowdown during arcs
        self.LIN_SMOOTH = 0.25
        self.LIN_NOISE = 0.03

        # ---------- State ----------
        self.prev_linear = 0.2
        self.prev_angular = 0.0

        self.in_arc = False
        self.arc_len = 0
        self.arc_tick = 0
        self.arc_peak = 0.0     # signed peak ω for this arc
        self.last_dir = 0       # -1 (left), +1 (right), 0 (none yet)

    def _choose_arc_direction(self):
        # Prefer opposite of last_dir with ALTERNATE_WEIGHT
        if self.last_dir == 0 or np.random.random() > self.ALTERNATE_WEIGHT:
            return np.random.choice([-1, 1])
        return -self.last_dir

    def _start_arc(self):
        sgn = self._choose_arc_direction()
        peak = truncated_gaussian(
            w_min, w_max, self.ARC_OMEGA_MU, self.ARC_OMEGA_SIG)
        peak = max(self.ARC_OMEGA_MIN, min(abs(peak), w_max)) * sgn
        self.arc_peak = float(np.clip(peak, w_min, w_max))
        self.arc_len = np.random.randint(
            self.ARC_LEN_MIN, self.ARC_LEN_MAX + 1)
        self.arc_tick = 0
        self.in_arc = True

    def _end_arc(self):
        self.in_arc = False
        self.last_dir = -1 if self.arc_peak < 0 else 1
        self.arc_len = 0
        self.arc_tick = 0
        self.arc_peak = 0.0
        self.cooldown_ticks = np.random.randint(
            self.COOLDOWN_MIN, self.COOLDOWN_MAX + 1)

    def _arc_envelope(self, t_idx, t_total):
        """
        Smooth ease-in/ease-out envelope in [0,1]:
        e(p) = sin(pi * p), p in [0,1]
        Starts/ends at 0, peaks at 1 mid-arc.
        """
        if t_total <= 1:
            return 1.0
        p = t_idx / float(t_total - 1)
        return float(np.sin(np.pi * p))

    def publish_random_vel(self):
        msg = Twist()

        # Cooldown / arc timing
        if self.cooldown_ticks > 0:
            self.cooldown_ticks -= 1

        if self.in_arc:
            self.arc_tick += 1
            if self.arc_tick >= self.arc_len:
                self._end_arc()
        else:
            if self.cooldown_ticks == 0 and np.random.random() < self.ARC_START_PROB:
                self._start_arc()

        # ----- Angular target -----
        if self.in_arc:
            # Smooth envelope to form an arc; add small internal wobble
            env = self._arc_envelope(self.arc_tick, self.arc_len)
            target_angular = self.arc_peak * env
            target_angular += np.random.normal(0.0, self.ARC_INTERNAL_NOISE)
        else:
            # Mostly straight with tiny, decaying drift
            drift_target = np.random.normal(0.0, self.DRIFT_TARGET_SIGMA)
            # Operate directly on state for smooth, decay-to-zero behavior
            self.prev_angular += self.DRIFT_SMOOTH * \
                (drift_target - self.prev_angular)
            self.prev_angular *= (1.0 - self.DRIFT_DECAY)
            self.prev_angular += np.random.normal(0.0, self.DRIFT_NOISE)
            target_angular = self.prev_angular

        # ----- Linear target -----
        if self.in_arc:
            base = truncated_gaussian(
                v_min, v_max, self.V_BASE_MU, self.V_BASE_SIG)
            target_linear = base * self.V_ARC_FACTOR
        else:
            target_linear = truncated_gaussian(
                v_min, v_max, self.V_BASE_MU, self.V_BASE_SIG)

        # ----- Smooth & noise -----
        # For arcs we rely on the envelope rather than a heavy smoothing filter,
        # but still blend a bit by reusing prev_angular as integration state.
        ang_blend = 0.35 if self.in_arc else 0.0
        self.prev_angular += ang_blend * (target_angular - self.prev_angular)

        self.prev_linear += self.LIN_SMOOTH * \
            (target_linear - self.prev_linear)
        self.prev_linear += np.random.normal(0.0, self.LIN_NOISE)

        # Saturate & publish
        msg.linear.x = float(np.clip(self.prev_linear,  v_min, v_max))
        msg.angular.z = float(
            np.clip(self.prev_angular if not self.in_arc else target_angular, w_min, w_max))

        self.publisher_.publish(msg)
        # self.get_logger().info(f'v={msg.linear.x:.3f}, w={msg.angular.z:.3f}, arc={self.in_arc}, dir={np.sign(self.arc_peak) if self.in_arc else 0}')


def main(args=None):
    rclpy.init(args=args)
    node = randomMotor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
