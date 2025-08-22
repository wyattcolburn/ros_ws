
"""
This node creates random walks to facilitate training.
Small improvements:
 - Prevent persistent circling by steering toward a near-zero angular target that changes every ~1–2 s.
 - Smoothly slew toward that target and add tiny zero-mean jitter.
 - Keep linear speed near-constant with slight variability.
"""

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
    # Sample from a truncated normal distribution within [min_val, max_val]
    a, b = (min_val - mu) / sigma, (max_val - mu) / sigma
    return float(truncnorm.rvs(a, b, loc=mu, scale=sigma))


def prev_based_gaussian(min_val, max_val, prev_mu, sigma):
    # Same idea as your original helper (kept for compatibility if you want it)
    a, b = (min_val - prev_mu) / sigma, (max_val - prev_mu) / sigma
    return float(truncnorm.rvs(a, b, loc=prev_mu, scale=sigma))


class randomMotor(Node):
    def __init__(self):
        super().__init__('random_motor_node')
        self.get_logger().info('Random motor node started')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(
            self.timer_period, self.publish_random_vel)

        # --- Angular state (minimal additions) ---
        self.prev_mu = 0.0                  # current angular command (rad/s)
        self.omega_set = 0.0                # small, near-zero target (rad/s)
        self.target_ticks = 0               # countdown until we resample target
        self.target_ticks_min = int(1.0 / self.timer_period)   # ~1 s
        self.target_ticks_max = int(2.0 / self.timer_period)   # ~2 s
        self.omega_bound = 0.55             # keep turns gentle: [-0.35, 0.35]

        self.slew = 0.12                    # per-tick blend toward target
        self.decay = 0.08                   # mild pull-to-zero each tick
        self.noise_sigma = 0.04             # tiny angular jitter

        # (Kept from your original; not strictly required now)
        self.angular_counter = 0
        self.angular_limit = 5

        self.prev_linear = 0.2      # Initialize linear velocity state
        self.prev_angular = 0.0

    def publish_random_vel(self):
        msg = Twist()

        # Add occasional challenge modes (20% of the time)
        if np.random.random() < 0.2:
            # Challenge mode: More extreme, less predictable movements
            target_linear = truncated_gaussian(
                0.05, v_max, 0.15, 0.12)  # Wider range
            target_angular = truncated_gaussian(
                w_min, w_max, 0.0, 1.0)  # More extreme turns

            # More responsive (less smooth) for challenge scenarios
            linear_smoothing = 0.35
            angular_smoothing = 0.45

            # More noise for unpredictability
            linear_noise = 0.08
            angular_noise = 0.15

        else:
            # Your current successful approach (80% of the time)
            target_linear = truncated_gaussian(v_min, v_max, 0.2, 0.1)
            target_angular = truncated_gaussian(w_min, w_max, 0.0, 0.6)

            linear_smoothing = 0.25
            angular_smoothing = 0.30
            linear_noise = 0.05
            angular_noise = 0.10

        # Apply smoothing
        self.prev_linear += linear_smoothing * \
            (target_linear - self.prev_linear)
        self.prev_angular += angular_smoothing * \
            (target_angular - self.prev_angular)

        # Add noise
        self.prev_linear += np.random.normal(0, linear_noise)
        self.prev_angular += np.random.normal(0, angular_noise)

        msg.linear.x = np.clip(self.prev_linear, v_min, v_max)
        msg.angular.z = np.clip(self.prev_angular, w_min, w_max)

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publishing: v={msg.linear.x:.3f}, w={msg.angular.z:.3f}')
    # def publish_random_vel(self):
    #     msg = Twist()
    #
    #     # Reduce smoothing and increase noise from original
    #     target_linear = truncated_gaussian(v_min, v_max, 0.2, 0.1)
    #     target_angular = truncated_gaussian(
    #         w_min, w_max, 0.0, 0.6)  # Higher variance
    #
    #     # Less aggressive smoothing
    #     self.prev_linear += 0.25 * \
    #         (target_linear - self.prev_linear)    # vs 0.12
    #     self.prev_angular += 0.30 * \
    #         (target_angular - self.prev_angular)  # vs 0.12
    #
    #     # More noise
    #     self.prev_linear += np.random.normal(0, 0.05)   # vs 0.04
    #     self.prev_angular += np.random.normal(0, 0.10)  # vs 0.04
    #
    #     msg.linear.x = np.clip(self.prev_linear, v_min, v_max)
    #     msg.angular.z = np.clip(self.prev_angular, w_min, w_max)
    #
    #     self.publisher_.publish(msg)
    #
    # def publish_random_vel(self):
    #     msg = Twist()
    #
    #     # --- Linear speed: near-constant with small variability (your original idea) ---
    #     # Mean ~0.2 m/s, sigma ~0.08, clipped to [0, 0.4]
    #     msg.linear.x = truncated_gaussian(v_min, v_max, 0.2, 0.08)
    #
    #     # --- Angular speed: small changes, no circles ---
    #     # Occasionally resample a small near-zero target (every ~1–2 s)
    #     if self.target_ticks <= 0:
    #         # Normal around 0, clipped to gentle range
    #         self.omega_set = float(np:clip(np.random.normal(
    #             0.0, 0.25), -self.omega_bound, self.omega_bound))
    #         self.target_ticks = np.random.randint(
    #             self.target_ticks_min, self.target_ticks_max + 1)
    #     self.target_ticks -= 1
    #
    #     # Optional: keep your older refresh path (low influence now)
    #     if self.angular_counter >= self.angular_limit:
    #         self.angular_counter = 0
    #         # Re-sample around current value (kept from your code)
    #         self.prev_mu = prev_based_gaussian(
    #             w_min, w_max, self.prev_mu, 0.10)
    #     self.angular_counter += 1
    #
    #     # Slew toward target + mild decay + tiny noise (prevents constant ω and big circles)
    #     self.prev_mu += self.slew * (self.omega_set - self.prev_mu)
    #     self.prev_mu = (1.0 - self.decay) * self.prev_mu + \
    #         float(np.random.normal(0.0, self.noise_sigma))
    #     self.prev_mu = float(np.clip(self.prev_mu, w_min, w_max))
    #
    #     msg.angular.z = self.prev_mu
    #
    #     self.publisher_.publish(msg)
        # Uncomment if you want to watch values:
        # self.get_logger().info(f'v={msg.linear.x:.2f}, omega={msg.angular.z:.2f}, target={self.omega_set:.2f}')


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
