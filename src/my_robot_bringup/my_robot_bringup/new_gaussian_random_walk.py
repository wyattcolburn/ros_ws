
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
try:
    from scipy.stats import truncnorm
    HAVE_SCIPY = True
except Exception:
    HAVE_SCIPY = False

# ---------------- Turnier profile (no pivot) ----------------
V_MAX          = 0.30      # m/s hard cap
W_MAX          = 1.40      # rad/s hard cap
DT             = 0.05      # 20 Hz

NO_PIVOT_V     = 0.05      # if v < this, w = 0 (no in-place spin)
MIN_V_TURN     = 0.10      # guarantee some forward motion in arcs
MIN_W_ARC      = 0.18      # minimum |w| during arcs (if v >= NO_PIVOT_V)

A_LAT_MAX      = 1.10      # m/s^2 lateral accel cap (higher → less slowing in tight turns)

# Baseline forward velocity
V_BASE_MU      = 0.22
V_BASE_SIG     = 0.05
LIN_SMOOTH     = 0.25
LIN_NOISE      = 0.03

# Arc episode controls (more, longer, tighter)
ARC_START_PROB = 0.35
ARC_LEN_MIN    = int(1.2 / DT)    # 1.2 s
ARC_LEN_MAX    = int(3.0 / DT)    # 3.0 s
K_PEAK_MU      = 1.35             # mean peak curvature (1/m)
K_PEAK_SIG     = 0.50
K_PEAK_MIN     = 0.40
K_PEAK_MAX     = 3.00
ARC_INTERNAL_NOISE = 0.05
ALTERNATE_WEIGHT   = 0.65

# Cooldown between arcs (shorter → more turning overall)
COOLDOWN_MIN   = int(0.8 / DT)    # 0.8 s
COOLDOWN_MAX   = int(1.6 / DT)    # 1.6 s

# Non-arc mild wandering curvature
NONARC_K_SIG   = 0.12

def tnorm(a, b, mu, sigma):
    if HAVE_SCIPY:
        A, B = (a - mu) / sigma, (b - mu) / sigma
        return float(truncnorm.rvs(A, B, loc=mu, scale=sigma))
    return float(np.clip(np.random.normal(mu, sigma), a, b))

def arc_envelope(t_idx, t_total):
    if t_total <= 1:
        return 1.0
    p = t_idx / float(t_total - 1)
    return float(np.sin(np.pi * p))

class RandomWalkNoPivot(Node):
    def __init__(self):
        super().__init__('random_walk_no_pivot')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(DT, self.step)

        self.prev_v = 0.0
        self.prev_w = 0.0

        self.in_arc = False
        self.arc_len = 0
        self.arc_tick = 0
        self.kappa_peak = 0.0
        self.last_dir = 0

        self.cooldown = 0

        self.get_logger().info("Random walk (more turning, no pivot) started.")

    def choose_arc_dir(self):
        if self.last_dir == 0 or np.random.rand() > ALTERNATE_WEIGHT:
            return np.random.choice([-1, 1])
        return -self.last_dir

    def start_arc(self):
        sgn = self.choose_arc_dir()
        k_peak = tnorm(K_PEAK_MIN, K_PEAK_MAX, K_PEAK_MU, K_PEAK_SIG)
        self.kappa_peak = float(np.clip(k_peak, K_PEAK_MIN, K_PEAK_MAX)) * sgn
        self.arc_len = np.random.randint(ARC_LEN_MIN, ARC_LEN_MAX + 1)
        self.arc_tick = 0
        self.in_arc = True

    def end_arc(self):
        self.in_arc = False
        self.last_dir = -1 if self.kappa_peak < 0 else 1
        self.kappa_peak = 0.0
        self.arc_len = 0
        self.arc_tick = 0
        self.cooldown = np.random.randint(COOLDOWN_MIN, COOLDOWN_MAX + 1)

    def step(self):
        # Arc timing
        if self.cooldown > 0:
            self.cooldown -= 1
        if self.in_arc:
            self.arc_tick += 1
            if self.arc_tick >= self.arc_len:
                self.end_arc()
        else:
            if self.cooldown == 0 and np.random.rand() < ARC_START_PROB:
                self.start_arc()

        # Curvature & linear target
        if self.in_arc:
            k_env = arc_envelope(self.arc_tick, self.arc_len)
            k_target = self.kappa_peak * k_env + np.random.normal(0.0, ARC_INTERNAL_NOISE)
            v_base = tnorm(0.0, V_MAX, V_BASE_MU * 0.90, V_BASE_SIG)
            v_base = max(v_base, MIN_V_TURN)
        else:
            k_target = np.random.normal(0.0, NONARC_K_SIG)
            v_base  = tnorm(0.0, V_MAX, V_BASE_MU, V_BASE_SIG)

        # Lateral accel cap → speed bound based on curvature
        if abs(k_target) > 1e-6:
            v_cap_lat = np.sqrt(max(0.0, A_LAT_MAX / abs(k_target)))
        else:
            v_cap_lat = V_MAX

        v_target = min(v_base, v_cap_lat, V_MAX)
        if self.in_arc:
            # keep moving forward during arcs (still obeys V_MAX)
            v_target = max(v_target, MIN_V_TURN)

        # Map curvature→ω and smooth
        w_target = float(np.clip(k_target * v_target, -W_MAX, W_MAX))

        self.prev_v += LIN_SMOOTH * (v_target - self.prev_v)
        self.prev_v += np.random.normal(0.0, LIN_NOISE)

        # Faster ω response to make turns obvious
        self.prev_w += 0.60 * (w_target - self.prev_w)

        # No-pivot rule + minimum |ω| in arcs
        v_cmd = float(np.clip(self.prev_v, 0.0, V_MAX))
        if v_cmd < NO_PIVOT_V:
            w_cmd = 0.0
        else:
            w_cmd = float(np.clip(self.prev_w, -W_MAX, W_MAX))
            if self.in_arc and abs(w_cmd) < MIN_W_ARC:
                w_cmd = np.sign(w_cmd if w_cmd != 0.0 else self.kappa_peak) * MIN_W_ARC

        msg = Twist()
        msg.linear.x = v_cmd
        msg.angular.z = w_cmd
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = RandomWalkNoPivot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
