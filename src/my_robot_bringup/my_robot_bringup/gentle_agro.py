
#!/usr/bin/env python3
"""
Mixed random walk:
  - 60% WALK   → straight with slight, low-freq jitter (no pivot)
  - 30% WANDER → gentle curvature drift
  - 10% ARC    → decisive arcs (90°ish), still moving forward

Key knobs for "straight but not perfectly straight":
  - OU_WALK_SIGMA:    0.006  (tiny curvature drift)
  - W_MICRO_SIGMA:    0.020  (band-limited ω jitter, very small)
  - W_MICRO_ALPHA:    0.92   (low-frequency jitter)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
try:
    from scipy.stats import truncnorm
    HAVE_SCIPY = True
except Exception:
    HAVE_SCIPY = False

DT = 0.05  # 20 Hz

# --- Limits & constraints ---
V_MAX       = 0.30
W_MAX       = 1.40
NO_PIVOT_V  = 0.06    # no in-place spins
MIN_V_TURN  = 0.10
A_LAT_MAX   = 1.80    # v^2 * |kappa| <= A_LAT_MAX

# --- Base speed profile ---
V_BASE_MU   = 0.22
V_BASE_SIG  = 0.04
LIN_SMOOTH  = 0.22
LIN_NOISE   = 0.01    # slightly smaller to keep speed steady

# --- Mode mix ---
MODE_WALK, MODE_WANDER, MODE_ARC = 0, 1, 2
P_WALK, P_WANDER, P_ARC = 0.60, 0.30, 0.10

# Segment durations (seconds)
WALK_DUR   = (3.0, 6.5)
WANDER_DUR = (1.8, 4.0)
ARC_DUR    = (1.2, 2.8)

# --- WALK: very subtle curvature + micro-jitter on ω ---
OU_WALK_BETA   = 0.60   # stronger pull to straight
OU_WALK_SIGMA  = 0.006  # tiny curvature noise (kappa)
W_MICRO_ALPHA  = 0.92   # AR(1) low-freq jitter on ω
W_MICRO_SIGMA  = 0.020  # very small jitter magnitude (rad/s)

# --- WANDER: gentle curvature drift ---
OU_WAND_BETA   = 0.25
OU_WAND_SIGMA  = 0.035

# --- ARC: decisive, still forward ---
K_PEAK_MIN = 0.9
K_PEAK_MU  = 2.2
K_PEAK_SIG = 0.8
K_PEAK_MAX = 5.0
MIN_W_ARC  = 0.60
ARC_NOISE  = 0.06
ALT_WEIGHT = 0.65  # prefer alternating L/R

def tnorm(a, b, mu, sigma):
    if HAVE_SCIPY:
        A, B = (a - mu) / sigma, (b - mu) / sigma
        return float(truncnorm.rvs(A, B, loc=mu, scale=sigma))
    return float(np.clip(np.random.normal(mu, sigma), a, b))

def arc_envelope(t_idx, t_total):
    if t_total <= 1:
        return 1.0
    p = t_idx / float(max(1, t_total - 1))
    return float(np.sin(np.pi * p))

class MixedRandomWalk(Node):
    def __init__(self):
        super().__init__('mixed_random_walk_straightish')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(DT, self.step)

        # states
        self.prev_v = 0.0
        self.prev_w = 0.0

        self.mode = MODE_WALK
        self.ticks_left = 0

        # curvature state for WALK/WANDER
        self.kappa = 0.0
        # micro ω jitter (band-limited) for WALK only
        self.w_micro = 0.0

        # arc state
        self.kappa_peak = 0.0
        self.arc_len = 0
        self.arc_tick = 0
        self.last_dir = 0

        self._choose_new_segment(initial=True)
        self.get_logger().info("MixedRandomWalk: 60% straight-ish, 30% gentle, 10% arcs. No pivoting.")

    # ---------- scheduler ----------
    def _choose_mode(self):
        r = np.random.rand()
        if r < P_WALK:
            return MODE_WALK
        elif r < P_WALK + P_WANDER:
            return MODE_WANDER
        else:
            return MODE_ARC

    def _duration_ticks(self, rng):
        lo, hi = rng
        return int(np.random.uniform(lo, hi) / DT)

    def _choose_new_segment(self, initial=False):
        m = self._choose_mode()
        if (not initial) and self.mode == MODE_ARC and m == MODE_ARC:
            # avoid back-to-back arcs
            m = MODE_WALK if np.random.rand() < 0.7 else MODE_WANDER

        self.mode = m
        if self.mode == MODE_WALK:
            self.ticks_left = self._duration_ticks(WALK_DUR)
        elif self.mode == MODE_WANDER:
            self.ticks_left = self._duration_ticks(WANDER_DUR)
        else:
            self.ticks_left = self._duration_ticks(ARC_DUR)
            self._start_arc()

    # ---------- arc helpers ----------
    def _choose_dir(self):
        if self.last_dir == 0 or np.random.rand() > ALT_WEIGHT:
            return np.random.choice([-1, 1])
        return -self.last_dir

    def _start_arc(self):
        sgn = self._choose_dir()
        k_peak = tnorm(K_PEAK_MIN, K_PEAK_MAX, K_PEAK_MU, K_PEAK_SIG)
        self.kappa_peak = float(np.clip(k_peak, K_PEAK_MIN, K_PEAK_MAX)) * sgn
        self.arc_len = self.ticks_left
        self.arc_tick = 0

    def _end_arc(self):
        self.last_dir = -1 if self.kappa_peak < 0 else 1
        self.kappa_peak = 0.0
        self.arc_len = 0
        self.arc_tick = 0

    # ---------- main step ----------
    def step(self):
        # segment timing
        self.ticks_left -= 1
        if self.ticks_left <= 0:
            if self.mode == MODE_ARC:
                self._end_arc()
            self._choose_new_segment()

        # curvature target
        if self.mode == MODE_ARC:
            self.arc_tick += 1
            k_env = arc_envelope(self.arc_tick, self.arc_len)
            k_target = self.kappa_peak * k_env + np.random.normal(0.0, ARC_NOISE)
        elif self.mode == MODE_WALK:
            # very small OU drift → nearly straight
            self.kappa += (-OU_WALK_BETA * self.kappa) * DT + OU_WALK_SIGMA * np.sqrt(DT) * np.random.normal()
            k_target = self.kappa
        else:  # WANDER
            self.kappa += (-OU_WAND_BETA * self.kappa) * DT + OU_WAND_SIGMA * np.sqrt(DT) * np.random.normal()
            k_target = self.kappa

        # base linear speed
        if self.mode == MODE_ARC:
            v_base = tnorm(0.0, V_MAX, V_BASE_MU * 0.95, V_BASE_SIG)
            v_base = max(v_base, MIN_V_TURN)
        else:
            v_base = tnorm(0.0, V_MAX, V_BASE_MU, V_BASE_SIG)

        # lateral acceleration cap
        if abs(k_target) > 1e-6:
            v_cap = np.sqrt(max(0.0, A_LAT_MAX / abs(k_target)))
        else:
            v_cap = V_MAX
        v_target = min(v_base, v_cap, V_MAX)

        # curvature → ω
        w_target = float(np.clip(k_target * v_target, -W_MAX, W_MAX))

        # add tiny band-limited jitter ONLY in WALK and only when already very straight
        if self.mode == MODE_WALK and abs(w_target) < 0.12:
            # AR(1): w_micro = a*w_micro + sigma*sqrt(1-a^2)*N(0,1)
            a = W_MICRO_ALPHA
            s = W_MICRO_SIGMA * np.sqrt(max(1e-6, 1.0 - a * a))
            self.w_micro = a * self.w_micro + s * np.random.normal()
            w_target += self.w_micro
        else:
            # bleed jitter out when not walking straight
            self.w_micro *= 0.85

        # smooth v and ω (faster ω during arcs to make them decisive)
        self.prev_v += LIN_SMOOTH * (v_target - self.prev_v)
        self.prev_v += np.random.normal(0.0, LIN_NOISE)
        w_alpha = 0.65 if self.mode == MODE_ARC else 0.22
        self.prev_w += w_alpha * (w_target - self.prev_w)

        # no-pivot rule + minimum ω in arcs
        v_cmd = float(np.clip(self.prev_v, 0.0, V_MAX))
        if v_cmd < NO_PIVOT_V:
            w_cmd = 0.0
        else:
            w_cmd = float(np.clip(self.prev_w, -W_MAX, W_MAX))
            if self.mode == MODE_ARC and abs(w_cmd) < MIN_W_ARC:
                sgn = np.sign(w_cmd if w_cmd != 0.0 else self.kappa_peak)
                w_cmd = sgn * MIN_W_ARC

        msg = Twist()
        msg.linear.x = v_cmd
        msg.angular.z = w_cmd
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MixedRandomWalk()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
