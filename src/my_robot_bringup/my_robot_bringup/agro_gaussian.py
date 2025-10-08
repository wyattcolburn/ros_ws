
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

# ===== Global limits =====
DT        = 0.05   # 20 Hz
V_MAX     = 0.30   # m/s
W_MAX     = 1.40   # rad/s
A_LAT_MAX = 1.10   # m/s^2   # cap v * |w|

NO_PIVOT_V = 0.05
MIN_V_TURN = 0.10

V_BASE_MU  = 0.22
V_BASE_SIG = 0.05

LIN_SMOOTH = 0.25
LIN_NOISE  = 0.03
W_BLEND    = 0.75   # faster ω response so turns show up

# ===== Episode mixture =====
P_STRAIGHT = 0.60
P_GENTLE   = 0.30
P_AGGR     = 0.10

# Durations
STRAIGHT_LEN_MIN = int(0.6 / DT)
STRAIGHT_LEN_MAX = int(1.3 / DT)
ARC_LEN_MIN      = int(1.2 / DT)
ARC_LEN_MAX      = int(3.0 / DT)

# Desired heading changes
GENTLE_ANGLE_DEG = (30.0, 70.0)
AGGR_ANGLE_DEG   = (70.0, 110.0)

# ω floors (boost gentle a bit)
MIN_W_GENTLE = 0.28
W_GENTLE_MAX = 1.00
MIN_W_AGGR   = 0.35
W_AGGR_MAX   = 1.00

# "Straight" should still bend a bit
STRAIGHT_W_DRIFT_SIG  = 0.06  # ↑ drift magnitude
STRAIGHT_W_NOISE_SIG  = 0.015
STRAIGHT_W_DRIFT_GAIN = 0.10   # ↑ drift adapt rate

# Micro-weave inside straight segments (short S-curves)
MICRO_WEAVE_PROB   = 0.35
MICRO_LEN_MIN      = int(0.5 / DT)
MICRO_LEN_MAX      = int(1.2 / DT)
MICRO_W_MIN        = 0.25
MICRO_W_MAX        = 0.40
MICRO_ALTERNATE_WT = 0.70

# Turn watchdog: force a gentle arc if we haven’t been turning
TURN_WATCHDOG_SEC   = 3.0
TURN_WATCHDOG_MINW  = 0.08
TURN_WATCHDOG_LEN   = int(TURN_WATCHDOG_SEC / DT)

ALTERNATE_WEIGHT = 0.65
ENVELOPE_MEAN = 2.0 / np.pi

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
        super().__init__('random_walk_mixture_weave')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(DT, self.step)

        self.prev_v = 0.0
        self.prev_w = 0.0

        # episode state
        self.mode = None                    # 'straight' | 'gentle' | 'aggressive'
        self.episode_len = 0
        self.episode_tick = 0

        # arc params
        self.w_peak = 0.0
        self.v_arc  = 0.0
        self.last_dir = 0

        # straight drift + micro-weave
        self.w_bias = 0.0
        self.weave_active = False
        self.weave_len = 0
        self.weave_tick = 0
        self.weave_w_peak = 0.0
        self.weave_dir_last = 0

        # turn watchdog buffer (rolling mean of |w_cmd|)
        self.w_hist = np.zeros(TURN_WATCHDOG_LEN, dtype=float)
        self.w_hist_i = 0

        self.start_new_episode()
        self.get_logger().info("RW mix: 45% straight (jitter+weave), 38% gentle, 17% aggressive — no pivot.")

    # --------- helpers ----------
    def _choose_dir(self, last, weight):
        if last == 0 or np.random.rand() > weight:
            return np.random.choice([-1, 1])
        return -last

    def _start_weave(self):
        self.weave_active = True
        self.weave_len  = np.random.randint(MICRO_LEN_MIN, MICRO_LEN_MAX + 1)
        self.weave_tick = 0
        sgn = self._choose_dir(self.weave_dir_last, MICRO_ALTERNATE_WT)
        self.weave_w_peak = sgn * np.random.uniform(MICRO_W_MIN, MICRO_W_MAX)

    def _stop_weave(self):
        self.weave_active = False
        self.weave_dir_last = -1 if self.weave_w_peak < 0 else 1
        self.weave_tick = 0
        self.weave_len = 0
        self.weave_w_peak = 0.0

    def _setup_arc(self, angle_deg_range, w_cap, w_min, v_factor):
        self.episode_len  = np.random.randint(ARC_LEN_MIN, ARC_LEN_MAX + 1)
        self.episode_tick = 0
        T = self.episode_len * DT
        dpsi = np.deg2rad(np.random.uniform(*angle_deg_range))
        w_peak_req = dpsi / max(T * ENVELOPE_MEAN, 1e-6)
        sgn = self._choose_dir(self.last_dir, ALTERNATE_WEIGHT)
        w_peak = np.clip(w_peak_req, w_min, min(w_cap, W_MAX)) * sgn

        v_base = tnorm(MIN_V_TURN, V_MAX, V_BASE_MU * v_factor, V_BASE_SIG)
        v_cap_from_alat = A_LAT_MAX / max(abs(w_peak), 1e-6)
        v_arc = min(V_MAX, v_base, v_cap_from_alat)
        if v_arc < MIN_V_TURN:
            v_arc = MIN_V_TURN
            w_peak = np.sign(w_peak) * min(abs(w_peak), A_LAT_MAX / v_arc, W_MAX)

        self.w_peak = float(w_peak)
        self.v_arc  = float(v_arc)

    def start_new_episode(self):
        r = np.random.rand()
        if r < P_STRAIGHT:
            self.mode = 'straight'
            self.episode_len  = np.random.randint(STRAIGHT_LEN_MIN, STRAIGHT_LEN_MAX + 1)
            self.episode_tick = 0
            self.w_peak = 0.0; self.v_arc = 0.0
            self._stop_weave()
        elif r < P_STRAIGHT + P_GENTLE:
            self.mode = 'gentle'
            self._setup_arc(GENTLE_ANGLE_DEG, W_GENTLE_MAX, MIN_W_GENTLE, v_factor=0.85)
        else:
            self.mode = 'aggressive'
            self._setup_arc(AGGR_ANGLE_DEG, W_AGGR_MAX, MIN_W_AGGR, v_factor=0.70)

    def _force_gentle_arc(self):
        # Called by watchdog to guarantee turns
        self.mode = 'gentle'
        self._setup_arc(GENTLE_ANGLE_DEG, W_GENTLE_MAX, MIN_W_GENTLE, v_factor=0.85)

    # --------- main loop ----------
    def step(self):
        # episode timing
        self.episode_tick += 1
        if self.episode_tick >= self.episode_len:
            if self.mode in ('gentle', 'aggressive'):
                self.last_dir = -1 if self.w_peak < 0 else 1
            self.start_new_episode()

        # targets for this tick
        if self.mode == 'straight':
            v_target = tnorm(0.0, V_MAX, V_BASE_MU, V_BASE_SIG)
            # slow bias drift + small white noise
            self.w_bias += STRAIGHT_W_DRIFT_GAIN * (np.random.normal(0.0, STRAIGHT_W_DRIFT_SIG) - self.w_bias)
            w_target = self.w_bias + np.random.normal(0.0, STRAIGHT_W_NOISE_SIG)

            # occasional short micro-weave
            if (not self.weave_active) and (np.random.rand() < MICRO_WEAVE_PROB):
                self._start_weave()
            if self.weave_active:
                self.weave_tick += 1
                env = arc_envelope(self.weave_tick, self.weave_len)
                w_target += self.weave_w_peak * env
                if self.weave_tick >= self.weave_len:
                    self._stop_weave()

        elif self.mode in ('gentle', 'aggressive'):
            env = arc_envelope(self.episode_tick, self.episode_len)
            w_target = self.w_peak * env
            v_target = max(self.v_arc, MIN_V_TURN)

            # keep gentle arcs from fading to near-zero at start
            if self.mode == 'gentle' and abs(w_target) < MIN_W_GENTLE * 0.6:
                w_target = np.sign(self.w_peak) * MIN_W_GENTLE * 0.6

        else:
            v_target = V_BASE_MU
            w_target = 0.0

        # lateral acceleration cap (reduces v in tight turns, never increases w)
        if abs(w_target) > 1e-6:
            v_target = min(v_target, A_LAT_MAX / abs(w_target), V_MAX)
        else:
            v_target = min(v_target, V_MAX)

        # smooth commands
        self.prev_v += LIN_SMOOTH * (v_target - self.prev_v)
        self.prev_v += np.random.normal(0.0, LIN_NOISE)
        self.prev_w += W_BLEND * (w_target - self.prev_w)

        # no-pivot rule
        v_cmd = float(np.clip(self.prev_v, 0.0, V_MAX))
        if v_cmd < NO_PIVOT_V:
            w_cmd = 0.0
        else:
            w_cmd = float(np.clip(self.prev_w, -W_MAX, W_MAX))

        # --- turn watchdog: if we've been too straight for too long, force an arc
        self.w_hist[self.w_hist_i] = abs(w_cmd)
        self.w_hist_i = (self.w_hist_i + 1) % TURN_WATCHDOG_LEN
        if self.mode == 'straight' and self.w_hist_i == 0:  # once per window
            if self.w_hist.mean() < TURN_WATCHDOG_MINW:
                self._force_gentle_arc()

        # publish
        msg = Twist()
        msg.linear.x  = v_cmd
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
