
#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

# ============================================================
#  Parameters copied from your random walk ROS node
# ============================================================
DT        = 0.05   # 20 Hz
V_MAX     = 0.30
W_MAX     = 1.40
A_LAT_MAX = 1.10

NO_PIVOT_V = 0.05
MIN_V_TURN = 0.10

V_BASE_MU  = 0.22
V_BASE_SIG = 0.05

LIN_SMOOTH = 0.25
LIN_NOISE  = 0.03
W_BLEND    = 0.75

P_STRAIGHT = 0.60
P_GENTLE   = 0.30
P_AGGR     = 0.10

STRAIGHT_LEN_MIN = int(0.6 / DT)
STRAIGHT_LEN_MAX = int(1.3 / DT)
ARC_LEN_MIN      = int(1.2 / DT)
ARC_LEN_MAX      = int(3.0 / DT)

GENTLE_ANGLE_DEG = (30, 70)
AGGR_ANGLE_DEG   = (70, 110)

MIN_W_GENTLE = 0.28
W_GENTLE_MAX = 1.00
MIN_W_AGGR   = 0.35
W_AGGR_MAX   = 1.00

STRAIGHT_W_DRIFT_SIG  = 0.06
STRAIGHT_W_NOISE_SIG  = 0.015
STRAIGHT_W_DRIFT_GAIN = 0.10

MICRO_WEAVE_PROB   = 0.35
MICRO_LEN_MIN      = int(0.5 / DT)
MICRO_LEN_MAX      = int(1.2 / DT)
MICRO_W_MIN        = 0.25
MICRO_W_MAX        = 0.40
MICRO_ALTERNATE_WT = 0.70

TURN_WATCHDOG_SEC   = 3.0
TURN_WATCHDOG_MINW  = 0.08
TURN_WATCHDOG_LEN   = int(TURN_WATCHDOG_SEC / DT)

ALTERNATE_WEIGHT = 0.65
ENVELOPE_MEAN = 2.0 / np.pi


# ============================================================
# Helper functions
# ============================================================

def tnorm(a, b, mu, sigma):
    """Truncated normal without scipy."""
    x = np.random.normal(mu, sigma)
    return float(np.clip(x, a, b))

def arc_envelope(t_idx, t_total):
    if t_total <= 1:
        return 1.0
    p = t_idx / float(t_total - 1)
    return float(np.sin(np.pi * p))


# ============================================================
# Random walk simulator (non-ROS version)
# ============================================================

class RandomWalkSim:
    def __init__(self):
        self.prev_v = 0.0
        self.prev_w = 0.0

        self.mode = None
        self.episode_len = 0
        self.episode_tick = 0

        self.w_peak = 0.0
        self.v_arc  = 0.0
        self.last_dir = 0

        self.w_bias = 0.0
        self.weave_active = False
        self.weave_len = 0
        self.weave_tick = 0
        self.weave_w_peak = 0.0
        self.weave_dir_last = 0

        self.w_hist = np.zeros(TURN_WATCHDOG_LEN)
        self.w_hist_i = 0

        self.start_new_episode()

    # --------------------------------------------------------
    def _choose_dir(self, last, weight):
        if last == 0 or np.random.rand() > weight:
            return np.random.choice([-1, 1])
        return -last

    def _start_weave(self):
        self.weave_active = True
        self.weave_len  = np.random.randint(MICRO_LEN_MIN, MICRO_LEN_MAX + 1)
        self.weave_tick = 0
        s = self._choose_dir(self.weave_dir_last, MICRO_ALTERNATE_WT)
        self.weave_w_peak = s * np.random.uniform(MICRO_W_MIN, MICRO_W_MAX)

    def _stop_weave(self):
        self.weave_active = False
        self.weave_dir_last = -1 if self.weave_w_peak < 0 else 1
        self.weave_tick = 0
        self.weave_len = 0
        self.weave_w_peak = 0.0

    def _setup_arc(self, angle_range, w_cap, w_min, v_factor):
        self.episode_len  = np.random.randint(ARC_LEN_MIN, ARC_LEN_MAX + 1)
        self.episode_tick = 0

        T = self.episode_len * DT
        dpsi = np.deg2rad(np.random.uniform(*angle_range))

        w_peak_req = dpsi / max(T * ENVELOPE_MEAN, 1e-6)
        s = self._choose_dir(self.last_dir, ALTERNATE_WEIGHT)
        w_peak = np.clip(w_peak_req, w_min, min(w_cap, W_MAX)) * s

        v_base = tnorm(MIN_V_TURN, V_MAX, V_BASE_MU * v_factor, V_BASE_SIG)
        v_cap_lat = A_LAT_MAX / max(abs(w_peak), 1e-6)
        v_arc = min(V_MAX, v_base, v_cap_lat)

        if v_arc < MIN_V_TURN:
            v_arc = MIN_V_TURN
            w_peak = np.sign(w_peak) * min(abs(w_peak), A_LAT_MAX / v_arc, W_MAX)

        self.w_peak = float(w_peak)
        self.v_arc  = float(v_arc)

    def start_new_episode(self):
        r = np.random.rand()
        if r < P_STRAIGHT:
            self.mode = "straight"
            self.episode_len = np.random.randint(STRAIGHT_LEN_MIN, STRAIGHT_LEN_MAX + 1)
            self.episode_tick = 0
            self._stop_weave()

        elif r < P_STRAIGHT + P_GENTLE:
            self.mode = "gentle"
            self._setup_arc(GENTLE_ANGLE_DEG, W_GENTLE_MAX, MIN_W_GENTLE, v_factor=0.85)

        else:
            self.mode = "aggressive"
            self._setup_arc(AGGR_ANGLE_DEG, W_AGGR_MAX, MIN_W_AGGR, v_factor=0.70)

    # --------------------------------------------------------
    def step(self):
        self.episode_tick += 1
        if self.episode_tick >= self.episode_len:
            if self.mode in ("gentle", "aggressive"):
                self.last_dir = -1 if self.w_peak < 0 else 1
            self.start_new_episode()

        if self.mode == "straight":
            v_target = tnorm(0, V_MAX, V_BASE_MU, V_BASE_SIG)
            self.w_bias += STRAIGHT_W_DRIFT_GAIN * (np.random.normal(0, STRAIGHT_W_DRIFT_SIG) - self.w_bias)
            w_target = self.w_bias + np.random.normal(0, STRAIGHT_W_NOISE_SIG)

            if (not self.weave_active) and (np.random.rand() < MICRO_WEAVE_PROB):
                self._start_weave()

            if self.weave_active:
                self.weave_tick += 1
                env = arc_envelope(self.weave_tick, self.weave_len)
                w_target += self.weave_w_peak * env
                if self.weave_tick >= self.weave_len:
                    self._stop_weave()

        elif self.mode in ("gentle", "aggressive"):
            env = arc_envelope(self.episode_tick, self.episode_len)
            w_target = self.w_peak * env
            v_target = max(self.v_arc, MIN_V_TURN)
            if self.mode == "gentle" and abs(w_target) < MIN_W_GENTLE * 0.6:
                w_target = np.sign(self.w_peak) * MIN_W_GENTLE * 0.6

        else:
            v_target = V_BASE_MU
            w_target = 0.0

        if abs(w_target) > 1e-6:
            v_target = min(v_target, A_LAT_MAX / abs(w_target), V_MAX)

        self.prev_v += LIN_SMOOTH * (v_target - self.prev_v)
        self.prev_v += np.random.normal(0, LIN_NOISE)
        self.prev_w += W_BLEND * (w_target - self.prev_w)

        v_cmd = float(np.clip(self.prev_v, 0, V_MAX))
        if v_cmd < NO_PIVOT_V:
            w_cmd = 0.0
        else:
            w_cmd = float(np.clip(self.prev_w, -W_MAX, W_MAX))

        self.w_hist[self.w_hist_i] = abs(w_cmd)
        self.w_hist_i = (self.w_hist_i + 1) % TURN_WATCHDOG_LEN

        if self.mode == "straight" and self.w_hist_i == 0:
            if self.w_hist.mean() < TURN_WATCHDOG_MINW:
                self.mode = "gentle"
                self._setup_arc(GENTLE_ANGLE_DEG, W_GENTLE_MAX, MIN_W_GENTLE, v_factor=0.85)

        return v_cmd, w_cmd


# ============================================================
# Run simulation and plot histograms
# ============================================================

sim = RandomWalkSim()
N = 100000  # simulate 20k timesteps (~1000 s)
vs = []
ws = []

for _ in range(N):
    v, w = sim.step()
    vs.append(v)
    ws.append(w)

vs = np.array(vs)
ws = np.array(ws)

fig, axes = plt.subplots(1, 2, figsize=(12, 5))

# Linear velocity histogram
axes[0].hist(vs, bins=40, density=True, color="steelblue", alpha=0.85)
axes[0].axvline(vs.mean(), linestyle="--", color="black", label=f"mean={vs.mean():.2f}")
axes[0].set_title("Random Walk Policy (Linear Velocity)")
axes[0].set_xlabel("Velocity (m/s)")
axes[0].set_ylabel("Density")
axes[0].legend()

# Angular velocity histogram
axes[1].hist(ws, bins=40, density=True, color="steelblue", alpha=0.85)
axes[1].axvline(ws.mean(), linestyle="--", color="black", label=f"mean={ws.mean():.2f}")
axes[1].set_title("Random Walk Policy (Angular Velocity)")
axes[1].set_xlabel("Velocity (rad/s)")
axes[1].set_ylabel("Density")
axes[1].legend()

plt.tight_layout()

plt.savefig("agro_gauss_random_walk_policy.png")
plt.show()
