
#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

# ---------------- Turnier profile (no pivot) ----------------
V_MAX          = 0.30      # m/s hard cap
W_MAX          = 1.40      # rad/s hard cap
DT             = 0.05      # 20 Hz

NO_PIVOT_V     = 0.05      # if v < this, w = 0 (no in-place spin)
MIN_V_TURN     = 0.10      # guarantee some forward motion in arcs
MIN_W_ARC      = 0.18      # minimum |w| during arcs (if v >= NO_PIVOT_V)

A_LAT_MAX      = 1.10      # m/s^2 lateral accel cap

# Baseline forward velocity
V_BASE_MU      = 0.22
V_BASE_SIG     = 0.05
LIN_SMOOTH     = 0.25
LIN_NOISE      = 0.03

# Arc episode controls
ARC_START_PROB = 0.35
ARC_LEN_MIN    = int(1.2 / DT)    # 1.2 s
ARC_LEN_MAX    = int(3.0 / DT)    # 3.0 s
K_PEAK_MU      = 1.35             # mean peak curvature (1/m)
K_PEAK_SIG     = 0.50
K_PEAK_MIN     = 0.40
K_PEAK_MAX     = 3.00
ARC_INTERNAL_NOISE = 0.05
ALTERNATE_WEIGHT   = 0.65

# Cooldown between arcs
COOLDOWN_MIN   = int(0.8 / DT)    # 0.8 s
COOLDOWN_MAX   = int(1.6 / DT)    # 1.6 s

# Non-arc mild wandering curvature
NONARC_K_SIG   = 0.12


# ---------- helpers (no SciPy) ----------

def tnorm(a, b, mu, sigma):
    """Cheap truncated normal: clip a normal sample into [a, b]."""
    x = np.random.normal(mu, sigma)
    return float(np.clip(x, a, b))


def arc_envelope(t_idx, t_total):
    """Sine envelope so curvature ramps up and down smoothly."""
    if t_total <= 1:
        return 1.0
    p = t_idx / float(t_total - 1)
    return float(np.sin(np.pi * p))


# ---------- simulator version of the node ----------

class TurnierRandomWalkSim:
    def __init__(self):
        self.prev_v = 0.0
        self.prev_w = 0.0

        self.in_arc = False
        self.arc_len = 0
        self.arc_tick = 0
        self.kappa_peak = 0.0
        self.last_dir = 0

        self.cooldown = 0

    def choose_arc_dir(self):
        """Alternate left/right with probability ALTERNATE_WEIGHT."""
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
        """Advance one 50 ms step and return (v_cmd, w_cmd)."""
        # --- arc timing & state machine ---
        if self.cooldown > 0:
            self.cooldown -= 1

        if self.in_arc:
            self.arc_tick += 1
            if self.arc_tick >= self.arc_len:
                self.end_arc()
        else:
            if self.cooldown == 0 and np.random.rand() < ARC_START_PROB:
                self.start_arc()

        # --- curvature & base velocity ---
        if self.in_arc:
            k_env = arc_envelope(self.arc_tick, self.arc_len)
            k_target = self.kappa_peak * k_env + np.random.normal(0.0, ARC_INTERNAL_NOISE)
            v_base  = tnorm(0.0, V_MAX, V_BASE_MU * 0.90, V_BASE_SIG)
            v_base  = max(v_base, MIN_V_TURN)
        else:
            k_target = np.random.normal(0.0, NONARC_K_SIG)
            v_base  = tnorm(0.0, V_MAX, V_BASE_MU, V_BASE_SIG)

        # --- lateral acceleration bound → speed cap ---
        if abs(k_target) > 1e-6:
            v_cap_lat = np.sqrt(max(0.0, A_LAT_MAX / abs(k_target)))
        else:
            v_cap_lat = V_MAX

        v_target = min(v_base, v_cap_lat, V_MAX)
        if self.in_arc:
            v_target = max(v_target, MIN_V_TURN)

        # --- curvature → angular velocity, then smoothing ---
        w_target = float(np.clip(k_target * v_target, -W_MAX, W_MAX))

        self.prev_v += LIN_SMOOTH * (v_target - self.prev_v)
        self.prev_v += np.random.normal(0.0, LIN_NOISE)

        # slightly slower blend than the other policy, like your node
        self.prev_w += 0.60 * (w_target - self.prev_w)

        # --- no-pivot rule + min |ω| in arcs ---
        v_cmd = float(np.clip(self.prev_v, 0.0, V_MAX))
        if v_cmd < NO_PIVOT_V:
            w_cmd = 0.0
        else:
            w_cmd = float(np.clip(self.prev_w, -W_MAX, W_MAX))
            if self.in_arc and abs(w_cmd) < MIN_W_ARC:
                # if sign is still zero, use kappa_peak
                base_sign = w_cmd if w_cmd != 0.0 else self.kappa_peak
                w_cmd = np.sign(base_sign) * MIN_W_ARC

        return v_cmd, w_cmd


# ---------- run simulation & plot ----------

def main():
    sim = TurnierRandomWalkSim()
    N = 20000  # 1000 seconds at 20 Hz

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
    axes[0].hist(vs, bins=40, density=True, alpha=0.85)
    axes[0].axvline(vs.mean(), linestyle="--", label=f"mean={vs.mean():.2f}")
    axes[0].set_title("Turnier Random Walk (Linear Velocity)")
    axes[0].set_xlabel("Velocity (m/s)")
    axes[0].set_ylabel("Density")
    axes[0].legend()

    # Angular velocity histogram
    axes[1].hist(ws, bins=40, density=True, alpha=0.85)
    axes[1].axvline(ws.mean(), linestyle="--", label=f"mean={ws.mean():.2f}")
    axes[1].set_title("Turnier Random Walk (Angular Velocity)")
    axes[1].set_xlabel("Velocity (rad/s)")
    axes[1].set_ylabel("Density")
    axes[1].legend()

    plt.tight_layout()
    plt.savefig("random_walk_policy_2.png")
    plt.show()


if __name__ == "__main__":
    main()
