
import numpy as np
from scipy.stats import truncnorm
import matplotlib.pyplot as plt

v_min, v_max = 0.0, 0.4
w_min, w_max = -1.4, 1.4

def truncated_gaussian(min_val, max_val, mu, sigma, size=1000):
    a, b = (min_val - mu) / sigma, (max_val - mu) / sigma
    return truncnorm.rvs(a, b, loc=mu, scale=sigma, size=size)

N = 1000000

# Normal mode
v_normal = truncated_gaussian(v_min, v_max, 0.2, 0.1, N)
w_normal = truncated_gaussian(w_min, w_max, 0.0, 0.6, N)

# Challenge mode
v_challenge = truncated_gaussian(0.05, v_max, 0.15, 0.12, N)
w_challenge = truncated_gaussian(w_min, w_max, 0.0, 1.0, N)

# Plot
fig, axes = plt.subplots(2, 2, figsize=(10, 8))
titles = [
    "Normal Mode (Linear Velocity)",
    "Normal Mode (Angular Velocity)",
    "Challenge Mode (Linear Velocity)",
    "Challenge Mode (Angular Velocity)"
]
data = [v_normal, w_normal, v_challenge, w_challenge]
limits = [(v_min, v_max), (w_min, w_max), (v_min, v_max), (w_min, w_max)]

for ax, d, t, (lo, hi) in zip(axes.flat, data, titles, limits):
    ax.hist(d, bins=40, range=(lo, hi), density=True, alpha=0.6, color='steelblue')
    ax.axvline(np.mean(d), color='red', linestyle='--', label=f"mean={np.mean(d):.2f}")
    ax.set_title(t)
    ax.set_xlim(lo, hi)
    ax.set_xlabel('Velocity (m/s or rad/s)')
    ax.set_ylabel('Density')
    ax.legend()

plt.tight_layout()
plt.savefig("random_walk_policy.png")
plt.show()
