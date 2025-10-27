
import math
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from dataclasses import dataclass

# --------------------------- Data types ---------------------------

@dataclass
class Obstacle:
    center_x: float
    center_y: float
    radius: float

# --------------------------- Constants ---------------------------

OFFSET = 0.5    # baseline corridor half-width (perpendicular offset)
RADIUS = 0.15   # base obstacle radius

# --------------------------- Geometry utils ---------------------------

def _point_segment_distance(px, py, x1, y1, x2, y2):
    """Distance from point P to segment (x1,y1)-(x2,y2)."""
    vx, vy = x2 - x1, y2 - y1
    wx, wy = px - x1, py - y1
    L2 = vx*vx + vy*vy
    if L2 == 0.0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, (wx*vx + wy*vy) / L2))
    cx, cy = x1 + t*vx, y1 + t*vy
    return math.hypot(px - cx, py - cy)

def calculate_min_distance_to_path(px, py, path_points):
    """Minimum distance from (px,py) to a polyline path_points=[(x,y), ...]."""
    if len(path_points) == 0:
        return float("inf")
    if len(path_points) == 1:
        x0, y0 = path_points[0]
        return math.hypot(px - x0, py - y0)
    dmin = float("inf")
    for a, b in zip(path_points[:-1], path_points[1:]):
        dmin = min(dmin, _point_segment_distance(px, py, a[0], a[1], b[0], b[1]))
    return dmin

def signed_curvature_from_path(path_xy, i):
    """
    Signed curvature estimate at vertex i using heading change.
    +kappa: left bend, -kappa: right bend.
    path_xy: list of (x,y); i should be 1..len-2
    """
    n = len(path_xy)
    if n < 3:
        return 0.0
    i = max(1, min(n-2, i))
    x0, y0 = path_xy[i-1]
    x1, y1 = path_xy[i]
    x2, y2 = path_xy[i+1]
    h0 = math.atan2(y1 - y0, x1 - x0)
    h1 = math.atan2(y2 - y1, x2 - x1)
    # unwrap to (-pi, pi]
    d = (h1 - h0 + math.pi) % (2*math.pi) - math.pi
    L = math.hypot(x2 - x1, y2 - y1) + 1e-9
    return d / L

# --------------------------- Obstacle creators ---------------------------

def obstacle_creation_sym(p_curr, p_next, offset=OFFSET, radius=RADIUS):
    """Create two symmetric obstacles about the midpoint of segment p_curr->p_next."""
    xc, yc = p_curr
    xn, yn = p_next

    mid_x = 0.5 * (xc + xn)
    mid_y = 0.5 * (yc + yn)

    dir_x = xn - xc
    dir_y = yn - yc
    length = math.hypot(dir_x, dir_y)
    if length > 0:
        dir_x /= length
        dir_y /= length

    perp_x, perp_y = -dir_y, dir_x
    offset_x = perp_x * offset
    offset_y = perp_y * offset

    ob1 = Obstacle(mid_x + offset_x, mid_y + offset_y, radius)
    ob2 = Obstacle(mid_x - offset_x, mid_y - offset_y, radius)
    return ob1, ob2

def obstacle_creation_asymmetric(current_local_goal, next_local_goal, i, path_xy,
                                 OFFSET=OFFSET, RADIUS=RADIUS,
                                 skew_s_max=0.3, skew_gain=4.0,
                                 min_inside=0.40, max_outmul=1.80, widen_out=0.5,
                                 jitter_std=0.0, along_std=0.0,
                                 radius_range=(1.0, 1.0),
                                 drop_prob=0.0, clutter_prob=0.0, clutter_rad=0.4,
                                 rng=None):
    """
    Curvature-aware corridor:
      - Inside wall pulled in, outside wall pushed out w.r.t curvature.
      - Optional jitter and along-track scatter.
      - Enforces minimum clearance from the path polyline.
    Returns (Obstacle_in, Obstacle_out).
    """
    if rng is None:
        rng = random.Random(42)

    # --- segment geometry ---
    x1, y1 = current_local_goal
    x2, y2 = next_local_goal
    dx, dy = x2 - x1, y2 - y1
    L = math.hypot(dx, dy)
    if L == 0.0:
        # degenerate segment: place both at the point
        return Obstacle(x1, y1, RADIUS), Obstacle(x1, y1, RADIUS)

    dx, dy = dx / L, dy / L
    px, py = -dy, dx                       # left-facing unit normal
    mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0

    # --- curvature & skew ---
    kappa = signed_curvature_from_path(path_xy, i)
    skew  = skew_s_max * (1.0 - math.exp(-skew_gain * abs(kappa)))

    BASE = OFFSET
    off_in  = max(min_inside * BASE, BASE * (1.0 - skew))
    off_out = min(max_outmul * BASE, BASE * (1.0 + widen_out * skew))

    inside_s  = +1 if kappa > 0.0 else -1  # inside of a left turn is +normal
    outside_s = -inside_s

    def normal(std):
        if std == 0.0:
            return 0.0
        u1 = max(1e-9, rng.random())
        u2 = rng.random()
        return math.sqrt(-2.0 * math.log(u1)) * math.cos(2 * math.pi * u2) * std

    r_lo, r_hi = radius_range
    rad_in  = RADIUS * (r_lo + (r_hi - r_lo) * rng.random())
    rad_out = RADIUS * (r_lo + (r_hi - r_lo) * rng.random())

    cx_in  = mx + inside_s  * off_in  * px + normal(jitter_std) + normal(along_std) * dx
    cy_in  = my + inside_s  * off_in  * py + normal(jitter_std) + normal(along_std) * dy
    cx_out = mx + outside_s * off_out * px + normal(jitter_std) + normal(along_std) * dx
    cy_out = my + outside_s * off_out * py + normal(jitter_std) + normal(along_std) * dy

    # --- clearance enforcement (push away from path if too close) ---
    def enforce_clearance(cx, cy, radius):
        # at least 1.2*RADIUS or 1.1*radius, plus 3cm
        min_clear = max(radius * 1.10, RADIUS * 1.20) + 0.03
        d = calculate_min_distance_to_path(cx, cy, path_xy)
        need = min_clear - d
        if need > 0.0:
            # push along normal side based on which side of the segment center it lies
            sign = math.copysign(1.0, (px * (cx - mx) + py * (cy - my)))
            cx += sign * need * px
            cy += sign * need * py
        return cx, cy

    cx_in,  cy_in  = enforce_clearance(cx_in,  cy_in,  rad_in)
    cx_out, cy_out = enforce_clearance(cx_out, cy_out, rad_out)

    return Obstacle(cx_in, cy_in, rad_in), Obstacle(cx_out, cy_out, rad_out)

# --------------------------- Demo / main ---------------------------

def main():
    # A gentle left curve (same as you had)
    local_goals_x = [
    1.00, 1.04, 1.08, 1.12, 1.16, 1.20, 1.24, 1.28, 1.32, 1.36,
    1.40, 1.44, 1.48, 1.52, 1.56, 1.60, 1.64, 1.68, 1.72, 1.76, 1.80
    ]

    local_goals_y = [
        0.0000, 0.0013, 0.0050, 0.0113, 0.0200, 0.0312, 0.0450, 0.0612, 0.0800, 0.1012,
        0.1250, 0.1512, 0.1800, 0.2112, 0.2450, 0.2813, 0.3200, 0.3613, 0.4050, 0.4512, 0.5000
    ]
    path = list(zip(local_goals_x, local_goals_y))

    print(f"path is {path}")
    obs_list = []
    rng = random.Random(0)

    for i in range(len(path) - 1):
        print(f"value per func {path[i]} and next {path[i+1]}")
        # ob1, ob2 = obstacle_creation_asymmetric(
        #     path[i], path[i+1], i, path_xy=path,
        #     OFFSET=OFFSET, RADIUS=RADIUS,
        #     skew_s_max=0.3, skew_gain=4.0,
        #     min_inside=0.40, max_outmul=1.80, widen_out=0.5,
        #     jitter_std=0.0, along_std=0.0,
        #     radius_range=(1.0, 1.0),
        #     rng=rng
        # )
        ob1, ob2 = obstacle_creation_sym(path[i], path[i+1])
        obs_list.extend([ob1, ob2])

    # Plot
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(local_goals_x, local_goals_y, marker='o', linestyle='-',
            markersize=3, color='black', label='Odom path')

    for j, ob in enumerate(obs_list):
        circ = patches.Circle((ob.center_x, ob.center_y), ob.radius,
                              fill=False, linewidth=2, edgecolor='tab:orange',
                              label='Obstacle' if j == 0 else None)
        ax.add_patch(circ)

    ax.set_aspect('equal', adjustable='box')
    ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0),
              borderaxespad=0., frameon=True)
    fig.subplots_adjust(right=0.8)
    ax.set_xlabel("x-coordinate")
    ax.set_xlim(.8,2.4)
    ax.set_ylim(-.8,1)
    ax.set_ylabel("y-coordinate")
    # ax.set_title("Asymmetric Corridor Obstacles (curvature-aware)")
    # plt.savefig("Asymetric_Obstacle_Placement.jpg")
    ax.set_title("Symmetric Corridor Obstacles")
    plt.savefig("Symmetric_Obstacle_Placement.jpg")
    plt.show()

if __name__ == "__main__":
    main()
