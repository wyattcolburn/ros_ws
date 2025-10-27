import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from dataclasses import dataclass

@dataclass
class Obstacle:
    center_x: float
    center_y: float
    radius: float

OFFSET = 0.5   # perpendicular offset distance
RADIUS = 0.15  # obstacle radius

def obstacle_creation_sym(p_curr, p_next, offset=OFFSET, radius=RADIUS):
    """Create two symmetric obstacles about the midpoint of segment p_curr->p_next."""
    xc, yc = p_curr
    xn, yn = p_next

    # Midpoint
    mid_x = 0.5 * (xc + xn)
    mid_y = 0.5 * (yc + yn)

    # Direction (unit) vector along the segment
    dir_x = xn - xc
    dir_y = yn - yc
    length = math.hypot(dir_x, dir_y)
    if length > 0:
        dir_x /= length
        dir_y /= length

    # Perpendicular unit vector
    perp_x = -dir_y
    perp_y =  dir_x

    # Offset from midpoint
    offset_x = perp_x * offset
    offset_y = perp_y * offset

    ob1 = Obstacle(mid_x + offset_x, mid_y + offset_y, radius)
    ob2 = Obstacle(mid_x - offset_x, mid_y - offset_y, radius)
    return ob1, ob2

def main():
    local_goals_x = [1, 1.2, 1.4, 1.6, 1.8]
    local_goals_y = [0, 0, 0, 0, 0]
    path = list(zip(local_goals_x, local_goals_y))

    obs_list = []
    for i in range(len(path) - 1):
        ob1, ob2 = obstacle_creation_sym(path[i], path[i+1])
        obs_list.extend([ob1, ob2])

    # Plot
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(local_goals_x, local_goals_y, marker='o', linestyle='-',
            markersize=3, color='black', label='Odom path')

    # Draw obstacles (label only first one so legend has a single entry)
    for j, ob in enumerate(obs_list):
        if j == 0:
            circ = patches.Circle((ob.center_x, ob.center_y), ob.radius,
                                  fill=False, linewidth=2, edgecolor='tab:orange',
                                  label='Obstacle')
        else:
            circ = patches.Circle((ob.center_x, ob.center_y), ob.radius,
                                  fill=False, linewidth=2, edgecolor='tab:orange')
        ax.add_patch(circ)
    ax.set_aspect('equal', adjustable='box')
    leg = ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0),
                    borderaxespad=0., frameon=True)
    fig.subplots_adjust(right=0.8)  # make room on the right
    plt.xlabel("x-coordinate")
    plt.ylabel("y-coordinate")
    plt.show()
    # ax.grid(True, linestyle='--', alpha=0.3)
    # ax.legend(loc='upper right')  # legend in top right
    # plt.show()

if __name__ == "__main__":
    main()
