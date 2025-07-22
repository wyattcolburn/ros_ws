import math
import numpy as np


def path_coord_to_gazebo_coord(x, y):
    # Tis is from the jackal_timer github repo from dperille (UT-AUSTIN LAB)
    RADIUS=.075
    r_shift = -RADIUS - (30 * RADIUS * 2)
    c_shift = RADIUS + 5

    gazebo_x = x * (RADIUS * 2) + r_shift
    gazebo_y = y * (RADIUS * 2) + c_shift

    return (gazebo_x, gazebo_y)

def yaw_calculation(x1, y1, x2, y2):

    return math.atan2(y2-y1, x2-x1)


path = np.load('BARN_turtlebot/path_files/path_0.npy')
print(path)
first_point = path_coord_to_gazebo_coord(path[1][0], path[1][1])
second_point = path_coord_to_gazebo_coord(path[2][0], path[2][1])

print(f"first point {first_point} second point {second_point}")
print(yaw_calculation(first_point[0], first_point[1], second_point[0], second_point[1]))
