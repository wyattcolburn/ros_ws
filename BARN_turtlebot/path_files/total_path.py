import numpy as np



def path_coord_to_gazebo_coord(x, y):
    # Tis is from the jackal_timer github repo from dperille (UT-AUSTIN LAB)
    RADIUS=.075
    r_shift = -RADIUS - (30 * RADIUS * 2)
    c_shift = RADIUS + 5

    gazebo_x = x * (RADIUS * 2) + r_shift
    gazebo_y = y * (RADIUS * 2) + c_shift

    return (gazebo_x, gazebo_y)
print("path 100")
path = np.load('path_100.npy')
path_gazebo = []
for element in path:
    path_gazebo.append(path_coord_to_gazebo_coord(element[0], element[1]))

print(path_gazebo)
print("path 12")
path = np.load('path_12.npy')
path_gazebo = []
for element in path:
    path_gazebo.append(path_coord_to_gazebo_coord(element[0], element[1]))
print(path_gazebo)
