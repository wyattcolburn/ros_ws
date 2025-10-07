import numpy as np
import os
from pathlib import Path


def path_coord_to_gazebo_coord(x, y):
    # Tis is from the jackal_timer github repo from dperille (UT-AUSTIN LAB)
    RADIUS = .075
    r_shift = -RADIUS - (30 * RADIUS * 2)
    c_shift = RADIUS + 5

    gazebo_x = x * (RADIUS * 2) + r_shift
    gazebo_y = y * (RADIUS * 2) + c_shift

    return (gazebo_x, gazebo_y)

def load_path_file(world_num):
    
    folder_path = Path("BARN_turtlebot/path_files")
    path_file = os.path.join(folder_path, f"path_{world_num}.npy")
    path_data = np.load(path_file)
    return path_data
#
# folder_path = Path("BARN_turtlebot/path_files")
#
# if not os.path.exists(folder_path):
#     print("path does not exist")
# else:
#     files_from_folder = sorted(os.listdir(folder_path))
#     print(files_from_folder)
#
#     for file in files_from_folder:
#
#         file_root, file_extension = os.path.splitext(file)
#         if file_extension == '.npy':
#             path_file = os.path.join(folder_path, file)
#             path_data = np.load(path_file)
#             path_map = [[0,0]] *path_data.shape[0]]
#             print(path_map)
#             for i, element in enumerate(path_data):
#                 map_coord = path_coord_to_gazebo_coord(path_data[i][0], path_data[i][1])
#                 path_map[i][0] = map_coord[0]
#                 path_map[i][1] = map_coord[1]
#
#             print(path_map)
#
#


