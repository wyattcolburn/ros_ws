# This is to handle all the 1500 trials of MLP asymmetric, for results
# Input type is like worldx_timestamp1_timestamp2, we can split to just get
# worldx, but then need to create an output folder with world_x format



import os
from pathlib import Path
import sys
import shutil
input_dkr = "large_results"
folder = None
if os.path.exists(input_dkr):
    folders = sorted([f for f in os.listdir(input_dkr) if os.path.isdir(os.path.join(input_dkr, f))])
    print(folders)

for folder in folders:
    folder_path = os.path.join(input_dkr, folder) # we need this to be able to copy files into new folder 
    print(f"path for this folder is {folder_path}")
    print(folder.split('_')[0])
    world_num = folder.split('_')[0].removeprefix('world') # get the world number from the bag
    new_folder_path = os.path.join(input_dkr, f"world_{world_num}")
    os.makedirs(new_folder_path, exist_ok=True)
    print(new_folder_path)
    shutil.move(folder_path, new_folder_path)
