import sys, subprocess

import os
from pathlib import Path

bags_dkr = Path('bags')

sub_dkr_list = os.listdir(bags_dkr)
print(sub_dkr_list)

result_list = []
for val in sub_dkr_list:
    if not val.find('results'):
        result_list.append(os.path.join(bags_dkr, val))

for directory in result_list:
    bags_within = os.listdir(directory)
    for world in bags_within:
        print(f"world is {world}")
        print(f"bag dkr : {directory}")
        
        world_path = os.path.join(directory, world)
        print(f"world path is : {world_path}")
        print(f"world : {world_path}")
        model =""
        if 'mlp_asym' in world_path:
            model = "MLP_ASYM" 
        elif 'mlp_sym' in world_path:
            model = "MLP_SYM" 
        elif 'cnn_sym' in world_path:
            model = "CNN_SYM" 
        else:
            model = "CNN_ASYM" 

        world_num = world.split('_')[1]
        model_type = model.split('_')[0]
        obstacle_type = model.split('_')[1]
        obstacle_title = ""
        if obstacle_type == "ASYM":
            obstacle_title = "Asymmetric Obstacle Placement"
        else: 
            obstacle_title = "Symmetric Obstacle Placement"

        cmd = [
            sys.executable, "plot_bag_overlay.py",
            "--bags", f"{world_path}/*", 
            "--odom", "/odom",
            "--plan", "/plan_barn",
            "--out", f"{bags_dkr}/{world}_{model}_overlay.png",
            "--title", f"All Odometry Overlays of World {world_num} for {model_type} with {obstacle_title}",
            "--map",  f"yaml_{world_num}.yaml"
        ]
        print(cmd)
        subprocess.run(cmd)
