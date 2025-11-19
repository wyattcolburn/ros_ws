import sys, subprocess

import os
from pathlib import Path

bags_dkr = Path('dwa_baseline/')

sub_dkr_list = os.listdir(bags_dkr)
print(sub_dkr_list)

result_list = []
for val in sub_dkr_list:
    full_path = os.path.join(bags_dkr, val)
    # check: name contains 'world_' AND is a folder
    if 'world_' in val and os.path.isdir(full_path):
        result_list.append(full_path)
# for val in sub_dkr_list:
#     if not val.find('world_'):
#         result_list.append(os.path.join(bags_dkr, val))
    # result_list.append(os.path.join(bags_dkr, val))

print(f"result list includes {result_list}")
for directory in result_list:
    
    bags_within = sorted(os.listdir(directory))
    
    print(f"bag dkr : {directory}")
    
    # print(f"world is {world}")
    # world_path = os.path.join(directory, world)
    world_path = directory
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
    

    world_num = world_path.split('/')[1]   
    print(f"world num is {world_num}")
    world_num = world_num.split('_')[1]
    print(f"world num is {world_num}")
    # world_num=266
    model_type = model.split('_')[0]
    obstacle_type = model.split('_')[1]
    obstacle_type = "CNN"
    model_type = "MLP"
    obstacle_title = "DWA"
    # if obstacle_type == "ASYM":
    #     obstacle_title = "Asymmetric Obstacle Placement"
    # else: 
    #     obstacle_title = "Symmetric Noisy Obstacle Placement"

    cmd = [
        sys.executable, "using_different_bag_for_path_overlay.py",
        "--bags", f"{world_path}/*", 
        "--odom", "/odom",
        "--plan", "/plan_barn",
        "--out", f"{bags_dkr}/world_{world_num}_{obstacle_title}_overlay_old_path.png",
        # "--title", f"All Odometry Overlays of World {world_num} for {model_type} with {obstacle_title}",
        "--title", f"All Odometry Overlays of World {world_num} for {obstacle_title}",
        "--map",  f"yaml_{world_num}.yaml"
    ]
    print(cmd)
    subprocess.run(cmd)
    # sys.exit(0)

    # for world in bags_within:
    #     print(f"world is {world}")
    #     print(f"bag dkr : {directory}")
    #     
    #     # world_path = os.path.join(directory, world)
    #     world_path = directory
    #     print(f"world path is : {world_path}")
    #     print(f"world : {world_path}")
    #     model =""
    #     if 'mlp_asym' in world_path:
    #         model = "MLP_ASYM" 
    #     elif 'mlp_sym' in world_path:
    #         model = "MLP_SYM" 
    #     elif 'cnn_sym' in world_path:
    #         model = "CNN_SYM" 
    #     else:
    #         model = "CNN_ASYM" 
    #     
    #
    #     world_num = world_path.split('/')[2]   
    #     print(f"world num is {world_num}")
    #     world_num = world_num.split('_')[1]
    #     print(f"world num is {world_num}")
    #     # world_num=266
    #     model_type = model.split('_')[0]
    #     obstacle_type = model.split('_')[1]
    #     obstacle_type = "CNN"
    #     model_type = "MLP"
    #     obstacle_title = ""
    #     if obstacle_type == "ASYM":
    #         obstacle_title = "Asymmetric Obstacle Placement"
    #     else: 
    #         obstacle_title = "Symmetric Noisy Obstacle Placement"
    #
    #     cmd = [
    #         sys.executable, "using_different_bag_for_path_overlay.py",
    #         "--bags", f"{world_path}/*", 
    #         "--odom", "/odom",
    #         "--plan", "/plan_barn",
    #         "--out", f"{bags_dkr}/{world}_{model}_overlay_old_path.png",
    #         "--title", f"All Odometry Overlays of World {world_num} for {model_type} with {obstacle_title}",
    #         "--map",  f"yaml_{world_num}.yaml"
    #     ]
    #     print(cmd)
    #     subprocess.run(cmd)
    #     sys.exit(0)
