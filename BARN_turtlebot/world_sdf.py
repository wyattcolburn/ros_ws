import os
import glob

os.chdir('world_files')

world_files = glob.glob('*.world')

for file in world_files:
    basename = os.path.splitext(file)[0]

    new_file = f'{basename}.sdf'

    with open(file, 'r') as f:
        lines = f.readlines()

    if len(lines) > 1:
        lines[1] = lines[1].replace("<world_name='default'>", f"<world_name='{basename}'>")

    with open(new_file, 'w') as f:
        f.writelines(lines)

print("done")

