
#!/bin/bash

# Read the directory path from YAML
INPUT_BAG_DKR=$(yq '.TRAINING_DKR' config.yaml)

# Create array of all subdirectories
BAG_PATHS=()
while IFS= read -r -d '' dir; do
    BAG_PATHS+=("$dir")
done < <(find "$INPUT_BAG_DKR" -mindepth 1 -maxdepth 1 -type d -print0)

# Print all directories found
echo "Found ${#BAG_PATHS[@]} directories in $INPUT_BAG_DKR:"
for i in "${!BAG_PATHS[@]}"; do
    echo "  [$((i+1))] ${BAG_PATHS[$i]}"
done
for bag_path in "${BAG_PATHS[@]}"; do
    echo "Processing: $(basename "$bag_path")"
    
    # Pass the bag path to your Python script
    ros2 run my_robot_bringup old_multiple --input_bag "$bag_path"
    
    if [ $? -eq 0 ]; then
        echo "Successfully processed: $(basename "$bag_path")"
    else
        echo "Error processing: $(basename "$bag_path")"
    fi
done
# Now iterate through them
done
