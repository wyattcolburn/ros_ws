
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

# Now iterate through them
for bag_path in "${BAG_PATHS[@]}"; do
    echo "Processing: $bag_path"
    # Your processing command here
    # python3 your_script.py --input_bag "$bag_path"
done
