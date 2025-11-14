
import yaml

# Worlds to skip

# Load YAML
with open("config.yaml", "r") as f:
    data = yaml.safe_load(f)

# Generate full world list
data["WORLD_NUMS"] = [i for i in range(300)]

# Save YAML back
with open("config.yaml", "w") as f:
    yaml.safe_dump(data, f, sort_keys=False)

