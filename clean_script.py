import pandas as pd
import numpy as np

# File paths
input_file = "/home/wyattcolburn/ros_ws/lidar_data.csv"
output_file = "/home/wyattcolburn/ros_ws/lidar_data_cleaned.csv"

# Read CSV, removing extra quotes if they exist
with open(input_file, "r") as f:
    lines = f.readlines()

# Remove quotes from each line and save back
clean_lines = [line.replace('"', '') for line in lines]
with open(output_file, "w") as f:
    f.writelines(clean_lines)

# Load cleaned CSV
df = pd.read_csv(output_file)

# Detect timestamp column (assuming it's the first column)
timestamp_col = df.columns[0]

# Convert timestamps to proper datetime format (assuming Unix time in seconds)
df[timestamp_col] = pd.to_datetime(df[timestamp_col], unit='s', errors='coerce')

# Replace `inf`, `-inf`, and invalid values with NaN
df.replace([np.inf, -np.inf], np.nan, inplace=True)

# Drop rows with NaN timestamps
df.dropna(subset=[timestamp_col], inplace=True)

# Save final cleaned data
df.to_csv(output_file, index=False)


