import pandas as pd
import numpy as np

# Load CSV

def lg_distance(input_file, output_file, interval, next_target):
    df = pd.read_csv(input_file)

# Assuming columns ['x', 'y', 'z'] exist
    df['dx'] = df['odom_x'].diff()
    df['dy'] = df['odom_y'].diff()

# Euclidean distance between points
    df['step_distance'] = np.sqrt(df['dx']**2 + df['dy']**2)
# Compute cumulative distance
    df['cumulative_distance'] = df['step_distance'].cumsum().fillna(0)



# Store selected local goals
    local_goals_x = []
    local_goals_y = []

# Store points at regular distance intervals


    for i, row in df.iterrows():
        if row['cumulative_distance'] >= next_target:
            local_goals_x.append(row['odom_x'])
            local_goals_y.append(row['odom_y'])
            next_target += interval  # Update next target distance
    print(local_goals_x)

    # Create DataFrame
    goals_df = pd.DataFrame({'local_goals_x': local_goals_x, 'local_goals_y': local_goals_y})

# Save to CSV or display
    goals_df.to_csv(output_file, index=False)

    return local_goals_x, local_goals_y

