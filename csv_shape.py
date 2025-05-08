import pandas as pd

# rosbag2_2025_05_02-15_39_18   Read the CSV file
df = pd.read_csv('rosbag2_2025_05_02-15_39_18/input_data/lidar_data.csv')
df1 = pd.read_csv('rosbag2_2025_05_02-15_39_18/input_data/local_goals.csv')
df2 = pd.read_csv('rosbag2_2025_05_02-15_39_18/input_data/cmd_vel_output.csv')
df3 = pd.read_csv('rosbag2_2025_05_02-15_39_18/input_data/odom_data.csv')
df4 = pd.read_csv('rosbag2_2025_05_02-15_39_18/input_data/odom_data.csv')
df5 = pd.read_csv('rosbag2_2025_05_02-15_39_18/input_data/cmd_vel_output.csv')

# Get the dimensions
num_rows, num_columns = df.shape

print(f"Number of rows: {num_rows}")
print(f"Number of columns: {num_columns}")
print(f"CSV dimensions: {num_rows} × {num_columns}")

num_rows, num_columns = df1.shape

print(f"Number of rows: {num_rows}")
print(f"Number of columns: {num_columns}")
print(f"CSV dimensions: {num_rows} × {num_columns}")


num_rows, num_columns = df2.shape

print(f"Number of rows: {num_rows}")
print(f"Number of columns: {num_columns}")
print(f"CSV dimensions: {num_rows} × {num_columns}")
print("BAG 2:")
# Get the dimensions
num_rows, num_columns = df3.shape

print(f"Number of rows: {num_rows}")
print(f"Number of columns: {num_columns}")
print(f"CSV dimensions: {num_rows} × {num_columns}")

num_rows, num_columns = df4.shape

print(f"Number of rows: {num_rows}")
print(f"Number of columns: {num_columns}")
print(f"CSV dimensions: {num_rows} × {num_columns}")


num_rows, num_columns = df5.shape

print(f"Number of rows: {num_rows}")
print(f"Number of columns: {num_columns}")
print(f"CSV dimensions: {num_rows} × {num_columns}")
