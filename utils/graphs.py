import os
import matplotlib.pyplot as plt
import pandas as pd

frame_dkr_input = "test1_input/_data/"
frame_dkr_output = "test1_output/_data"

csv_file_input = os.path.join(frame_dkr_input,"odom_data.csv")
csv_file_output = os.path.join(frame_dkr_output, "odom_data.csv")


input = pd.read_csv(csv_file_input)
output = pd.read_csv(csv_file_output)

odom_x_input = input['odom_x'].tolist()
odom_y_input = input['odom_y'].tolist()
odom_x_output= output['odom_x'].tolist()
odom_y_output = output['odom_y'].tolist()

plt.figure(figsize=(8, 6))

    ### Plot odometry path (without yaw)
plt.plot(odom_x_input, odom_y_input, marker='o', linestyle='-', markersize=3, color='blue', label="Odometry Path Input")

plt.plot(odom_x_output, odom_y_output, marker='o', linestyle='-', markersize=3, color='red', label="Odometry Path output")

plt.show()
