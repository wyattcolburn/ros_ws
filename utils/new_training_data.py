# One stop show scrip to turn a bag into lidar data
# Input : Bag file
# Output : odom_data .csv
#        : odom_data_lg.csv
#        : lidar_data.csv
#        : validation frames dkr
import numpy as np
import matplotlib.pyplot as plt
#from ros_csv import extract_messages, save_to_csv
import pandas as pd
from visual_fast import perp_circle, ray_trace, hall_csv, ray_trace_one, perp_circle_array
from visual_vid import draw_ray, draw_rays, generate_frames, generate_frames_obst
from path_distance import lg_distance

# only things one should have to change
#input_bag = "/home/wyattcolburn/ros_ws/utils/rosbag2_2025_02_12-10_17_50"
odom_csv_file = "long_odom.csv"
local_goal_file = "long_odom_lg.csv"
num_lg_goals = 20 # look into how A star does it
lidar_data_file = "dist_long_lidar.csv"
frame_dkr = "feb_19"
obstacle_radius = .2
obstacle_offset = .7

class Obstacle:
    def __init__(self, cx, cy, x_points, y_points):
        self.centerPoint = (cx,cy)
        self.x_points = x_points
        self.y_points = y_points

def local_goals(input_csv, output_csv, num_lg_goals):

    df = pd.read_csv(input_csv)
    print(len(df))

    timestamps = df['timestamp'].tolist() #add lg for plotjuggler
    odom_x = df['odom_x'].tolist()
    odom_y = df['odom_y'].tolist()
    odom_yaw = df['odom_yaw'].tolist()

    incrementor = len(timestamps) // num_lg_goals
    print(incrementor)
    local_goal_data = {'timestamp':timestamps[::incrementor],
            'odom_x': odom_x[::incrementor],
            'odom_y': odom_y[::incrementor],
            'odom_yaw':odom_yaw[::incrementor]
            }

    df_output = pd.DataFrame(local_goal_data)
    df_output.to_csv(output_csv, index=False)
def main():

    #save_to_csv(input_bag, odom_csv_file) # turned bag into csv
    local_goals_x, local_goals_y = lg_distance(odom_csv_file, "lg_dist.csv", .4, 0)
    #local_goals(odom_csv_file, local_goal_file, num_lg_goals)
    
    #extracting odom_data, local_goal_data

    df = pd.read_csv(odom_csv_file)

    # Extract only x and y positions
    odom_x = df['odom_x'].tolist()
    odom_y = df['odom_y'].tolist()

    #df_lg = pd.read_csv(local_goal_file)

    #local_goals_x = df_lg['odom_x'].tolist()
    #local_goals_y = df_lg['odom_y'].tolist()
    #local_goals_yaw = df_lg['odom_yaw'].tolist()

    # Convert yaw angles to unit vectors for quiver
    arrow_length = 0.1  # Adjust the arrow length as needed
    #dx = arrow_length * np.cos(local_goals_yaw)
    #dy = arrow_length * np.sin(local_goals_yaw)


    # Create figure
    plt.figure(figsize=(8, 6))

    # Plot odometry path (without yaw)
    plt.plot(odom_x, odom_y, marker='o', linestyle='-', markersize=3, color='blue', label="Odometry Path")

    # Plot local goals
    plt.plot(local_goals_x, local_goals_y, marker='o', linestyle='-', markersize=3, color='red', label="Local Goals")

    # Add arrows representing yaw at each local goal
    #plt.quiver(local_goals_x, local_goals_y, dx, dy, angles='xy', scale_units='xy', scale=1, color='black', label="Local Goals Yaw")


    # Add legend for the path
    plt.legend(loc="best")

    #obstacles = np.zeros((len(local_goals_x)*2, 640,2)) # each local goal gives two obstacles
    #obstacle_counter = 0
    #print(f"lenght of lcoal goals, {len(local_goals_x)}")
    # Labels and grid
    #plt.xlabel("X Position (m)")
    #plt.ylabel("Y Position (m)")
    #plt.title("Odometry Path Visualization")
    #plt.grid(True)
    
        
    print(len(local_goals_x))
        
    #for i in range(len(local_goals_x)-1): #dont need an obstacle for each odom, just local goals
        #obstacles, obstacle_counter = perp_circle((local_goals_x[i], local_goals_y[i]), (local_goals_x[i + 1], local_goals_y[i + 1]), obstacle_radius, obstacle_offset, obstacles, obstacle_counter)
    #print(len(obstacles))
    
    obstacleArray = []
    obstacleArrayCounter = 0
    for i in range(len(local_goals_x)-1): #dont need an obstacle for each odom, just local goals
        obstacleOne, obstacleTwo = perp_circle_array((local_goals_x[i], local_goals_y[i]), (local_goals_x[i + 1], local_goals_y[i + 1]), obstacle_radius, obstacle_offset)
        obstacleArray.append(obstacleOne)
        obstacleArray.append(obstacleTwo)
    lidar_readings = ray_trace_one(obstacleArray, odom_x, odom_y, local_goals_x) 
    generate_frames_obst(odom_x, odom_y, local_goals_x, local_goals_y, obstacleArray, obstacleArrayCounter, lidar_readings, "feb_19_test3") 
    #print(len(obstacles))
    #obstacle_counter = 0
    #for i in range(len(local_goals_x)-1):
    #    print(f"in the obstacle creator loop : {i}")
    #    local_obstacles = []
    #    obstacle_counter += 2
    #    print(obstacle_counter)
    #    ob_count =0
    #    local_obstacles, ob_count = perp_circle((local_goals_x[i], local_goals_y[i]), (local_goals_x[i + 1], local_goals_y[i + 1]), obstacle_radius, obstacle_offset, local_obstacles, ob_count)
    #    lidar_readings = []
    #    lidar_readings = ray_trace(local_obstacles, odom_x,odom_y, 2)
    #    print("lidar readings have been calculated")
    #    print(lidar_readings)
    #    generate_frames_obst(odom_x, odom_y, local_goals_x, local_goals_y,local_obstacles, obstacle_counter, lidar_readings, "draw_obstacles_2")
    #    print(f"done with frame {obstacle_counter}")
    plt.show()
    #test_odom_x = odom_x[0:99]
    #test_odom_y = odom_y[0:99]
    #lidar_readings = ray_trace(obstacles, odom_x, odom_y, local_goals_x) # give obstacle data and each odom point
    #hall_csv(lidar_readings, lidar_data_file)
    #print("data has been saved to lidar file")
    #generate_frames(odom_x, odom_y, local_goals_x, local_goals_y, dx, dy, lidar_readings, obstacles, obstacle_counter, frame_dkr) 
    #print("frames have been saved")
if __name__ == "__main__":
    main()
