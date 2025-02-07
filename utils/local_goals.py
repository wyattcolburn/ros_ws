import pandas as pd
import argparse


def main():
    parser = argparse.ArgumentParser(description="create x amount of local_goals")
    parser.add_argument("input_file", type=str, help="Input file name")
    parser.add_argument("output_file", type=str, help="Output file name")
    parser.add_argument("num_lg", type=int, help="Num of local goals")

    args = parser.parse_args()

    input_file = args.input_file
    output_file = args.output_file
    num_of_local_goals = args.num_lg
    
    df = pd.read_csv(input_file)

    timestamps = df['timestamp'].tolist() #add lg for plotjuggler
    odom_x = df['odom_x'].tolist()
    odom_y = df['odom_y'].tolist()
    odom_yaw = df['odom_yaw'].tolist()

    incrementor = len(timestamps) // num_of_local_goals
    print(incrementor)
    local_goal_data = {'timestamp':timestamps[::incrementor],
            'odom_x': odom_x[::incrementor],
            'odom_y': odom_y[::incrementor],
            'odom_yaw':odom_yaw[::incrementor]
            }

    df_output = pd.DataFrame(local_goal_data)
    df_output.to_csv(output_file, index=False)
if __name__ == "__main__":
    main()

