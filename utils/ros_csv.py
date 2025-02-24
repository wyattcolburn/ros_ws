import rosbag2_py
import pandas as pd
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from dataclasses import asdict, is_dataclass
import sys
from collections import defaultdict
from scipy.spatial.transform import Rotation as R
import argparse
def extract_messages(bag_path):
    """Extract messages from a ROS 2 bag and store them in a dictionary grouped by timestamp."""
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    allowed_topics = {'/odom'}
    # Dictionary to group messages by timestamp
    grouped_data = defaultdict(dict)

    while reader.has_next():
        topic, msg, timestamp = reader.read_next()
        
        if topic not in allowed_topics:
            continue
        # Deserialize message
        msg_type = get_message(type_map[topic])
        msg_deserialized = deserialize_message(msg, msg_type)
        if topic == "/odom":
    # Extract x, y from position
            x = msg_deserialized.pose.pose.position.x
            y = msg_deserialized.pose.pose.position.y

            # Extract orientation quaternion and convert to yaw
            qx = msg_deserialized.pose.pose.orientation.x
            qy = msg_deserialized.pose.pose.orientation.y
            qz = msg_deserialized.pose.pose.orientation.z
            qw = msg_deserialized.pose.pose.orientation.w
            yaw = R.from_quat([qx, qy, qz, qw]).as_euler('xyz')[2]

            grouped_data.setdefault(timestamp, {}).update({
                "odom_x": x,
                "odom_y": y,
                "odom_yaw": yaw
            })
        elif topic == "/scan":
            # Convert LaserScan ranges to individual columns
            range_data = list(msg_deserialized.ranges)

            # Store each range as a separate column with an indexed key
            for i, value in enumerate(range_data):
                grouped_data.setdefault(timestamp, {}).update({
                    f"scan_range_{i}": value
                })
        elif topic == "/cmd_vel":
            v = msg_deserialized.linear.x
            w = msg_deserialized.angular.z
            print(f" v value {v}, w {w}")

            grouped_data.setdefault(timestamp, {}).update({
                "cmd_v": v, 
                "cmd_w": w
                })
            
    return grouped_data

def save_to_csv(bag_path, output_csv):
    """Converts extracted messages to CSV format."""
    messages = extract_messages(bag_path)

    if not messages:
        print("No messages found in the bag file.")
        return

    # Convert dictionary to Pandas DataFrame
    df = pd.DataFrame.from_dict(messages, orient="index")

    # Reset index to turn timestamp into a column
    df.reset_index(inplace=True)
    df.rename(columns={'index': 'timestamp'}, inplace=True)

    df.to_csv(output_csv, index=False)
    print(f"Saved {len(df)} messages to {output_csv}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="bag-->csv")
    parser.add_argument("input_file", type=str, help="Input file")
    parser.add_argument("output_file", type=str, help="Output file")
    args = parser.parse_args()
    bag_path = args.input_file 
    rclpy.init()
    #group_data = extract_messages(bag_path)
    save_to_csv(bag_path, args.output_file) 
    # Print only the first 5 timestamps and their messages
    #for timestamp, topics in list(group_data.items())[:5]:
    #    print(f"Timestamp: {timestamp}")
    #    for topic, message in topics.items():
    #        print(f"  {topic}: {message}")
    #    print("\n")  # Newline for readabilityu


    rclpy.shutdown()

