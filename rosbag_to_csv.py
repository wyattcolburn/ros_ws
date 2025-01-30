import rosbag2_py
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def read_rosbag(bag_path):
    """Read messages from a ROS 2 bag file and print them."""
    
    # Open the bag file
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Get all topics and their types
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    print(f"Reading bag file: {bag_path}")
    print(f"Topics found: {list(type_map.keys())}")
    
    bag_dict = defaultdict(dict)

    while reader.has_next(): # if there are more messages in the bag
        topic, msg, timestamp = reader.read_next() #read the next message, get three values, seconds in nano seconds
        # Deserialize message
        msg_type = get_message(type_map[topic])
        msg_deserialized = deserialize_message(msg, msg_type) # decode the message
        
        print(f"[{timestamp}] Topic: {topic} → {msg_deserialized}")

if __name__ == "__main__":
    bag_file = "rosbag2_2025_01_29-12_36_23"  # Change this to your bag file path
    rclpy.init()
    read_rosbag(bag_file)
    rclpy.shutdown()

