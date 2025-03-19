/*
 * The purpose of this node is to publish the feature data to the neural net plug in
 *
 * Packet Structure: Float64 Multi Array message size, because odom data is doubled
 *					 Lidar data is sized up from floats to doubles so that data can
 *					 all be published using one array/message. The array is static for 
 *					 performance reasons. 
 * Motivation: Lidar values are the min of hallucinated and real ones? 
 *			   
 *
 *Packet Declaration:
 * <odom_x, odom_y, odom_v, odom_w, lidar_ranges [1080 values]
 *
 *Improvements: Need to add the local values, not sure how we get those values, would like
 *			    to verify the syncronzation is working
 *
 *			    [200~https://github.com/ros-navigation/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/example_follow_path.py
 *
 *
 *
 *
 *
 *This node: takes the data published from publish_features node and applies to raytracing node?
*/


#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


//header files
#include "obstacles.hpp"
#include "raytracing.hpp"
#include "mpc.hpp"
using namespace std::chrono_literals;

class middleNode: public rclcpp::Node
{
public:
  middleNode() : Node("middle_node")
  {
    // Create a subscriber to the /odom topic.
    // The message type is std::msg:Float64MulitArray
    data_subscriber_ = this->create_subscription<std_msgs::msg::Float64MutliArray>(
      "/packetOut", 10,
      std::bind(&dataNode::data_callback, this, std::placeholders::_1));

    // Create a timer that calls timer_callback() every 500ms.
    timer_ = this->create_wall_timer(
      500ms, std::bind(&dataNode::timer_callback, this));
  }

private:
  // Packet definition 
  // doubles < odom_x, odom_y, odom_v, odom_w, need local_goals_x, local_goal_y, lidar data >
  
  static constexpr size_t ODOM_FIELD_COUNT = 4;
  static constexpr size_t LIDAR_COUNT = 1080;
  double packetIn[ODOM_FIELD_COUNT + LIDAR_COUNT];

  double odom_x, odom_y, local_goal_x, local_goal_y, current_cmd_v, current_cmd_w;
  double lidar_ranges[LIDAR_COUNT];



  //output, will contain min of hallucinated lidar, and obstacle coordinates  
  static constexpr size_t OBSTACLE_COUNT = 8; //

  double packetOut[ODOM_FIELD_COUNT + LIDAR_COUNT + OBSTACLE_COUNT];

  // Callback function for the /packoutOut subscriber
  void data_callback(const std_msgs::msg::Float64MultiArray packetIn)
  {
	  odom_x = packetIn[0];
	  odom_y = packetIn[1];
	  current_cmd_v = packetIn[2];
	  current_cmd_w = packetIn[3];
	  for (int lidar_counter = 0; lidar_counter < LIDAR_COUNT; lidar_counter ++){
		  lidar_ranges = packetIn[4+lidar_counter];
	  }
  }
  
  // Timer callback function for publishing messages.
  void timer_callback()
  {
	  std_msgs::msg::Float64MultiArray msg;
	  msg.data.resize(ODOM_FIELD_COUNT);
	  for (size_t i = 0; i <ODOM_FIELD_COUNT; ++i){
		  msg.data[i] = packetOut[i];
	  }
	  RCLCPP_INFO(this->get_logger(), "Publishing odom data");
    publisher_->publish(msg);
	printPacketOut();
  }
  void printPacketOut() {
    // Calculate the total number of elements in the packetOut array.
    size_t totalElements = ODOM_FIELD_COUNT + LIDAR_COUNT;
    for (size_t i = 0; i < totalElements; ++i) {
      // Print each element to the console.
      std::cout << "packetOut[" << i << "] = " << packetOut[i] << std::endl;
    }
  }

  // Subscriber for /odom.
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr data_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  // Initialize the ROS 2 system.
  rclcpp::init(argc, argv);
  // Create and spin the node.
  rclcpp::spin(std::make_shared<middleNode>());
  // Shutdown the ROS 2 system.
  rclcpp::shutdown();
  return 0;
}

