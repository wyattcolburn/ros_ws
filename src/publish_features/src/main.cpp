/*
 * The purpose of this node is to publish the feature data to the neural net plug in
 *
 * Packet Structure: Float64 Multi Array message size, because odom data is doubled
 *					 Lidar data is sized up from floats to doubles so that data can
 *					 all be published using one array/message. The array is static for 
 *					 performance reasons. 
 *
 *Packet Declaration:
 * <odom_x, odom_y, odom_v, odom_w, lidar_ranges [1080 values]
 *
 *Improvements: Need to add the local values, not sure how we get those values, would like
 *			    to verify the syncronzation is working
*/


#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class dataNode : public rclcpp::Node
{
public:
  dataNode() : Node("data_node")
  {
    // Create a subscriber to the /odom topic.
    // The message type is nav_msgs::msg::Odometry.
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&dataNode::odom_callback, this, std::placeholders::_1));
	lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
			"/scan", 10,
			std::bind(&dataNode::lidar_callback, this, std::placeholders::_1));

    // Create a publisher that publishes std_msgs::msg::String messages.
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("packetOut", 10);

    // Create a timer that calls timer_callback() every 500ms.
    timer_ = this->create_wall_timer(
      500ms, std::bind(&dataNode::timer_callback, this));
  }

private:
  // Packet definition 
  // doubles < odom_x, odom_y, odom_v, odom_w, need local_goals_x, local_goal_y, lidar data >
  
  static constexpr size_t ODOM_FIELD_COUNT = 4;
  static constexpr size_t LIDAR_COUNT = 1080;
  double packetOut[ODOM_FIELD_COUNT + LIDAR_COUNT];


  // Callback function for the /odom subscriber.
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
      packetOut[0] = msg->pose.pose.position.x,
      packetOut[1] =msg->pose.pose.position.y,
	  packetOut[2] = msg->twist.twist.linear.x,
 	  packetOut[3] = msg->twist.twist.angular.z;
    
	  RCLCPP_INFO(
      this->get_logger(),
      "Received /odom: position [x: %.2f, y: %.2f, odom_v : %.2f, odom_w : %.2f, odom_w], ",
	  packetOut[0],
	  packetOut[1],
	  packetOut[2],
	  packetOut[3]);
  }

  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
	  //printing out how many ranges there are to verify sim working 
	  size_t num_ranges = msg->ranges.size();
	
	  std::cout << "Type of ranges:     " << typeid(msg->ranges[0]).name() << std::endl;
	  size_t offset = ODOM_FIELD_COUNT;
	  //upscaling floats to doubles
	  std::transform(msg->ranges.begin(), msg->ranges.end(), packetOut + offset,
			[](float value) -> double { return static_cast<double>(value); });
	  RCLCPP_INFO(
		this->get_logger(),
		"Received /laserscan: num_ranges: %ld",
		num_ranges);

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
  private:
  void printPacketOut() {
    // Calculate the total number of elements in the packetOut array.
    size_t totalElements = ODOM_FIELD_COUNT + LIDAR_COUNT;
    for (size_t i = 0; i < totalElements; ++i) {
      // Print each element to the console.
      std::cout << "packetOut[" << i << "] = " << packetOut[i] << std::endl;
    }
  }

  // Subscriber for /odom.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  // Subscriber for /laser_Scan
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
  // Publisher for chatter.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  // Timer for periodic publishing.
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  // Initialize the ROS 2 system.
  rclcpp::init(argc, argv);
  // Create and spin the node.
  rclcpp::spin(std::make_shared<dataNode>());
  // Shutdown the ROS 2 system.
  rclcpp::shutdown();
  return 0;
}

