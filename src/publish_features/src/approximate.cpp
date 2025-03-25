
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
*/


#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <vector>
//time sync library
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

using namespace std::chrono_literals;


class dataNode : public rclcpp::Node
{
public:
  dataNode() : Node("data_node")
  {
    // Create a subscriber to the /odom topic.
    // The message type is nav_msgs::msg::Odometry.
	rclcpp::QoS qos = rclcpp::QoS(10);
	Odom_sub.subscribe(this,"/odom", qos.get_rmw_qos_profile());
	LaserScan_sub.subscribe(this, "/scan", qos.get_rmw_qos_profile());


    // Create a publisher that publishes std_msgs::msg::String messages.
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("packetOut", qos);

	lg_subscriber_ = this->create_subscription<std_msgs::msg::String>(
			"local_goals", 10, std::bind(&dataNode::lg_subscriber_callback, this, std::placeholders::_1));

	uint32_t queue_size=10;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, nav_msgs::msg::Odometry> MySyncPolicy;
	sync_approx = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(queue_size), LaserScan_sub, Odom_sub);


	sync_approx->registerCallback(std::bind(&dataNode::SyncCallback, this, std::placeholders::_1,std::placeholders:: _2));

	}
    // Create a timer that calls timer_callback() every 500ms.
	//timer_ = this->create_wall_timer(
    //  500ms, std::bind(&dataNode::timer_callback, this));
  //}

private:

  static constexpr size_t ODOM_FIELD_COUNT = 4;
  static constexpr size_t LIDAR_COUNT = 1080;
  double packetOut[ODOM_FIELD_COUNT + LIDAR_COUNT];
	

	void SyncCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
		const nav_msgs::msg::Odometry::ConstSharedPtr & odom)
	{
	  //where all the code processing to make new packet should be
	    //RCLCPP_INFO(this->get_logger(), "Sync callback with %u and %u as times",
		 //   laser_scan->header.stamp.sec, odom->header.stamp.sec);

	    //odom_callback(odom);
	}


  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
      packetOut[0] = msg->pose.pose.position.x;
      packetOut[1] =msg->pose.pose.position.y;
	  packetOut[2] = msg->twist.twist.linear.x;
 	  packetOut[3] = msg->twist.twist.angular.z;
    
	  RCLCPP_INFO(
      this->get_logger(),
	  "Received /odom: position [x: %.2f, y: %.2f, odom_v : %.2f, odom_w : %.2f, odom_w], ",
	  packetOut[0],
	  packetOut[1],
	  packetOut[2],
	  packetOut[3]);
  }

  void lg_subscriber_callback(const std_msgs::msg::String::ConstSharedPtr msg)
  {
		RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  
  // Packet definition 
  // doubles < odom_x, odom_y, odom_v, odom_w, need local_goals_x, local_goal_y, lidar data >
/*  
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
*/
  // Subscriber for /odom.
  //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  // Subscriber for /laser_Scan
  //rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
  // Publisher for chatter.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lg_subscriber_;
  // Timer for periodic publishing
  
  rclcpp::TimerBase::SharedPtr timer_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> Odom_sub;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, nav_msgs::msg::Odometry> MySyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_approx;
  
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> LaserScan_sub;
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

