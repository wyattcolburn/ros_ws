#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LidarRecorder : public rclcpp::Node
{
public:
  LidarRecorder()
  : Node("lidar_recorder")
  {
    // Declare the topic name parameter
    this->declare_parameter<std::string>("lidar_topic", "/scan");

    // Get the parameter value
    std::string lidar_topic = this->get_parameter("lidar_topic").as_string();

    // Create a subscription to the LiDAR topic
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      lidar_topic, 10, std::bind(&LidarRecorder::lidar_callback, this, std::placeholders::_1));
  }

private:
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Process the LiDAR data here
    RCLCPP_INFO(this->get_logger(), "Received LiDAR scan with %zu ranges", msg->ranges.size());
  
  for (size_t i = 0; i < msg->ranges.size(); ++i){
	  if (std::isfinite(msg->ranges[i])){
		  RCLCPP_INFO(this->get_logger(), "Range[%zu]: %f", i, msg->ranges[i]);
	  }
  }
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarRecorder>());
  rclcpp::shutdown();
  return 0;
}

