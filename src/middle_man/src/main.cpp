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
#include "local_goal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


//header files
#include "obstacles.hpp"
#include "raytracing.hpp"
#include "mpc.hpp"
#include "local_goal.hpp"
using namespace std::chrono_literals;




class middleNode: public rclcpp::Node
{
public:
  middleNode() : Node("middle_node")
  {
    // Create a subscriber to the /odom topic.
    // The message type is std::msg:Float64MulitArray
    data_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/packetOut", 10,
      std::bind(&middleNode::data_callback, this, std::placeholders::_1));

	lg_subscriber_ = this->create_subscription<std_msgs::msg::String>(
			"local_goals", 10, std::bind(&middleNode::lg_subscriber_callback, this, std::placeholders::_1));
    // Create a timer that calls timer_callback() every 500ms.
    //timer_ = this->create_wall_timer(
	//		500ms, std::bind(&middleNode::timer_callback, this));
			}


private:
// Packet definition 
// doubles < odom_x, odom_y, odom_v, odom_w, need local_goals_x, local_goal_y, lidar data >
  Local_Goal_Manager local_goal_manager_;
  ObstacleManager obstacle_manager_;
  static constexpr size_t ODOM_FIELD_COUNT = 4;
  static constexpr size_t LIDAR_COUNT = 1080;
  double packetIn[ODOM_FIELD_COUNT + LIDAR_COUNT];

  double odom_x, odom_y, local_goal_x, local_goal_y, current_cmd_v, current_cmd_w;
  double lidar_ranges[LIDAR_COUNT];



  //output, will contain min of hallucinated lidar, and obstacle coordinates  
  static constexpr size_t OBSTACLE_COUNT = 8; //

  double packetOut[ODOM_FIELD_COUNT + LIDAR_COUNT + OBSTACLE_COUNT];

  // Callback function for the /packoutOut subscriber, main loop
  void data_callback(const std_msgs::msg::Float64MultiArray& packetIn){
	  RCLCPP_INFO(this->get_logger(), "RECEIVING SYNCED DATA");
	  local_goal_manager_.updateLocalGoal(); //update local goal, now need to add to output array
      proccessOdomLidar(packetIn);						  

	  return;
  }


  void lg_subscriber_callback(const std_msgs::msg::String::ConstSharedPtr msg)
  {
		if (local_goal_manager_.getLocalGoalVector().size() > 0)
		{
			RCLCPP_INFO(this->get_logger(), "Already filled obstacle array");
		}
		else 
		{
			splitString(msg);
			obstacle_manager_.local_goals_to_obs(local_goal_manager_);		
			std::cout << "Num of obstacles created    :" << obstacle_manager_.count << std::endl;
		}
  }

  void proccessOdomLidar(const std_msgs::msg::Float64MultiArray& packetIn)
  {
	  odom_x = packetIn[0];
	  odom_y = packetIn[1];
	  current_cmd_v = packetIn[2];
	  current_cmd_w = packetIn[3];
	  for (int lidar_counter = 0; lidar_counter < LIDAR_COUNT; lidar_counter ++){
		  lidar_ranges = packetIn[4+lidar_counter];
	  }
  }
  
  void processPacketOut()
  {
	packetOut[0] = odom_x;
	packetOut[1] = odom_y;
	packetOut[2] = current_cmd_v;
	packetOut[3] = current_cmd_w;

	return;

  }

  void splitString(const std_msgs::msg::String::ConstSharedPtr msg) 
  {
	  //This function is responsible for taking the message and creating a vector with all the local goal data. This will be used to create obstacle, but also feed into the network, should only be called once
	  

	  std::vector<std::string> obstacleData;
	  std::stringstream ss(msg->data.c_str());
	  std::string element;
      
	  std::vector<std::string> tokens;
	  while(std::getline(ss, element, '|')){

		  if (!element.empty()){
			  tokens.push_back(std::move(element));
		  }
	  }

	  if (tokens.size() % 3 !=0){
		  RCLCPP_INFO(this->get_logger(), "MAJOR ISSUE LOCAL GOALS");
		  return;
	  }
	  for (uint8_t lg_counter = 0; lg_counter < tokens.size(); lg_counter+=3) {
		  Local_Goal currentLG; //will constantly get written over, is that okay
		  try {
			  currentLG.x_point = std::stod(tokens[lg_counter]);
			  currentLG.y_point = std::stod(tokens[lg_counter + 1]);
			  currentLG.yaw = std::stod(tokens[lg_counter + + 2]);
		  }
		  catch(const std::exception &e){
		  RCLCPP_ERROR(rclcpp::get_logger("splitString"), "Conversion error: %s", e.what());
            continue;
		  }

		  local_goal_manager_.add_construct_lg(currentLG);
		}

	  RCLCPP_INFO(this->get_logger(), "SIZE OF local goals manager array %ld",
			  local_goal_manager_.getLocalGoalVector().size());
  }

  void printPacketOut() {
    // Calculate the total number of elements in the packetOut array.
    size_t totalElements = ODOM_FIELD_COUNT + LIDAR_COUNT;
    for (size_t i = 0; i < totalElements; ++i) {
      // Print each element to the console.
      std::cout << "packetOut[" << i << "] = " << packetOut[i] << std::endl;
    }
  }

  // Subscriber for /constant time synced data.
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr data_subscriber_;
  
  // Subscriber for /obstacle data
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lg_subscriber_;
  
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

