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

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include "local_goal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/path.hpp"

#include <algorithm> // for std::min

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
	auto qos2 = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal);
	rclcpp::QoS qos = rclcpp::QoS(10);
    data_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/packetOut", 10,
      std::bind(&middleNode::data_callback, this, std::placeholders::_1));

	lg_subscriber_ = this->create_subscription<std_msgs::msg::String>(
			"local_goals", 10, std::bind(&middleNode::lg_subscriber_callback, this, std::placeholders::_1));

	//packetOut_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("neuralNetInput", qos);
    packetOut_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("neuralNetInput", 10);
	
	
	path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/plan", 10, std::bind(&middleNode::plan_callback, this, std::placeholders::_1));

	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>( "scan", 10,
				std::bind(&middleNode::scan_sub_callback, this, std::placeholders::_1));
   
	hall_pub_= this->create_publisher<sensor_msgs::msg::LaserScan>("HallScan", qos2);
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
  static constexpr size_t LIDAR_COUNT = 640;
  double packetIn[ODOM_FIELD_COUNT + LIDAR_COUNT];

  double odom_x, odom_y, local_goal_x, local_goal_y, current_cmd_v, current_cmd_w;
  double real_lidar_ranges[LIDAR_COUNT];
  double hall_lidar_ranges[LIDAR_COUNT];
  double min_lidar_ranges[LIDAR_COUNT];
  
// This is to check if the robot hasnot moved, dont do all the expensive computations
  double prev_odom_x, prev_odom_y;
  bool first_callback = true;
  bool obstacle_callback = false;
  int local_goal_reached = 0;

  //output, will contain min of hallucinated lidar, and obstacle coordinates  
  static constexpr size_t OBSTACLE_COUNT = 30; //kk

  double packetOut[ODOM_FIELD_COUNT + LIDAR_COUNT + OBSTACLE_COUNT*2];

  // Callback function for the /packoutOut subscriber, main loop
  void data_callback(const std_msgs::msg::Float64MultiArray& packetIn){
	
	if (!obstacle_callback) {
		RCLCPP_INFO(this->get_logger(), "Have no received obstacle data");
		return;

	}
	  proccessOdomLidar(packetIn);						  
	RCLCPP_INFO_STREAM(this->get_logger(),
			   "odom_x: " << odom_y << "\n" <<
				"odom_y: " << odom_y << "\n"
               << "prev_odom_x: " << prev_odom_x << "\n"
               << "prev_odom_y: " << prev_odom_y);

	  if ((!first_callback) && (odom_x == prev_odom_x) && (odom_y == prev_odom_y)) {

		  RCLCPP_INFO(this->get_logger(), "ROBOT HAS NOT MOVED");
		  return;
	  }
	  prev_odom_x = odom_x;
	  prev_odom_y = odom_y;

	  if (first_callback) {
		  RCLCPP_INFO(this->get_logger(), "First callback has hit");
		  first_callback = false;
	  }


	  RCLCPP_INFO(this->get_logger(), "procssed Odom lidar");
	  local_goal_reached = local_goal_manager_.updateLocalGoal(odom_x, odom_y); //If return 1, local goal was updated, need to update obstacles then, if 0, same obstacles
	  RCLCPP_INFO(this->get_logger(), "update local goal");
	obstacle_manager_.update_obstacles(local_goal_manager_);
//	  RCLCPP_INFO(this->get_logger(), "update obstacles");
	  RCLCPP_INFO(this->get_logger(), "have finished updating the obstacles");
      //test(obstacle_manager_);
	  compute_lidar_distances(odom_x, odom_y, LIDAR_COUNT, obstacle_manager_, &hall_lidar_ranges[0]); //compute the fake lidar reading 
	  RCLCPP_INFO(this->get_logger(), "compute lidar distances");
	  min_lidar();
	  //RCLCPP_INFO(this->get_logger(), "min lidar");
	  processPacketOut();
	  RCLCPP_INFO(this->get_logger(), "create output packet");

	  int packetOut_size = sizeof(packetOut) / sizeof(packetOut[0]);
	  std_msgs::msg::Float64MultiArray msg;
	  msg.data.resize(packetOut_size);
	  RCLCPP_INFO(this->get_logger(), "SIZE OF packetOUt %d", packetOut_size);

	  RCLCPP_INFO(this->get_logger(), "Size of neural net output %zu", msg.data.size());
	  // Test with a simpler message
		if (!packetOut_publisher_) {
    RCLCPP_ERROR(this->get_logger(), "Publisher is null!");
    return;
}
									   //
	  for (size_t i = 0; i < packetOut_size; ++i){
		  msg.data[i] = static_cast<double>(packetOut[i]);
	  }

	  RCLCPP_INFO(this->get_logger(), "HAVE SUCCESSFULLY COPIED THE MESSAGE");
	  packetOut_publisher_->publish(msg);
	  RCLCPP_INFO(this->get_logger(), "PUBLISHING NEURAL NET INPUT MESSAGE");

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
			std::cout << "Num of obstacles created    :" << obstacle_manager_.obstacle_count << std::endl;
			int num_obs; 
			const Obstacle* obstacle_list = obstacle_manager_.get_active_obstacles(num_obs);

			for (int i = 0; i < num_obs; i++) {
				std::cout << "obs coords : " << obstacle_list[i].center_x << "  "<< obstacle_list[i].center_y << std::endl;
			}
			obstacle_callback = true;
		}
  }
  void plan_callback(const nav_msgs::msg::Path::ConstSharedPtr pathMsg){

	//need to take in the message and make sure it is going to local_manager, so that I can create obstacles as well
	// Then need to add logic to make sure this only ones once?? multiple paths?
	// Need to create obstacles -->, then start normal operation of hallucinating lidar
	//
	// Need to validate then I am reaching next local goal
	  RCLCPP_INFO(this->get_logger(), "HITTING PATH CALLBACK");

	  RCLCPP_INFO(this->get_logger(), "Path data %zu poses", pathMsg->poses.size());
      std::vector<Local_Goal> local_goal_vec;

	  for (size_t i = 0; i < pathMsg->poses.size(); i++) {
		  Local_Goal currentLG;
		  currentLG.x_point = pathMsg->poses[i].pose.position.x;
		  currentLG.y_point = pathMsg->poses[i].pose.position.y;
		  currentLG.yaw = tf2::getYaw(pathMsg->poses[i].pose.orientation);
		  local_goal_vec.push_back(currentLG);
  
	  }
  }
  void scan_sub_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
  {
      //This function is to get the scan message and replicate the message but with min values, 
	  //aka what will feed into the neural network. Publishing under "/HallScan"
	  sensor_msgs::msg::LaserScan hall_msg;

	  hall_msg.header = scanMsg->header;
	  hall_msg.angle_min = scanMsg->angle_min;
	  hall_msg.angle_max = scanMsg->angle_max;
	  hall_msg.angle_increment = scanMsg->angle_increment;
	  hall_msg.time_increment = scanMsg->time_increment;
	  hall_msg.scan_time = scanMsg->scan_time;
	  hall_msg.range_min = scanMsg->range_min;
	  hall_msg.range_max = scanMsg->range_max;
	  std::vector<float>hall_lidar_publish;
	  for (int i = 0; i < LIDAR_COUNT; i++) {
		  hall_lidar_publish.push_back(static_cast<float>(min_lidar_ranges[i]));
		  //RCLCPP_INFO(this->get_logger(), "Lidar value : %f", min_lidar_ranges[i]);
	  }
	  hall_msg.ranges = hall_lidar_publish;

       hall_pub_->publish(hall_msg);
	//RCLCPP_INFO(this->get_logger(), "Publishing fake lidar");
		return;
  }
  void min_lidar(){

	  for (int lidar_compare_counter = 0; lidar_compare_counter < LIDAR_COUNT; lidar_compare_counter++) {

		  min_lidar_ranges[lidar_compare_counter] = std::min(real_lidar_ranges[lidar_compare_counter], hall_lidar_ranges[lidar_compare_counter]);

	  }
	  return;
  }
  void proccessOdomLidar(const std_msgs::msg::Float64MultiArray& packetIn)
  {

	  //This is possible because of the statically defined packets
	  odom_x = packetIn.data[0];
	  odom_y = packetIn.data[1];
	  current_cmd_v = packetIn.data[2];
	  current_cmd_w = packetIn.data[3];
	  for (int lidar_counter = 0; lidar_counter < LIDAR_COUNT; lidar_counter ++){
		  real_lidar_ranges[lidar_counter] = packetIn.data[4+lidar_counter];
	  }
  }
  
  void processPacketOut()
  {
	packetOut[0] = odom_x;
	packetOut[1] = odom_y;
	packetOut[2] = current_cmd_v;
	packetOut[3] = current_cmd_w;

	for (int lidar_counter = 0; lidar_counter < LIDAR_COUNT; lidar_counter++){
		packetOut[4+lidar_counter] = hall_lidar_ranges[lidar_counter];
	}

	//filled with min lidar data
	int num_obstacles;
	const Obstacle* current_obstacles = obstacle_manager_.get_active_obstacles(num_obstacles);
		
	for (int local_obstacle_counter = 0; local_obstacle_counter < num_obstacles; local_obstacle_counter++) {
		int index = ODOM_FIELD_COUNT + LIDAR_COUNT + local_obstacle_counter*2;
		packetOut[index]= current_obstacles[local_obstacle_counter].center_x;
		packetOut[index+1]= current_obstacles[local_obstacle_counter].center_y;
	}
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
 
  //Subscriber to real lidar scan
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr packetOut_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr hall_pub_;

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

