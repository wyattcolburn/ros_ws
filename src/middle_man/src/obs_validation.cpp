#include "local_goal.hpp"
#include "obstacles.hpp"
#include "nav_msgs/msg/path.hpp"

#ifdef PI
#undef PI
#endif
#include "raytracing.hpp"
#include "rclcpp/rclcpp.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


//Want to add halucinating obstacles
class obsValid: public rclcpp::Node
{ 
	public:
		obsValid() : Node("obs_valid") {

		data_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
		  "/packetOut", 10,
		  std::bind(&obsValid::data_callback, this, std::placeholders::_1));

		path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/plan", 10, std::bind(&obsValid::path_callback, this, std::placeholders::_1));
		
		scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&obsValid::scan_callback, this, std::placeholders::_1));
		
		marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 10);
		hall_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/HallScan", 10);
		
		packetOut_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("neuralNetInput", 10);
		tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

		}

    private:
		Local_Goal_Manager local_goal_manager_;
		ObstacleManager obstacle_manager_;
		rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;	
	    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr data_subscriber_;
	    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr hall_pub_;
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr packetOut_publisher_;
		std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
		

	    static constexpr size_t ODOM_FIELD_COUNT = 5;
	    static constexpr size_t LIDAR_COUNT = 640;
	    double packetOut[ODOM_FIELD_COUNT + LIDAR_COUNT + NUM_VALID_OBSTACLES *2];

	    double packetIn[ODOM_FIELD_COUNT + LIDAR_COUNT];
	    double odom_x, odom_y, local_goal_x, local_goal_y, yaw, current_cmd_v, current_cmd_w;

	    double real_lidar_ranges[LIDAR_COUNT];
	    double hall_lidar_ranges[LIDAR_COUNT];

		double map_x = 0;
		double map_y = 0;
		double map_yaw = 0;
	    
	void data_callback(const std_msgs::msg::Float64MultiArray& packetin){
		
		processOdomLidar(packetin);	
		std::cout << "**************** yaw value : " << yaw << std::endl;
		 
		geometry_msgs::msg::PoseStamped odom_pose;
		odom_pose.header.frame_id = "odom";
		odom_pose.header.stamp = rclcpp::Time(0);  // latest available transform
		odom_pose.pose.position.x = odom_x;
		odom_pose.pose.position.y = odom_y;
		odom_pose.pose.position.z = 0.0;

		// If you have yaw:
		tf2::Quaternion q;
		q.setRPY(0, 0, yaw);  // only yaw
		odom_pose.pose.orientation = tf2::toMsg(q);

		geometry_msgs::msg::PoseStamped map_pose;
		try {
			map_pose = tf_buffer_->transform(odom_pose, "map");

			// Transformed output
			map_x = map_pose.pose.position.x;
			map_y = map_pose.pose.position.y;

			// Extract yaw if needed
			tf2::Quaternion q_map;
			tf2::fromMsg(map_pose.pose.orientation, q_map);
			double roll, pitch, yaw;
			tf2::Matrix3x3(q_map).getRPY(roll, pitch, yaw);
			map_yaw = yaw;

		} catch (const tf2::TransformException &ex) {
			RCLCPP_ERROR(rclcpp::get_logger("raytracing"), "Failed to transform odom pose: %s", ex.what());
			return;
		}


		RCLCPP_INFO(this->get_logger(),
			"odom (%.2f, %.2f) -> map (%.2f, %.2f) yaw %.2f), map_yaw %.2f",
			odom_x, odom_y, map_x, map_y, yaw, map_yaw);


		  local_goal_manager_.updateLocalGoal(odom_x, odom_y); //local goals have already been converted to odom																							
		  obstacle_manager_.update_obstacles(local_goal_manager_);
		  
		  int num_obs;
		  auto obs_list = obstacle_manager_.get_active_obstacles(num_obs);
		  auto marker_array = make_markers(obs_list, static_cast<size_t>(num_obs));
		  marker_pub_->publish(marker_array);
		  
		  

		  map_compute_lidar_distances(map_x, map_y, map_yaw, LIDAR_COUNT, obstacle_manager_, hall_lidar_ranges, *tf_buffer_);
		 processPacketOut();	
				
		  int packetOut_size = sizeof(packetOut) / sizeof(packetOut[0]);
		  std_msgs::msg::Float64MultiArray msg;
		  msg.data.resize(packetOut_size);
		  RCLCPP_INFO(this->get_logger(), "SIZE OF packetOUt %d", packetOut_size);

		  RCLCPP_INFO(this->get_logger(), "Size of neural net output %zu", msg.data.size());
		  // Test with a simpler message
			if (!packetOut_publisher_) {
		RCLCPP_ERROR(this->get_logger(), "Publisher is null!");
	}
										   //
		  for (size_t i = 0; i < packetOut_size; ++i){
			  msg.data[i] = static_cast<double>(packetOut[i]);
		  }

		  RCLCPP_INFO(this->get_logger(), "HAVE SUCCESSFULLY COPIED THE MESSAGE");
		  packetOut_publisher_->publish(msg);
		  RCLCPP_INFO(this->get_logger(), "PUBLISHING NEURAL NET INPUT MESSAGE");
			return;
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


	void path_callback(const nav_msgs::msg::Path::ConstSharedPtr pathMsg) {
		if (local_goal_manager_.get_num_lg() > 0) {
			RCLCPP_INFO(this->get_logger(), "ALREADY HAVE LOCAL GOALS");
			return;
		}
		
		RCLCPP_INFO(this->get_logger(), "Received %zu poses in frame %s", 
					pathMsg->poses.size(), pathMsg->header.frame_id.c_str());
		
		// Get transform from map to odom
		geometry_msgs::msg::TransformStamped transform;
		try {
			transform = tf_buffer_->lookupTransform("odom", "map", tf2::TimePointZero);
			RCLCPP_INFO(this->get_logger(), "Successfully got transform from map to odom");
		} catch (tf2::TransformException &ex) {
			RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
			return;
		}
		
		for (size_t i = 0; i < pathMsg->poses.size(); i+=8) {
			// Transform position from map to odom frame
			geometry_msgs::msg::PoseStamped pose_in_map;
			geometry_msgs::msg::PoseStamped pose_in_odom;
			
			pose_in_map.header.frame_id = "map";
			pose_in_map.pose = pathMsg->poses[i].pose;
			
			try {
				tf2::doTransform(pose_in_map, pose_in_odom, transform);
				
				// Extract yaw from quaternion
				tf2::Quaternion q(
					pose_in_odom.pose.orientation.x,
					pose_in_odom.pose.orientation.y,
					pose_in_odom.pose.orientation.z,
					pose_in_odom.pose.orientation.w
				);
				double roll, pitch, yaw;
				tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
				
				// Add transformed goal to manager
				local_goal_manager_.add_local_goal(
					pose_in_odom.pose.position.x,
					pose_in_odom.pose.position.y, 
					yaw
				);
			} catch (tf2::TransformException &ex) {
				RCLCPP_ERROR(this->get_logger(), "Transform error on pose %zu: %s", i, ex.what());
				continue;  // Skip this goal and try the next
			}
		}
		
		RCLCPP_INFO(this->get_logger(), "Our current Odom position is %f %f", odom_x, odom_y);
		
		if (local_goal_manager_.get_num_lg() > 0) {
			RCLCPP_INFO(this->get_logger(), "FIRST GOAL VALUES ARE %f and %f", 
					local_goal_manager_.data_vector[0].x_point, 
					local_goal_manager_.data_vector[0].y_point);
			RCLCPP_INFO(this->get_logger(), "SIZE OF LOCAL_GOAL_VECTOR %d", 
					local_goal_manager_.get_num_lg());
			
			obstacle_manager_.local_goals_to_obs(local_goal_manager_); 
			local_goal_manager_.set_distance_vector(odom_x, odom_y);
		} else {
			RCLCPP_WARN(this->get_logger(), "No local goals were added after transformation");
		}
		
		return;
	}
	  void processOdomLidar(const std_msgs::msg::Float64MultiArray& packetIn)
	  {

		  //This is possible because of the statically defined packets
		  odom_x = packetIn.data[0];
		  odom_y = packetIn.data[1];
		  current_cmd_v = packetIn.data[2];
		  current_cmd_w = packetIn.data[3];
		  yaw = packetIn.data[4];
		  return;
	  }

		void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg){
				
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
				  hall_lidar_publish.push_back(static_cast<float>(hall_lidar_ranges[i]));
				  //RCLCPP_INFO(this->get_logger(), "Lidar value : %f", min_lidar_ranges[i]);
			  }
			  hall_msg.ranges = hall_lidar_publish;

			  hall_pub_->publish(hall_msg);
		      return;
		}

		visualization_msgs::msg::MarkerArray make_markers(const Obstacle* obs_list, size_t count) {
			visualization_msgs::msg::MarkerArray marker_array;

			for (size_t i = 0; i < count; ++i) {
				const Obstacle& ob = obs_list[i];
				geometry_msgs::msg::PointStamped odom_point;
				odom_point.header.frame_id = "odom";
				odom_point.header.stamp = rclcpp::Time(0);  // This is a valid ROS 2 zero timestamp

				odom_point.point.x = ob.center_x;
				odom_point.point.y = ob.center_y;

				geometry_msgs::msg::PointStamped map_point;
				try {
					map_point = tf_buffer_->transform(odom_point, "map");  
				} 
				catch (const tf2::TransformException &ex) {
					RCLCPP_ERROR(this->get_logger(), "Transform failed for obstacle %zu: %s", i, ex.what());
					continue;
				}

				visualization_msgs::msg::Marker marker;
				marker.header.frame_id = "map";  // Now using the map frame
				marker.header.stamp = this->now();
				marker.ns = "hallucinated_obstacles";
				marker.id = i;
				marker.type = visualization_msgs::msg::Marker::CYLINDER;
				marker.action = visualization_msgs::msg::Marker::ADD;

				marker.pose.position.x = map_point.point.x;
				marker.pose.position.y = map_point.point.y;
				marker.pose.position.z = 0.1;
				marker.pose.orientation.w = 1.0;

				marker.scale.x = ob.radius * 5;
				marker.scale.y = ob.radius * .5;
				marker.scale.z = 0.2;

				marker.color.r = 1.0;
				marker.color.g = 0.0;
				marker.color.b = 0.0;
				marker.color.a = 0.8;

				marker.lifetime = rclcpp::Duration::from_seconds(0.5);

				marker_array.markers.push_back(marker);
			}

			return marker_array;
		}
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<obsValid>());
    rclcpp::shutdown();
    return 0;
}

