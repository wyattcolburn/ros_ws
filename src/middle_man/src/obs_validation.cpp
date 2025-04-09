#include "local_goal.hpp"
#include "obstacles.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "std_msgs/msg/float64_multi_array.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class obsValid: public rclcpp::Node
{ 
	public:
		obsValid() : Node("obs_valid") {

		data_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
		  "/packetOut", 10,
		  std::bind(&obsValid::data_callback, this, std::placeholders::_1));

			path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/plan", 10, std::bind(&obsValid::path_callback, this, std::placeholders::_1));
			marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 10);

		}

    private:
		Local_Goal_Manager local_goal_manager_;
		ObstacleManager obstacle_manager_;
		rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;	
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
	    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr data_subscriber_;

	    static constexpr size_t ODOM_FIELD_COUNT = 4;
	    static constexpr size_t LIDAR_COUNT = 640;
	    double packetIn[ODOM_FIELD_COUNT + LIDAR_COUNT];

	    double odom_x, odom_y, local_goal_x, local_goal_y, current_cmd_v, current_cmd_w;

		void data_callback(const std_msgs::msg::Float64MultiArray& packetIn){
			
		  processOdomLidar(packetIn);	

			
		  local_goal_manager_.updateLocalGoal(odom_x, odom_y); 																							
		  obstacle_manager_.update_obstacles(local_goal_manager_);
		  int num_obs;
		  auto obs_list = obstacle_manager_.get_active_obstacles(num_obs);
		  auto marker_array = make_markers(obs_list, static_cast<size_t>(num_obs));
		  marker_pub_->publish(marker_array);
		}

		void path_callback(const nav_msgs::msg::Path::ConstSharedPtr pathMsg) {
		

			if (local_goal_manager_.get_num_lg() > 0) {
				RCLCPP_INFO(this->get_logger(), "ALREADY HAVE LOCAL GOALS");
				return;
			}
		

			RCLCPP_INFO(this->get_logger(), "Received %zu poses", pathMsg->poses.size());
			for (size_t i = 0; i < pathMsg->poses.size(); i+=4) {
				tf2::Quaternion q(
				pathMsg->poses[i].pose.orientation.x,
				pathMsg->poses[i].pose.orientation.y,
				pathMsg->poses[i].pose.orientation.z,
				pathMsg->poses[i].pose.orientation.w
				);
				double roll, pitch, yaw;
				tf2::Matrix3x3(q).getRPY(roll, pitch, yaw); 
				local_goal_manager_.add_local_goal(pathMsg->poses[i].pose.position.x,	pathMsg->poses[i].pose.position.y, yaw); 
			}

			RCLCPP_INFO(this->get_logger(), "SIZE OF LOCAL_GOAL_VECOTR %d", local_goal_manager_.get_num_lg());

			obstacle_manager_.local_goals_to_obs(local_goal_manager_); 
			return; 
		}

	  void processOdomLidar(const std_msgs::msg::Float64MultiArray& packetIn)
	  {

		  //This is possible because of the statically defined packets
		  odom_x = packetIn.data[0];
		  odom_y = packetIn.data[1];
		  current_cmd_v = packetIn.data[2];
		  current_cmd_w = packetIn.data[3];
		  return;
	  }


		visualization_msgs::msg::MarkerArray make_markers(const Obstacle* obs_list, size_t count) {
			visualization_msgs::msg::MarkerArray marker_array;

			for (size_t i = 0; i < count; ++i) {
				const Obstacle& ob = obs_list[i];

				visualization_msgs::msg::Marker marker;
				marker.header.frame_id = "map";
				marker.header.stamp = rclcpp::Clock().now();
				marker.ns = "hallucinated_obstacles";
				marker.id = i;
				marker.type = visualization_msgs::msg::Marker::CYLINDER;
				marker.action = visualization_msgs::msg::Marker::ADD;

				marker.pose.position.x = ob.center_x;
				marker.pose.position.y = ob.center_y;
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

