#include <stdio.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
class Map:
{
	public:
	int map_array = [[10][10]]; //binary map
	int cost_per_edge = 10;
	float agent_radius = .5;


	private:
}


class multi_agent_planning: public rclcpp::Node
{
	public:
		multi_agent_planning(): Node("mutli_agent") {
		
			sub_ = this->create_subscriber<std_msgs::msg::float64_multi_array>("/agent_feedback", 10, std::bind(&multi_agent_planning::sub_callback, this, std::placeholder::_1));

			service_ = this->create_service<
	private:
		
		sub_callback(const std_msg::msg:float64_multi_array msg) {

			float init_x = msg[0];
			float init_y = msg[1];
		}
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<multi_agent_planning>());
    rclcpp::shutdown();
    return 0;
}
