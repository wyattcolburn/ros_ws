#ifndef MPC_HPP
#define MPC_HPP
#include "obstacles.hpp"
#include <cmath>
#include <iostream>
#include <random>

const float W1 = .4;
const float W2 = 1.0;
const float TIMESTEP = .05;
const float TURTLEBOT_RADIUS = 10;//342 x 339 mm
float gaussian(float input_val, float mean, float std_dev);

bool circles_intersect(const Obstacle& c1, const Obstacle& c2);

std::pair<float, float> modulation(float odom_x, float odom_y,
		float input_cmd_v, float input_cmd_w, ObstacleManager& local_manager, Obstacle* future_points);

std::pair<float, float> modulation_onnx(float odom_x, float odom_y,
		float input_cmd_v, float input_cmd_w, std::vector<Obstacle> obstacle_data);


#endif 
