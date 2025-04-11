#ifndef RAYTRACING_H 
#define RAYTRACING_H

#include "local_goal.hpp"
#include "obstacles.hpp"
#include <cmath>
#include <limits>
#include <iostream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
//function prototypes
//
constexpr double MIN_RANGE = .16452;
constexpr double MAX_RANGE = 12.00; //??
class Obstacle;
class ObstacleManager;

void compute_lidar_distances(
		double ray_origin_x, double ray_origin_y,
		int num_lidar_readings,
		ObstacleManager& local_manager_, 
		double* distances);

void map_compute_lidar_distances(
	double map_origin_x, double map_origin_y, double map_yaw, 
	int num_lidar_readings, ObstacleManager& local_mangager,
	double* distances, tf2_ros::Buffer& tf_buffer_); 

double normalize_angle(double angle_min);
#endif
