#ifndef RAYTRACING_H 
#define RAYTRACING_H

#include "local_goal.hpp"
#include "obstacles.hpp"
#include <cmath>
#include <limits>
#include "raylib.h"
#include <iostream>
//function prototypes
//
constexpr MIN_RANGE = .16452;
constexpr MAX_RANGE = 12.00; //??
class Obstacle;
class ObstacleManager;

void test(ObstacleManager& local_manager);
void draw_lidar(float odom_x, float odom_y, float* distances, int num_lidar_reading);
void compute_lidar_distances(
		double ray_origin_x, double ray_origin_y,
		int num_lidar_readings,
		ObstacleManager& local_manager, 
		double* distances);

#endif
