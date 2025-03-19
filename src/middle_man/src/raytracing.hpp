#ifndef RAYTRACING_H 
#define RAYTRACING_H

#include "obstacles.hpp"
#include <cmath>
#include <limits>
#include "raylib.h"
//function prototypes
class Obstacle;
class ObstacleManager;

void draw_lidar(float odom_x, float odom_y, float* distances, int num_lidar_reading);
void compute_lidar_distances(
		float ray_origin_x, float ray_origin_y,
		int num_lidar_readings,
		ObstacleManager& local_manager, 
		float* distances);

#endif
