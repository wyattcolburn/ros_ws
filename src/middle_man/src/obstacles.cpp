#include "local_goal.hpp"
#include "raytracing.hpp"
#include "obstacles.hpp"
#include <iostream>
#include <cmath>
#include <utility>
using namespace std;
//okay lets assume we get the local goals from A star, 
//	we can assume that it is like an array of points, Ox, Oy
//	Still want to use the perpidicular approach with requires a offset value
//
//
std::pair<Obstacle, Obstacle> create_obstacle(double current_lg_x, double current_lg_y, double next_lg_x, double next_lg_y, float OFFSET, float RADIUS) {
    float mx = current_lg_x;
    float my = current_lg_y;

    // Calculate direction vector
    float dir_x = next_lg_x - current_lg_x;
    float dir_y = next_lg_y - current_lg_y;

    // Normalize
    float length = sqrt(dir_x * dir_x + dir_y * dir_y);
    if (length > 0) {
        dir_x /= length;
        dir_y /= length;
    }

    // Get perpendicular vector
    float perp_x = -dir_y;
    float perp_y = dir_x;

    // Scale
    float offset_x = perp_x * OFFSET;
    float offset_y = perp_y * OFFSET;

    // Obstacle positions
    Obstacle ob1;
    ob1.center_x = mx + offset_x;
    ob1.center_y = my + offset_y;
    ob1.radius = RADIUS;

    Obstacle ob2;
    ob2.center_x = mx - offset_x;
    ob2.center_y = my - offset_y;
    ob2.radius = RADIUS;

    return {ob1, ob2};
}

