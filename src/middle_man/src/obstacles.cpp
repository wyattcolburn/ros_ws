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
std::pair<Obstacle,Obstacle> create_obstacle(double current_lg_x, double current_lg_y, double next_lg_x, double next_lg_y) {
    float mx = (current_lg_x + next_lg_x) / 2;
    float my = (current_lg_y + next_lg_y) / 2;
    cout << "point values " << current_lg_x << " " << current_lg_y << " " << next_lg_x << " " << next_lg_y << endl;
    
    // Calculate direction vector of the path
    float dir_x = next_lg_x - current_lg_x;
    float dir_y = next_lg_y - current_lg_y;
    
    // Normalize the direction vector
    float length = sqrt(dir_x * dir_x + dir_y * dir_y);
    if (length > 0) {
        dir_x /= length;
        dir_y /= length;
    }
    
    // Calculate perpendicular vector (rotate 90 degrees)
    float perp_x = -dir_y;
    float perp_y = dir_x;
    
    // Scale to desired offset
    float offset_x = perp_x * OFFSET;
    float offset_y = perp_y * OFFSET;
    
    cout << "mx, my " << mx << " " << my << endl;
    cout << "direction vector: " << dir_x << ", " << dir_y << endl;
    cout << "perpendicular vector: " << perp_x << ", " << perp_y << endl;
    cout << "offset_x, offset_y: " << offset_x << ", " << offset_y << endl;
    
    // Create obstacles
    float ob1_x = mx + offset_x;
    float ob1_y = my + offset_y;
    float ob2_x = mx - offset_x;
    float ob2_y = my - offset_y;
    
    Obstacle ob1;
    ob1.center_x = ob1_x;
    ob1.center_y = ob1_y;
    ob1.radius = RADIUS;
    
    Obstacle ob2;
    ob2.center_x = ob2_x;
    ob2.center_y = ob2_y;
    ob2.radius = RADIUS;
    
    cout << "Obstacle 1: center_x = " << ob1.center_x << ", center_y = " << ob1.center_y << endl;
    cout << "Obstacle 2: center_x = " << ob2.center_x << ", center_y = " << ob2.center_y << endl;
    
    return {ob1, ob2};
}
