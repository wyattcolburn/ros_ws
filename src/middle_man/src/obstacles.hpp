#ifndef OBSTACLES_H
#define OBSTACLES_H
#include "raytracing.hpp"
#include <iostream>
#include <cmath>
#include <utility>
#include <utility>
class Obstacle{
    public:	
		float center_x, center_y, radius;
};
const int MAX_OBSTACLES = 8;
struct ObstacleManager{

	Obstacle obstacle_array[MAX_OBSTACLES];
	bool is_active[MAX_OBSTACLES];
	int count; //num of obstacles current in array
		
	
	//init function
    ObstacleManager() : count(0) {
		for (int i = 0; i < MAX_OBSTACLES; i++) {
			is_active[i] = false;
		}
	}
	//function for creating obstacles
	int add_obstacle(const Obstacle& obs) {
		if (count >= MAX_OBSTACLES) return -1;

		for (int i = 0; i < MAX_OBSTACLES; i++) {
			
			if(!is_active[i]){
				obstacle_array[i] = obs;
				is_active[i] = true;
				count++;
				return 1;
			}
		}
	return -1; //no space
	}
  
	// Get array of active obstacles for passing to functions
    const Obstacle* get_active_obstacles(int& out_count) { //pass by reference
        static Obstacle active_obs[MAX_OBSTACLES]; //static
        out_count = 0;
        
        for (int i = 0; i < MAX_OBSTACLES; i++) {
            if (is_active[i]) {
                active_obs[out_count++] = obstacle_array[i];
            }
        }
        
        return active_obs;
    }
};




const float OFFSET = 20.0;
const float LG1_x = 400.0;
const float LG1_y = 400.0;

//function prototypes
std::pair<Obstacle,Obstacle> create_obstacle(float current_lg_x, float current_lg_y, 
		float next_lg_x, float next_lg_y);
#endif
