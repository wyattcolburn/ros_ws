#ifndef OBSTACLES_H
#define OBSTACLES_H
#include "raytracing.hpp"
#include "local_goal.hpp"
#include <iostream>
#include <cmath>
#include <utility>
#include <vector>

#define RADIUS = 10;
class Obstacle{
    public:	
		float center_x, center_y, radius;
};
//function prototypes
std::pair<Obstacle,Obstacle> create_obstacle(double current_lg_x, double current_lg_y, double next_lg_x, double next_lg_y);
const int MAX_OBSTACLES = 100;
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
 
	void local_goals_to_obs(Local_Goal_Manager& local_manager_){
		//does not work
		std::vector<Local_Goal> local_goal_vec;
		local_goal_vec = local_manager_.getLocalGoalVector();

		for (int lg_counter = 0; lg_counter < local_goal_vec.size(); lg_counter++){

			if (lg_counter + 1 > local_goal_vec.size()) 
			{
				return;
			}
			Local_Goal current_lg = local_goal_vec[lg_counter];
			Local_Goal next_lg = local_goal_vec[lg_counter+1];

			std::pair<Obstacle, Obstacle> obs_pair = create_obstacle(current_lg.x_point, current_lg.y_point, next_lg.x_point, next_lg.y_point);
			add_obstacle(obs_pair.first);
			add_obstacle(obs_pair.second);
		}
		return;
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

#endif
