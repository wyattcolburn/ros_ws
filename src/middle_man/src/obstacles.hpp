#ifndef OBSTACLES_H
#define OBSTACLES_H
#include "local_goal.hpp"
#include <iostream>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm>


const float RADIUS=.5;
const int NUM_VALID_OBSTACLES=10;
class Obstacle{
    public:	
		double center_x, center_y, radius;
};
//function prototypes
std::pair<Obstacle,Obstacle> create_obstacle(double current_lg_x, double current_lg_y, double next_lg_x, double next_lg_y);
const int MAX_OBSTACLES = 100;

const float OFFSET = 2.0;
struct ObstacleManager{

	Obstacle obstacle_array[MAX_OBSTACLES];
	bool is_active[MAX_OBSTACLES];
	int obstacle_count; //num of obstacles current in array
		
	
	//init function
    ObstacleManager() : obstacle_count(0) {
		for (int i = 0; i < MAX_OBSTACLES; i++) {
			is_active[i] = false;
		}
	}
	//function for creating obstacles
	int add_obstacle(const Obstacle& obs) {
		if (obstacle_count >= MAX_OBSTACLES) return -1;

		for (int i = 0; i < MAX_OBSTACLES; i++) {
			
			if(!is_active[i]){
				obstacle_array[i] = obs;
				is_active[i] = true;
				obstacle_count++;
				return 1;
			}
		}
	return -1; //no space
	}
 
	void local_goals_to_obs(Local_Goal_Manager& local_manager_){
		//does not work
		std::vector<Local_Goal> local_goal_vec;
		local_goal_vec = local_manager_.getLocalGoalVector();

		for (int lg_counter = 0; lg_counter < static_cast<int>(local_goal_vec.size() - 1); lg_counter++){

			if (lg_counter + 1 > local_goal_vec.size()) 
			{
				return;
			}
			Local_Goal current_lg = local_goal_vec[lg_counter];
			Local_Goal next_lg = local_goal_vec[lg_counter+1];

			std::pair<Obstacle, Obstacle> obs_pair = create_obstacle(current_lg.x_point, current_lg.y_point, next_lg.x_point, next_lg.y_point);
			add_obstacle(obs_pair.first);
			add_obstacle(obs_pair.second);

			std::cout << "Obstacle at " << obs_pair.first.center_x<< "  " << obs_pair.first.center_y<< std::endl;
		}
		return;
	}
	void update_obstacles(Local_Goal_Manager& local_manager_){

		//current goal is local_goal_counter, 
		// sliding window is, local_goal_counter ------------ +10 

		int local_goal_counter = local_manager_.get_local_goal_counter(); //returns which local goal we are on, if we are on local goal 0, we are on obstacle 0,1
	    std::cout << "local goal counter" << local_goal_counter << std::endl;
		std::cout << "sliding window bounds are: " << local_goal_counter << "   to   " << std::min(NUM_VALID_OBSTACLES -1 + local_goal_counter, obstacle_count) << std::endl;

		for (int counter = 0; counter < obstacle_count; counter++){
			//std::cout << "counter for loop" << counter << std::endl;
			if ((counter >= local_goal_counter) && (counter <= std::min(local_goal_counter + NUM_VALID_OBSTACLES - 1, obstacle_count))) {
				is_active[counter] = true;
			}

			else {
				is_active[counter] = false;
			}
		}
		std::cout << "have arrived to the end of update obstacle function" << std::endl;
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




#endif
