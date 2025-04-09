
#ifndef LOCAL_GOALS_H
#define LOCAL_GOALS_H
#include <vector>
#include <cmath>
#include <cstdint>
#include <iostream>



constexpr float THRESHOLD = 0.75f;


class Local_Goal{
	public:
		double x_point, y_point, yaw;
};

class Local_Goal_Manager {


	public:

		float prev_distance = 0;
		void add_construct_lg(Local_Goal newLG){

			data_vector.push_back(std::move(newLG));
			return;
		}
		void add_local_goal(double x_point, double y_point, double yaw){

			Local_Goal new_local_goal;
			new_local_goal.x_point = x_point;
			new_local_goal.y_point = y_point;
			new_local_goal.yaw = yaw;

			data_vector.push_back(std::move(new_local_goal));
			return;


		}
		int get_local_goal_counter(){

			return current_local_goal_counter;
		}
		Local_Goal getCurrentGoal() {
			return data_vector[current_local_goal_counter];
		}
		
		std::vector<Local_Goal> getLocalGoalVector(){
			return data_vector;
		}

		int get_num_lg(){
			return data_vector.size();
		}
		std::vector<Local_Goal>data_vector;
		uint8_t current_local_goal_counter = 0;
		uint8_t num_lg;	   
		
		int updateLocalGoal(double odom_x, double odom_y) {
			//this is to update the local goal, aka when you have reached current
			//goal within a threshold, then get next one
			//
	     std::cout << "CHECKING IF AT NEXT LOCAL GOAL" << std::endl;
		 if (data_vector.empty() || current_local_goal_counter >= data_vector.size()) {
			std::cout << "No more local goals available" << std::endl;
			return 0;
    }
			double dx = std::abs(odom_x - data_vector[current_local_goal_counter].x_point);
		    double dy = std::abs(odom_y - data_vector[current_local_goal_counter].y_point);


		    double distance = std::sqrt(dx*dx + dy*dy);
			std::cout << "distance to next local goal   " << distance << std::endl;
			std::cout << "prev distance : " << prev_distance << std::endl;
			if (distance < THRESHOLD){
			    //advance to next local goal
				current_local_goal_counter++; 
				std::cout << "HAVE REACHED LOCAL GOAL" << std::endl;
				return 1;
			}

			else if (distance > prev_distance + .1) {
				current_local_goal_counter++;
			}
			prev_distance = distance;
			return 0;
		}
};

		
#endif
