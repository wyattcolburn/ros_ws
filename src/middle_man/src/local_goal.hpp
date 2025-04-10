
#ifndef LOCAL_GOALS_H
#define LOCAL_GOALS_H
#include <vector>
#include <cmath>
#include <cstdint>
#include <iostream>



constexpr float THRESHOLD =  0.13f;


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
		std::vector<double> distance_vector;
		uint8_t current_local_goal_counter = 0;
		uint8_t num_lg;	   


		double previous_x = 0;
		double previous_y = 0;
		bool first_position = true;

		void set_distance_vector(double odom_x, double odom_y) {

			if (data_vector.empty()) {
				return;
			}

			for (int data_counter = 0; data_counter < get_num_lg(); data_counter ++) {

				
				double dx = odom_x - data_vector[data_counter].x_point;
				double dy = odom_y - data_vector[data_counter].y_point;
				double distance = std::sqrt(dx*dx + dy*dy);

				distance_vector.push_back(distance);
				std::cout << "distance at : " << data_counter << "  is " << distance << std::endl;
			}

			return;
		}

		int updateLocalGoal(double odom_x, double odom_y) {
			if (data_vector.empty() || current_local_goal_counter >= data_vector.size()) {
				std::cout << "No more local goals available" << std::endl;
				return 0;
			}
			
			// Calculate distance to current goal
			double dx = odom_x - data_vector[current_local_goal_counter].x_point;
			double dy = odom_y - data_vector[current_local_goal_counter].y_point;
			double distance = std::sqrt(dx*dx + dy*dy);
			
			std::cout << "Distance to current local goal (" << current_local_goal_counter << "): " << distance << std::endl;
			
			// Approach 1: Check if we've reached the goal directly
			if (distance < THRESHOLD) {
				std::cout << "REACHED LOCAL GOAL" << std::endl;
				current_local_goal_counter++;
				prev_distance = 0; // Reset for next goal
				return 1;
			}
			
			// Calculate heading from position changes
			double heading_x = 0;
			double heading_y = 0;
			
			if (first_position) {
				first_position = false;
			} else {
				heading_x = odom_x - previous_x;
				heading_y = odom_y - previous_y;
				
				// Only use heading if we've moved enough to get a reliable direction
				double movement = std::sqrt(heading_x*heading_x + heading_y*heading_y);
				
				if (movement > 0.01) { // Only if we've moved a meaningful amount
					// Normalize heading vector
					heading_x /= movement;
					heading_y /= movement;
					
					// Create a vector from current position to goal
					double goal_vector_x = data_vector[current_local_goal_counter].x_point - odom_x;
					double goal_vector_y = data_vector[current_local_goal_counter].y_point - odom_y;
					
					// Dot product - positive if goal is ahead, negative if we've passed it
					double dot_product = goal_vector_x * heading_x + goal_vector_y * heading_y;
					
					if (dot_product < 0) {
						// We've passed this goal, move to the next one
						std::cout << "PASSED LOCAL GOAL WITHOUT REACHING IT" << std::endl;
						current_local_goal_counter++;
						prev_distance = 0; // Reset for next goal
						return 1;
					}
				}
			}
			
			// Update previous position for next calculation
			previous_x = odom_x;
			previous_y = odom_y;
			
			// Store distance for next comparison
			prev_distance = distance;
			return 0;
		}
/*
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
*/
};

		
#endif
