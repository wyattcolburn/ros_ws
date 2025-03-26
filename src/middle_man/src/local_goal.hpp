
#ifndef LOCAL_GOALS_H
#define LOCAL_GOALS_H
#include <vector>
#include <cmath>
#include <cstdint>



#define THRESHOLD .1


class Local_Goal{
	public:
		double x_point, y_point, yaw;
};

class Local_Goal_Manager {


	public:
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

		Local_Goal getCurrentGoal() {
			return data_vector[current_local_goal_counter];
		}
		
		std::vector<Local_Goal> getLocalGoalVector(){
			return data_vector;
		}
	private:

		std::vector<Local_Goal>data_vector;
		uint8_t current_local_goal_counter = 0;
	    
		void updateLocalGoal(double odom_x, double odom_y, double yaw) {
			//this is to update the local goal, aka when you have reached current
			//goal within a threshold, then get next one
		    double dx = std::abs(odom_x - data_vector[current_local_goal_counter].x_point);
		    double dy = std::abs(odom_y - data_vector[current_local_goal_counter].y_point);


		    double distance = std::sqrt(dx*dx + dy*dy);

			if (distance < THRESHOLD){
			    //advance to next local goal
				current_local_goal_counter++; 
			}

			return;
		}
};

		
#endif
