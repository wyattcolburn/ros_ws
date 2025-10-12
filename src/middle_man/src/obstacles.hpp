#ifndef OBSTACLES_H
#define OBSTACLES_H
#include "local_goal.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <utility>
#include <vector>
// parameters from config.yaml

const int MAX_OBSTACLES = 100;

class Obstacle {
  public:
    double center_x, center_y, radius;
};
// function prototypes
std::pair<Obstacle, Obstacle> create_obstacle(double current_lg_x, double current_lg_y, double next_lg_x,
                                              double next_lg_y, float OFFSET, float RADIUS);
struct ObstacleManager {

    Obstacle obstacle_array[MAX_OBSTACLES];
    bool is_active[MAX_OBSTACLES];
    float RADIUS;
    int NUM_VALID_OBSTACLES;
    float OFFSET;
    int obstacle_count; // num of obstacles current in array
    ObstacleManager() : RADIUS(0.4f), NUM_VALID_OBSTACLES(20), OFFSET(1.0f), obstacle_count(0) {
        for (int i = 0; i < MAX_OBSTACLES; i++) {
            is_active[i] = false;
        }
    }

    void set_params(float radius, int num_valid, float offset) {
        // init values from config.yaml
        RADIUS = radius;
        NUM_VALID_OBSTACLES = num_valid;
        OFFSET = offset;
    }
    // function for creating obstacles
    int add_obstacle(const Obstacle &obs) {
        if (obstacle_count >= MAX_OBSTACLES)
            return -1;

        for (int i = 0; i < MAX_OBSTACLES; i++) {

            if (!is_active[i]) {
                obstacle_array[i] = obs;
                is_active[i] = true;
                obstacle_count++;
                return 1;
            }
        }
        return -1; // no space
    }

    void local_goals_to_obs(Local_Goal_Manager &local_manager_) {
        // does not work
        std::vector<Local_Goal> local_goal_vec;
        local_goal_vec = local_manager_.getLocalGoalVector();

        for (int lg_counter = 0; lg_counter < static_cast<int>(local_goal_vec.size() - 1); lg_counter++) {

            if (lg_counter + 1 > local_goal_vec.size()) {
                return;
            }
            Local_Goal current_lg = local_goal_vec[lg_counter];
            Local_Goal next_lg = local_goal_vec[lg_counter + 1];

            std::pair<Obstacle, Obstacle> obs_pair = create_obstacle(current_lg.x_point, current_lg.y_point,
                                                                     next_lg.x_point, next_lg.y_point, OFFSET, RADIUS);
            add_obstacle(obs_pair.first);
            add_obstacle(obs_pair.second);

            // std::cout << "Obstacle at " << obs_pair.first.center_x << "  " << obs_pair.first.center_y << std::endl;
        }
        return;
    }
    void update_obstacles(Local_Goal_Manager &local_manager_) {
        if (obstacle_count <= 0)
            return;

        // Current segment index (which local goal we're on)
        int g = local_manager_.get_local_goal_counter();

        // Map segment index -> obstacle index range (2 obstacles per segment)
        int start = 2 * g; // first obstacle for this segment
        int end = std::min(start + NUM_VALID_OBSTACLES - 1, obstacle_count - 1);

        // Log (obstacle indices, not goal indices)
        // std::cout << "goal/segment index: " << g << "  -> active obstacle window: [" << start << ", " << end << "] of
        // "
        // << obstacle_count << std::endl;

        for (int i = 0; i < obstacle_count; ++i) {
            is_active[i] = (i >= start && i <= end);
        }
    }

    void update_obstacles_sliding(Local_Goal_Manager &local_manager_) {

        if (obstacle_count <= 0)
            return;

        int g = local_manager_.get_local_goal_counter();
        int start_offset = NUM_VALID_OBSTACLES / 4;
        int start = 2 * g - start_offset;
        int end = std::min(start + NUM_VALID_OBSTACLES - 1, obstacle_count - 1);

        for (int i = 0; i < obstacle_count; ++i) {
            is_active[i] = (i >= start && i <= end);
        }
        // int g = local_manager_.get_local_goal_counter();
        // int start_offset = NUM_VALID_OBSTACLES / 4;
        // int start = std::max(0, 2 * g - start_offset);
        // start = std::min(start, std::max(0, obstacle_count - 1));
        // int end = std::min(start + NUM_VALID_OBSTACLES - 1, obstacle_count - 1);
        //
        // for (int i = 0; i < obstacle_count; ++i) {
        //     is_active[i] = (i >= start && i <= end);
        // }

        // Get array of active obstacles for passing to functions
        const Obstacle *get_active_obstacles(int &out_count) {
            static Obstacle active_obs[MAX_OBSTACLES];
            out_count = 0;

            for (int i = 0; i < MAX_OBSTACLES; i++) {
                if (is_active[i]) {
                    active_obs[out_count++] = obstacle_array[i];
                }
            }

            return active_obs;
        }

        void clean_data() {
            // new path so need to make new obstacles
            for (int i = 0; i < MAX_OBSTACLES; i++) {
                is_active[i] = false;
            }
            obstacle_count = 0;
            memset(obstacle_array, 0, sizeof(obstacle_array));
            return;
        }
    };

#endif
