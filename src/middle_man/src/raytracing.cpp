#include "raytracing.hpp"
#include "local_goal.hpp"

void draw_lidar(float odom_x, float odom_y,float* distances, int num_lidar_readings) {

        float angle_step = 2.0f * M_PI / num_lidar_readings;
        for (int i = 0; i < num_lidar_readings; i += 1) {
            float theta = angle_step * i;
            float dx = cosf(theta);
            float dy = sinf(theta);
            
            if (distances[i] > 0) {
                // Draw hit rays in green
                DrawLine(
                    odom_x, odom_y,
                    odom_x + dx * distances[i],
                    odom_y + dy * distances[i],
                    GREEN
                );
            }
        }
}


void compute_lidar_distances(
    double ray_origin_x, double ray_origin_y,
    int num_lidar_readings,
    ObstacleManager& local_manager, 
    double* distances) {


	// Get current obstacles
	int num_obstacles;
	const Obstacle* active_obstacles = local_manager.get_active_obstacles(num_obstacles);


	//std::cout << "origin point : " << ray_origin_x << "  " << ray_origin_y << std::endl;

	
	//std::cout << "size of active obstacles" << num_obstacles << std::endl;
	//std::cout << "********************************************************" << std::endl;
	//std::cout << "first obstacle :  " << active_obstacles[0].center_x << "   " << active_obstacles[0].center_y << std::endl;
	// Precompute the angle step for efficiency
    float angle_step = 2.0f * M_PI / num_lidar_readings;
	std::cout << "have computed all the angles for lidar cast" << std::endl;
    // For each ray direction
    for (int index = 0; index < num_lidar_readings; index++) {
        // Initialize to max distance to find closest
        distances[index] = std::numeric_limits<float>::max();

        // Calculate direction vector for this ray
        float theta = angle_step * index;
		//std::cout << "num lidar : " << num_lidar_readings << std::endl;
		//std::cout << "theta : " << theta << std::endl;
        float dx = cosf(theta);
        float dy = sinf(theta);

        // Test against each obstacle
        for (int obs = 0; obs < num_obstacles; obs++) {
            // Get obstacle data
			//
			//std::cout << "looking at obstacle : " << obs << std::endl;
            float Cx = active_obstacles[obs].center_x;
            float Cy = active_obstacles[obs].center_y;
            float radius = active_obstacles[obs].radius;

            // Compute ray-circle intersection
            float mx = ray_origin_x - Cx;
            float my = ray_origin_y - Cy;
            float a = dx * dx + dy * dy;
            float b = 2.0f * (mx * dx + my * dy);
            float c = mx * mx + my * my - radius * radius;

            // Compute discriminant
            float discriminant = b * b - 4.0f * a * c;
			//std::cout << "discriminant" << discriminant << std::endl;
            if (discriminant >= 0.0f) {
				std::cout << "We have intersection" << std::endl;
                // Has intersection(s)
                float sqrt_discriminant = sqrtf(discriminant);
                float t1 = (-b - sqrt_discriminant) / (2.0f * a);
                float t2 = (-b + sqrt_discriminant) / (2.0f * a);

                // Find closest valid intersection
                if (t1 > 0.0f && t1 < distances[index]) {
                    distances[index] = t1;
                }

                if (t2 > 0.0f && t2 < distances[index]) {
                    distances[index] = t2;
                }
            }
        }

        // If no intersection was found, mark with -1
        if (distances[index] == std::numeric_limits<float>::max()) {
            distances[index] = MAX_RANGE;
        }
    }
}
