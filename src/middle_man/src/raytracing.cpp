#include "raytracing.hpp"
#include "local_goal.hpp"

constexpr float kPi = 3.14159265358979323846f;



void compute_lidar_distances(
    double ray_origin_x, double ray_origin_y,
    int num_lidar_readings,
    ObstacleManager& local_manager, 
    double* distances) {


	// Get current obstacles
	int num_obstacles;
	const Obstacle* active_obstacles = local_manager.get_active_obstacles(num_obstacles);
    std::cout << "**************************************** : " << num_obstacles << std::endl;

	std::cout << "num lidar reading :   " << num_lidar_readings << std::endl;


	
	std::cout << "size of active obstacles" << num_obstacles << std::endl;
	std::cout << "********************************************************" << std::endl;
		// Precompute the angle step for efficiency
    float angle_step = 2.0f * kPi / num_lidar_readings;
	std::cout << "have computed all the angles for lidar cast" << std::endl;
    // For each ray direction
    for (int index = 0; index < num_lidar_readings; index++) {
        // Initialize to max distance to find closest
        distances[index] = std::numeric_limits<float>::max();

        // Calculate direction vector for this ray
        float theta = -M_PI + angle_step * index;
		std::cout << "index is : " << theta << std::endl;
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
