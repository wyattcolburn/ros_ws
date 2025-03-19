#include "raytracing.hpp"

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
    float ray_origin_x, float ray_origin_y,
    int num_lidar_readings,
    ObstacleManager& local_manager, 
    float* distances) {


	// Get current obstacles
	int num_obstacles;
	const Obstacle* active_obstacles =local_manager.get_active_obstacles(num_obstacles);
    // Precompute the angle step for efficiency
    float angle_step = 2.0f * M_PI / num_lidar_readings;

    // For each ray direction
    for (int index = 0; index < num_lidar_readings; index++) {
        // Initialize to max distance to find closest
        distances[index] = std::numeric_limits<float>::max();

        // Calculate direction vector for this ray
        float theta = angle_step * index;
        float dx = cosf(theta);
        float dy = sinf(theta);

        // Test against each obstacle
        for (int obs = 0; obs < num_obstacles; obs++) {
            // Get obstacle data
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
            distances[index] = -1.0f;
        }
    }
}
