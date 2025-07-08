#include "raytracing.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "local_goal.hpp"
#include "obstacles.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>
constexpr float kPi = 3.14159265358979323846f;

double normalize_angle(double angle_rad) {
    while (angle_rad > M_PI)
        angle_rad -= 2.0 * M_PI;
    while (angle_rad < -M_PI)
        angle_rad += 2.0 * M_PI;
    return angle_rad;
}

void map_compute_lidar_distances(double map_origin_x, double map_origin_y, double map_yaw, int num_lidar_readings,
                                 ObstacleManager &local_manager_, double *distances, tf2_ros::Buffer &tf_buffer)

{

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer.lookupTransform("map", "odom", rclcpp::Time(0));
    } catch (tf2::TransformException &ex) {
        return;
    }

    int num_obstacles;
    const Obstacle *active_obstacles = local_manager_.get_active_obstacles(num_obstacles);

    Obstacle map_obs_array[num_obstacles];
    for (int obs_counter = 0; obs_counter < num_obstacles; obs_counter++) {
        // Create the point to transform
        geometry_msgs::msg::PointStamped obs_point_msg;
        obs_point_msg.header.frame_id = "odom";
        obs_point_msg.header.stamp = rclcpp::Time(0);
        obs_point_msg.point.x = active_obstacles[obs_counter].center_x;
        obs_point_msg.point.y = active_obstacles[obs_counter].center_y;
        obs_point_msg.point.z = 0.0;

        try {
            // Transform the point
            geometry_msgs::msg::PointStamped map_point_msg = tf_buffer.transform(obs_point_msg, "map");

            // Store the transformed point
            map_obs_array[obs_counter].center_x = map_point_msg.point.x;
            map_obs_array[obs_counter].center_y = map_point_msg.point.y;
            map_obs_array[obs_counter].radius = active_obstacles[obs_counter].radius;
        } catch (tf2::TransformException &ex) {
        }
    }

    float angle_step = 2.0f * kPi / num_lidar_readings;
    std::cout << "have computed all the angles for lidar cast" << std::endl;
    // For each ray direction
    for (int index = 0; index < num_lidar_readings; index++) {
        // Initialize to max distance to find closest
        distances[index] = std::numeric_limits<float>::max();

        // Calculate direction vector for this ray
        float theta_prev = map_yaw - M_PI / 2 + angle_step * index;

        float theta = normalize_angle(theta_prev);

        // std::cout << "num lidar : " << num_lidar_readings << std::endl;
        float dx = cosf(theta);
        float dy = sinf(theta);

        // Test against each obstacle
        for (int obs = 0; obs < num_obstacles; obs++) {
            // Get obstacle data
            //
            // std::cout << "looking at obstacle : " << obs << std::endl;
            float Cx = map_obs_array[obs].center_x;
            float Cy = map_obs_array[obs].center_y;
            float radius = map_obs_array[obs].radius;

            // Compute ray-circle intersection
            float mx = map_origin_x - Cx;
            float my = map_origin_y - Cy;
            float a = dx * dx + dy * dy;
            float b = 2.0f * (mx * dx + my * dy);
            float c = mx * mx + my * my - radius * radius;

            // Compute discriminant
            float discriminant = b * b - 4.0f * a * c;
            // std::cout << "discriminant" << discriminant << std::endl;
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

void compute_lidar_distances(double ray_origin_x, double ray_origin_y, int num_lidar_readings,
                             ObstacleManager &local_manager, double *distances) {

    // Get current obstacles
    int num_obstacles;
    const Obstacle *active_obstacles = local_manager.get_active_obstacles(num_obstacles);
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
        // std::cout << "num lidar : " << num_lidar_readings << std::endl;
        // std::cout << "theta : " << theta << std::endl;
        float dx = cosf(theta);
        float dy = sinf(theta);

        // Test against each obstacle
        for (int obs = 0; obs < num_obstacles; obs++) {
            // Get obstacle data
            //
            // std::cout << "looking at obstacle : " << obs << std::endl;
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
            // std::cout << "discriminant" << discriminant << std::endl;
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
