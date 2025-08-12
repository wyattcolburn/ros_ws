#include "local_goal.hpp"
#include "nav_msgs/msg/path.hpp"
#include "obstacles.hpp"

#ifdef PI
#undef PI
#endif
#include "raytracing.hpp"
#include "rclcpp/rclcpp.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Want to add halucinating obstacles
class obsValid : public rclcpp::Node {
  public:
    obsValid() : Node("obs_valid") {

        data_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/packetOut", 10, std::bind(&obsValid::data_callback, this, std::placeholders::_1));

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan_barn", 10, std::bind(&obsValid::path_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&obsValid::scan_callback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 10);
        // Add this to your constructor
        local_goal_marker_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("/local_goal_markers", 10);

        hall_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/HallScan", 10);

        packetOut_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("neuralNetInput", 10);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

  private:
    Local_Goal_Manager local_goal_manager_;
    ObstacleManager obstacle_manager_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr data_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr hall_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr packetOut_publisher_;
    // Add this to your private members
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_goal_marker_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    static constexpr size_t ODOM_FIELD_COUNT = 2; // odom current v, w
    static constexpr size_t LOCAL_GOAL_COUNT = 3; // local goal x, y, yaw
    static constexpr size_t LIDAR_COUNT = 1080;
    static constexpr size_t CURRENT_ODOM = 3; // x, y, yaw:wq

    bool path_flag = false;
    bool goal_flag = false;
    double
        packetOut[ODOM_FIELD_COUNT + LOCAL_GOAL_COUNT + LIDAR_COUNT + CURRENT_ODOM]; // should be 1085 + obstacle data

    double packetIn[ODOM_FIELD_COUNT + LIDAR_COUNT]; // pos_x, pos_y, cmd_v, cmd_w, yaw
    double odom_x, odom_y, local_goal_x, local_goal_y, yaw, current_cmd_v, current_cmd_w;

    double real_lidar_ranges[LIDAR_COUNT];
    double hall_lidar_ranges[LIDAR_COUNT];

    double map_x = 0;
    double map_y = 0;
    double map_yaw = 0;

    double map_lg_x = 0;
    double map_lg_y = 0;
    double map_lg_yaw = 0;
    Local_Goal GOAL;

    static constexpr double GOAL_THRESHOLD = .5;

    void data_callback(const std_msgs::msg::Float64MultiArray &packetin) {

        if (path_flag == false) {
            // std::cout << "Have yet to receive a path yet" << std::endl;
            return;
        }
        processOdomLidar(packetin);
        // std::cout << "**************** yaw value : " << yaw << std::endl;

        geometry_msgs::msg::PoseStamped odom_pose;
        odom_pose.header.frame_id = "odom";
        odom_pose.header.stamp = rclcpp::Time(0); // latest available transform
        odom_pose.pose.position.x = odom_x;
        odom_pose.pose.position.y = odom_y;
        odom_pose.pose.position.z = 0.0;

        // If you have yaw:
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw); // only yaw
        odom_pose.pose.orientation = tf2::toMsg(q);

        geometry_msgs::msg::PoseStamped map_pose;
        try {
            map_pose = tf_buffer_->transform(odom_pose, "map");

            // Transformed output
            map_x = map_pose.pose.position.x;
            map_y = map_pose.pose.position.y;

            // Extract yaw if needed
            tf2::Quaternion q_map;
            tf2::fromMsg(map_pose.pose.orientation, q_map);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q_map).getRPY(roll, pitch, yaw);
            map_yaw = yaw;

        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(rclcpp::get_logger("raytracing"), "Failed to transform odom pose: %s", ex.what());
            return;
        }

        // RCLCPP_INFO(this->get_logger(), "odom (%.2f, %.2f) -> map (%.2f, %.2f) yaw %.2f), map_yaw %.2f", odom_x,
        // odom_y,
        //             map_x, map_y, yaw, map_yaw);

        local_goal_manager_.updateLocalGoal(odom_x, odom_y); // local goals have already been converted to odom
        obstacle_manager_.update_obstacles(local_goal_manager_);

        auto local_goal_markers = make_local_goal_markers();
        local_goal_marker_pub_->publish(local_goal_markers);
        int num_obs;
        auto obs_list = obstacle_manager_.get_active_obstacles(num_obs);
        auto marker_array = make_markers(obs_list, static_cast<size_t>(num_obs));
        marker_pub_->publish(marker_array);

        map_compute_lidar_distances(map_x, map_y, map_yaw, LIDAR_COUNT, obstacle_manager_, hall_lidar_ranges,
                                    *tf_buffer_);
        processPacketOut();

        int packetOut_size = sizeof(packetOut) / sizeof(packetOut[0]);
        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(packetOut_size);
        // RCLCPP_INFO(this->get_logger(), "SIZE OF packetOUt %d", packetOut_size);

        // RCLCPP_INFO(this->get_logger(), "Size of neural net output %zu", msg.data.size());
        // Test with a simpler message
        if (!packetOut_publisher_) {
            RCLCPP_ERROR(this->get_logger(), "Publisher is null!");
        }
        //
        for (size_t i = 0; i < packetOut_size; ++i) {
            msg.data[i] = static_cast<double>(packetOut[i]);
        }
        RCLCPP_INFO(this->get_logger(), "LOCAL GOAL DATA TO NN %.3f %.3f", msg.data[2], msg.data[3]);
        // RCLCPP_INFO(this->get_logger(), "LOCAL GOAL VEC SIZE, and current position %zu, %d",
        // local_goal_manager_.data_vector.size(), local_goal_manager_.current_local_goal_counter);
        // RCLCPP_INFO(this->get_logger(), "HAVE SUCCESSFULLY COPIED THE MESSAGE");
        packetOut_publisher_->publish(msg);
        // reached_goal(odom_x, odom_y);
        return;
    }

    void processPacketOut() {
        packetOut[0] = current_cmd_v;
        packetOut[1] = current_cmd_w;

        Local_Goal currentLG = local_goal_manager_.data_vector[local_goal_manager_.current_local_goal_counter];
        packetOut[2] = currentLG.x_point;
        packetOut[3] = currentLG.y_point;
        packetOut[4] = currentLG.yaw;
        // packetOut[2] = map_lg_x;
        // packetOut[3] = map_lg_y;
        // packetOut[4] = currentLG.yaw;

        for (int lidar_counter = 0; lidar_counter < LIDAR_COUNT; lidar_counter++) {
            packetOut[5 + lidar_counter] = hall_lidar_ranges[lidar_counter];
        }

        packetOut[ODOM_FIELD_COUNT + LOCAL_GOAL_COUNT + LIDAR_COUNT] = odom_x;
        packetOut[ODOM_FIELD_COUNT + LOCAL_GOAL_COUNT + LIDAR_COUNT + 1] = odom_y;
        packetOut[ODOM_FIELD_COUNT + LOCAL_GOAL_COUNT + LIDAR_COUNT + 2] = yaw;

        // just sending lidar data no obstacles
        //  filled with min lidar data
        //  int num_obstacles;
        //  const Obstacle *current_obstacles = obstacle_manager_.get_active_obstacles(num_obstacles);
        //  for (int local_obstacle_counter = 0; local_obstacle_counter < num_obstacles; local_obstacle_counter++) {
        //      int index = ODOM_FIELD_COUNT + LOCAL_GOAL_COUNT + LIDAR_COUNT + 2 + local_obstacle_counter * 2;
        //      packetOut[index] = current_obstacles[local_obstacle_counter].center_x;
        //      packetOut[index + 1] = current_obstacles[local_obstacle_counter].center_y;
        //  }
    }

    void path_callback(const nav_msgs::msg::Path::ConstSharedPtr pathMsg) {
        // With barn should only get called once
        RCLCPP_INFO(this->get_logger(), "Received %zu poses in frame %s", pathMsg->poses.size(),
                    pathMsg->header.frame_id.c_str());

        // Get transform from map to odom
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform("odom", "map", tf2::TimePointZero);
            // RCLCPP_INFO(this->get_logger(), "Successfully got transform from map to odom");
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }

        // RCLCPP_INFO(this->get_logger(), "PRE TRANSFORMATION LOCAL GOAL %.3f, %.3f",
        // pathMsg->poses[0].pose.position.x,
        //             pathMsg->poses[0].pose.position.y);
        // map_lg_x = pathMsg->poses[0].pose.position.x;
        // map_lg_y = pathMsg->poses[0].pose.position.y;

        tf2::Quaternion q(pathMsg->poses[0].pose.orientation.x, pathMsg->poses[0].pose.orientation.y,
                          pathMsg->poses[0].pose.orientation.z, pathMsg->poses[0].pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        map_lg_yaw = yaw;
        local_goal_manager_.clean_data();
        // translate local goals into odom points
        for (size_t i = 0; i < pathMsg->poses.size(); i++) {
            // Transform position from map to odom frame
            geometry_msgs::msg::PoseStamped pose_in_map;
            geometry_msgs::msg::PoseStamped pose_in_odom;

            pose_in_map.header.frame_id = "map";
            pose_in_map.pose = pathMsg->poses[i].pose;
            try {
                tf2::doTransform(pose_in_map, pose_in_odom, transform);

                // Extract yaw from quaternion
                tf2::Quaternion q(pose_in_odom.pose.orientation.x, pose_in_odom.pose.orientation.y,
                                  pose_in_odom.pose.orientation.z, pose_in_odom.pose.orientation.w);
                double roll, pitch, yaw;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                // Add transformed goal to manager
                local_goal_manager_.add_local_goal(pose_in_odom.pose.position.x, pose_in_odom.pose.position.y, yaw);
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "Transform error on pose %zu: %s", i, ex.what());
                continue; // Skip this goal and try the next
            }
        }
        // This code works when you are constantly polling path topic because nav stack is updating,
        // with BARN, only one plan is used so no need to constiently update
        //
        if (local_goal_manager_.get_num_lg() > 0) {
            obstacle_manager_.local_goals_to_obs(local_goal_manager_);
            path_flag = true;

            GOAL.x_point = local_goal_manager_.data_vector.back().x_point;
            GOAL.y_point = local_goal_manager_.data_vector.back().y_point;
            GOAL.yaw = local_goal_manager_.data_vector.back().yaw;

            // std::cout << "GOAL after setting: (" << GOAL.x_point << ", " << GOAL.y_point << ")" << std::endl;
        } else {
            RCLCPP_WARN(this->get_logger(), "No local goals were added after transformation");
        }
        return;
        // //  RCLCPP_INFO(this->get_logger(), "Our current Odom position is %f %f", odom_x, odom_y);
        // //  add to local goal manager to create obstacles
        // if (local_goal_manager_.get_num_lg() > 0) {
        //     // RCLCPP_INFO(this->get_logger(), "FIRST GOAL VALUES ARE %f and %f",
        //     //             local_goal_manager_.data_vector[0].x_point,
        //     //             local_goal_manager_.data_vector[0].y_point);
        //     // RCLCPP_INFO(this->get_logger(), "SIZE OF LOCAL_GOAL_VECTOR %d",
        //     // local_goal_manager_.get_num_lg());
        //     obstacle_manager_.clean_data(); // reset the obstacles array
        //     obstacle_manager_.local_goals_to_obs(local_goal_manager_);
        //     local_goal_manager_.set_distance_vector(odom_x, odom_y);
        //     path_flag = true;
        //
        //     GOAL.x_point = local_goal_manager_.data_vector.back().x_point;
        //     GOAL.y_point = local_goal_manager_.data_vector.back().y_point;
        //     GOAL.yaw = local_goal_manager_.data_vector.back().yaw;
        //
        //     std::cout << "GOAL after setting: (" << GOAL.x_point << ", " << GOAL.y_point << ")" << std::endl;
        // } else {
        //     RCLCPP_WARN(this->get_logger(), "No local goals were added after transformation");
        // }
        // return;
    }
    void processOdomLidar(const std_msgs::msg::Float64MultiArray &packetIn) {

        // This is possible because of the statically defined packets
        odom_x = packetIn.data[0];
        odom_y = packetIn.data[1];
        current_cmd_v = packetIn.data[2];
        current_cmd_w = packetIn.data[3];
        yaw = packetIn.data[4];
        return;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg) {

        for (size_t i = 0; i < LIDAR_COUNT && i < scanMsg->ranges.size(); i++) {
            real_lidar_ranges[i] = static_cast<double>(scanMsg->ranges[i]);
        }

        sensor_msgs::msg::LaserScan hall_msg;

        hall_msg.header = scanMsg->header;
        hall_msg.angle_min = scanMsg->angle_min;
        hall_msg.angle_max = scanMsg->angle_max;
        hall_msg.angle_increment = scanMsg->angle_increment;
        hall_msg.time_increment = scanMsg->time_increment;
        hall_msg.scan_time = scanMsg->scan_time;
        hall_msg.range_min = scanMsg->range_min;
        hall_msg.range_max = scanMsg->range_max;
        std::vector<float> hall_lidar_publish;
        min_hall_lidar(real_lidar_ranges, hall_lidar_ranges, 1080);
        for (int i = 0; i < LIDAR_COUNT; i++) {
            hall_lidar_publish.push_back(static_cast<float>(hall_lidar_ranges[i]));
            // RCLCPP_INFO(this->get_logger(), "Lidar value : %f",
            // min_lidar_ranges[i]);
        }
        hall_msg.ranges = hall_lidar_publish;

        hall_pub_->publish(hall_msg);
        return;
    }

    visualization_msgs::msg::MarkerArray make_markers(const Obstacle *obs_list, size_t count) {
        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < count; ++i) {
            const Obstacle &ob = obs_list[i];
            geometry_msgs::msg::PointStamped odom_point;
            odom_point.header.frame_id = "odom";
            odom_point.header.stamp = rclcpp::Time(0); // This is a valid ROS 2 zero timestamp

            odom_point.point.x = ob.center_x;
            odom_point.point.y = ob.center_y;

            geometry_msgs::msg::PointStamped map_point;
            try {
                map_point = tf_buffer_->transform(odom_point, "map");
            } catch (const tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "Transform failed for obstacle %zu: %s", i, ex.what());
                continue;
            }

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map"; // Now using the map frame
            marker.header.stamp = this->now();
            marker.ns = "hallucinated_obstacles";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = map_point.point.x;
            marker.pose.position.y = map_point.point.y;
            marker.pose.position.z = 0.1;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = ob.radius * 5;
            marker.scale.y = ob.radius * .5;
            marker.scale.z = 0.2;

            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;

            marker.lifetime = rclcpp::Duration::from_seconds(0.5);
            marker_array.markers.push_back(marker);
        }
        return marker_array;
    }
    void min_hall_lidar(const double *real_lidar, double *hall_lidar, size_t array_size) {
        // Takes the real_lidar from /scan and compares to hall_lidar, want to have lowest possible values incase
        // close to obstacle
        for (size_t lidar_counter = 0; lidar_counter < array_size; lidar_counter++) {
            if ((hall_lidar[lidar_counter] > 0) && (real_lidar[lidar_counter] > 0)) {
                if (real_lidar[lidar_counter] < hall_lidar[lidar_counter]) {
                    hall_lidar[lidar_counter] = real_lidar[lidar_counter];
                }
            }
        }
    }
    int reached_goal(const double odom_x, const double odom_y) {
        double dx = odom_x - GOAL.x_point;
        double dy = odom_y - GOAL.y_point;
        double distance = std::sqrt(dx * dx + dy * dy);

        // std::cout << "Robot odom pos: (" << odom_x << ", " << odom_y << ")" << std::endl;
        // std::cout << "GOAL values: (" << GOAL.x_point << ", " << GOAL.y_point << ")" << std::endl;
        // std::cout << "dx: " << dx << ", dy: " << dy << std::endl;

        if (distance < GOAL_THRESHOLD) {
            // std::cout << "REACH GOAL POSE" << std::endl;
            goal_flag = true;
            return 1;
        } else {
            // std::cout << "dist to goal is : " << distance << std::endl;
            return 0;
        }
    }
    visualization_msgs::msg::MarkerArray make_local_goal_markers() {
        visualization_msgs::msg::MarkerArray marker_array;

        // Clear any existing markers first
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = "map"; // Changed to map frame
        delete_marker.header.stamp = this->now();
        delete_marker.ns = "local_goals";
        delete_marker.id = 0;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        // Create markers for each local goal
        for (size_t i = 0; i < local_goal_manager_.data_vector.size(); ++i) {
            const Local_Goal &lg = local_goal_manager_.data_vector[i];

            // Transform local goal from odom to map frame
            geometry_msgs::msg::PoseStamped odom_pose;
            odom_pose.header.frame_id = "odom";
            odom_pose.header.stamp = rclcpp::Time(0);
            odom_pose.pose.position.x = lg.x_point;
            odom_pose.pose.position.y = lg.y_point;
            odom_pose.pose.position.z = 0.0;

            tf2::Quaternion q_odom;
            q_odom.setRPY(0, 0, lg.yaw);
            odom_pose.pose.orientation = tf2::toMsg(q_odom);

            geometry_msgs::msg::PoseStamped map_pose;
            try {
                map_pose = tf_buffer_->transform(odom_pose, "map");
            } catch (const tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "Transform failed for local goal %zu: %s", i, ex.what());
                continue; // Skip this goal if transform fails
            }

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map"; // Now using map frame
            marker.header.stamp = this->now();
            marker.ns = "local_goals";
            marker.id = i + 1; // Start from 1 since 0 is used for delete
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Position (now in map frame)
            marker.pose.position.x = map_pose.pose.position.x;
            marker.pose.position.y = map_pose.pose.position.y;
            marker.pose.position.z = 0.0;

            // Orientation (now in map frame)
            marker.pose.orientation = map_pose.pose.orientation;

            // Scale (arrow size)
            marker.scale.x = 0.3;  // Length
            marker.scale.y = 0.05; // Width
            marker.scale.z = 0.05; // Height

            // Color - different colors for different goals
            if (i == local_goal_manager_.current_local_goal_counter) {
                // Current goal - bright green
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 0.8;
            } else if (i == local_goal_manager_.data_vector.size() - 1) {
                // Final goal - blue
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marker.color.a = 0.8;
            } else {
                // Intermediate goals - yellow
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 0.4;
            }

            marker.lifetime = rclcpp::Duration::from_seconds(0.0); // Persistent until replaced
            marker_array.markers.push_back(marker);
        }

        return marker_array;
    }
};
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<obsValid>());
    rclcpp::shutdown();
    return 0;
}
