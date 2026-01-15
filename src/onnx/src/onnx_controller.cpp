#include "onnx/onnx_controller.hpp"
#include <algorithm>
#include <fstream>
#include <memory>
#include <string>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// In your ONNXController class:
#include "std_msgs/msg/float64_multi_array.hpp"

#include "mpc.hpp"
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(onnx_controller::ONNXController, nav2_core::Controller)

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

std::vector<float> readCSVToFloats(const std::string &filename, char delimiter = ',', bool skipHeader = false) {
    std::vector<float> data;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return data;
    }

    std::string line;

    // Skip the header row if skipHeader is true
    if (skipHeader && std::getline(file, line)) {
        // Optional: print header for debugging
        // std::cout << "Skipped header: " << line << std::endl;
    }

    // Read file contents into a string first
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string fileContents = buffer.str();

    // Make sure the last character is a newline for consistent parsing
    if (!fileContents.empty() && fileContents.back() != '\n') {
        fileContents.push_back('\n');
    }

    // Parse each line
    std::istringstream iss(fileContents);
    int lineCount = 0;

    while (std::getline(iss, line)) {
        lineCount++;
        // Skip empty lines
        if (line.empty())
            continue;
        // For non-CSV files with one value per line (like your scaler files)
        if (line.find(delimiter) == std::string::npos) {
            try {
                data.push_back(std::stof(line));
            } catch (const std::exception &e) {
                std::cerr << "Warning: Could not convert '" << line << "' to float. Using 0.0 instead." << std::endl;
                data.push_back(0.0f);
            }
        }
        // For CSV files with delimiter-separated values
        else {
            std::stringstream ss(line);
            std::string cell;

            while (std::getline(ss, cell, delimiter)) {
                if (!cell.empty()) {
                    try {
                        data.push_back(std::stof(cell));
                    } catch (const std::exception &e) {
                        std::cerr << "Warning: Could not convert '" << cell << "' to float. Using 0.0 instead."
                                  << std::endl;
                        data.push_back(0.0f);
                    }
                }
            }
        }
    }

    std::cout << "Read " << lineCount << " lines and parsed " << data.size() << " values from " << filename
              << std::endl;

    file.close();
    return data;
}
namespace onnx_controller {
void ONNXController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                               const std::shared_ptr<tf2_ros::Buffer> tf,
                               const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    node_ = parent;

    auto node = node_.lock();

    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    RCLCPP_INFO(logger_, "Configuring ONNX Controller...");

    // Declare parameters
    declare_parameter_if_not_declared(node, plugin_name_ + ".model_path", rclcpp::ParameterValue("/pathOfModel.onnx"));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));

    node->get_parameter(plugin_name_ + ".model_path", model_path_);
    node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
    node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);

    feature_mins_ = readCSVToFloats("/home/mobrob/ros_ws/model_path/current_scaler_mins.txt");
    feature_maxs_ = readCSVToFloats("/home/mobrob/ros_ws/model_path/current_scaler_maxs.txt");

    // Load ONNX Model
    Ort::Env env;
    Ort::SessionOptions session_options;

    session_ = std::make_unique<Ort::Session>(env, model_path_.c_str(), session_options);

    // Get input and output names
    Ort::AllocatorWithDefaultOptions ort_alloc;
    Ort::AllocatedStringPtr inputName = session_->GetInputNameAllocated(0, ort_alloc);
    Ort::AllocatedStringPtr outputName = session_->GetOutputNameAllocated(0, ort_alloc);
    input_name_ = {inputName.get()};
    output_name_ = {outputName.get()};
    inputName.release();
    outputName.release();
    trajectory_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("mpc_trajectories", 1);
    global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
    onnx_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/neuralNetInput", rclcpp::QoS(10), std::bind(&ONNXController::onnxInputCallback, this, std::placeholders::_1)

    );
}
void ONNXController::cleanup() {
    RCLCPP_INFO(logger_, "Cleaning up ONNX Controller...");
    session_.reset();
    global_pub_.reset();
}

void ONNXController::activate() {
    RCLCPP_INFO(logger_, "Activating ONNX Controller...");
    global_pub_->on_activate();
}

void ONNXController::deactivate() {
    RCLCPP_INFO(logger_, "Deactivating ONNX Controller...");
    global_pub_->on_deactivate();
}

void ONNXController::setSpeedLimit(const double &speed_limit, const bool &percentage) {
    RCLCPP_INFO(logger_, "Set speed limit");
    (void)speed_limit;
    (void)percentage;
}

void ONNXController::setPlan(const nav_msgs::msg::Path &path) {
    RCLCPP_INFO(logger_, "Got a plan");
    global_pub_->publish(path);
    global_plan_ = path;
}

void ONNXController::onnxInputCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // get latest input from model and hallucination node
    latest_onnx_input_ = *msg;
    // RCLCPP_INFO(logger_, "Receiving Input from Middle man...");
}

// geometry_msgs::msg::TwistStamped ONNXController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
//                                                                          const geometry_msgs::msg::Twist &,
//                                                                          nav2_core::GoalChecker *) {
//
//     static const float min_linear_vel = 0.003;
//     static const float max_linear_vel = .3;
//     static const float min_angular_vel = -1.4;
//     static const float max_angular_vel = 1.4;
//
//     // RCLCPP_INFO(logger_, "REACHING COMPUTE VELOCITY");
//
//     // 1) Gather your raw data (doubles) from the Float64MultiArray
//     const auto &latest =
//         latest_onnx_input_
//             .data; // std::vector<double>, contains data for neural net and obstacle data (20 obstacle data)
//
//     if (latest.size() < 1088) { // Minimum size needed for odom_x, odom_y
//         // RCLCPP_ERROR(logger_, "Input data size (%zu) is smaller than required (1088)", latest.size());
//         // Return a zero velocity command or use previously valid command
//         geometry_msgs::msg::TwistStamped cmd_vel;
//         cmd_vel.header.frame_id = pose.header.frame_id;
//         cmd_vel.twist.linear.x = 0.0;
//         cmd_vel.twist.angular.z = 0.0;
//         return cmd_vel;
//     }
//
//     const double *lidar_pointer = latest.data() + 5;
//     static const size_t num_lidar = 1080;
//
//     // 1) Populate Data:
//
//     std::vector<float> input_data_f(1085); // Only allocate what's needed for the model
//     for (size_t i = 0; i < 1085; i++) {
//         input_data_f[i] = latest[i];
//     }
//     // RCLCPP_INFO(logger_, "Pre normalize");
//
//     for (size_t i = 0; i < 20; i++) {
//         // RCLCPP_INFO(logger_, "input_data_f[%zu]: %f", i, input_data_f[i]);
//     }
//     // 2) Normalize the data from training values
//     if (feature_mins_.size() != input_data_f.size() || feature_maxs_.size() != input_data_f.size()) {
//         std::cerr << "Error: Scaler min/max dimensions don't match input data dimensions!" << std::endl;
//         std::cerr << "feature_min size: " << feature_mins_.size() << ", features_max size: " << feature_maxs_.size()
//                   << ", input_data size: " << input_data_f.size() << std::endl;
//         // Handle this error appropriately
//     } else {
//         // Apply MinMaxScaler normalization to each element
//         for (size_t i = 0; i < input_data_f.size(); i++) {
//             // Apply the same transformation that MinMaxScaler would do
//             // X_scaled = (X - X_min) / (X_max - X_min)
//             float range = feature_maxs_[i] - feature_mins_[i];
//
//             // Avoid division by zero
//             if (range > 1e-10) {
//                 input_data_f[i] = (input_data_f[i] - feature_mins_[i]) / range;
//             } else {
//                 input_data_f[i] = 0; // Or any other default for constant features
//             }
//
//             // Clip values to [0,1] range in case of out-of-bound inputs
//             input_data_f[i] = std::max(0.0f, std::min(input_data_f[i], 1.0f));
//         }
//         // RCLCPP_INFO(logger_, "Normalization worked");
//     }
//
//     for (size_t i = 0; i < 20; i++) {
//         // RCLCPP_INFO(logger_, "input_data_f[%zu]: %f", i, input_data_f[i]);
//     }
//     // 3) Define your input shape
//     std::vector<int64_t> inputShape = {1, static_cast<int64_t>(1085)};
//
//     float current_v = latest[0];
//     float current_w = latest[1];
//     float odom_x = latest[1085];
//     float odom_y = latest[1086];
//     float odom_yaw = latest[1087];
//
//     // just doing lidar for now
//     //
//     //
//
//     // std::vector<Obstacle> obstacle_data;
//     // for (size_t obstacle_counter = 1088; obstacle_counter < latest.size(); obstacle_counter += 2) {
//     //     Obstacle current_obs;
//     //     current_obs.center_x = latest[obstacle_counter];
//     //     current_obs.center_y = latest[obstacle_counter + 1];
//     //     obstacle_data.push_back(current_obs);
//     // }
//     // 4) Create the Ort tensor<float>
//     auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
//     Ort::Value inputTensor = Ort::Value::CreateTensor<float>(memory_info, input_data_f.data(), input_data_f.size(),
//                                                              inputShape.data(), inputShape.size());
//
//     // 5) Run inference
//     const char *const *input_names = input_name_.data();
//     const char *const *output_names = output_name_.data();
//     auto outputTensors = session_->Run(Ort::RunOptions{}, input_names, &inputTensor, 1, output_names, 1);
//
//     // 6) Read back float outputs, then cast to double for ROS msgs
//     float *out_f = outputTensors[0].GetTensorMutableData<float>();
//     double predicted_linear = static_cast<double>(out_f[0]);
//     double predicted_angular = static_cast<double>(out_f[1]);
//
//     // RCLCPP_INFO(logger_, "Predicted (lin, ang): %.3f, %.3f", predicted_linear, predicted_angular);
//
//     // Now we want to modulate the output of the network
//
//     RCLCPP_INFO(logger_, "current_x, current_W, , %.3f, %.3f", current_v, current_w);
//
//     geometry_msgs::msg::PoseStamped odom_pose;
//     odom_pose.header.frame_id = "odom";
//     odom_pose.header.stamp = pose.header.stamp;
//     odom_pose.pose.position.x = odom_x;
//     odom_pose.pose.position.y = odom_y;
//     odom_pose.pose.position.z = 0.0;
//     tf2::Quaternion q;
//     q.setRPY(0, 0, odom_yaw);
//     odom_pose.pose.orientation = tf2::toMsg(q);
//
//     geometry_msgs::msg::PoseStamped map_pose;
//     std::pair<float, float> cmds;
//     try {
//         map_pose = tf_->transform(odom_pose, "map", tf2::durationFromSec(0.1));
//
//         // Extract map coordinates
//         float map_x = map_pose.pose.position.x;
//         float map_y = map_pose.pose.position.y;
//         float map_yaw = tf2::getYaw(map_pose.pose.orientation);
//
//         // Visualize in map frame
//         auto viz_markers = visualize_mpc_projections(map_x, map_y, map_yaw, predicted_linear, predicted_angular,
//                                                      lidar_pointer, num_lidar);
//         trajectory_pub_->publish(viz_markers);
//
//         cmds =
//             modulation_onnx_lidar(map_x, map_y, map_yaw, predicted_linear, predicted_angular, lidar_pointer,
//             num_lidar);
//     } catch (tf2::TransformException &ex) {
//         RCLCPP_WARN(logger_, "Could not transform to map for visualization: %s", ex.what());
//
//         cmds = modulation_onnx_lidar(odom_x, odom_y, odom_yaw, predicted_linear, predicted_angular, lidar_pointer,
//                                      num_lidar);
//     }
//
//     // Add safety clamping on final commands too
//     RCLCPP_INFO(logger_, "Final commands after modulation (lin, ang): %.3f, %.3f", cmds.first, cmds.second);
//     cmds.first = std::clamp(cmds.first, min_linear_vel, max_linear_vel);
//     cmds.second = std::clamp(cmds.second, min_angular_vel, max_angular_vel);
//
//     // RCLCPP_INFO(logger_, "Final clamped commands (lin, ang): %.3f, %.3f", cmds.first, cmds.second);
//     // RCLCPP_INFO(logger_, "LOCAL GOAL DATA: %.3f, %.3f", latest[2], latest[3]);
//     // 7) Pack into TwistStamped
//     geometry_msgs::msg::TwistStamped cmd_vel;
//     cmd_vel.header.frame_id = pose.header.frame_id;
//     cmd_vel.twist.linear.x = cmds.first;
//     cmd_vel.twist.angular.z = cmds.second;
//
//     return cmd_vel;
// }
geometry_msgs::msg::TwistStamped ONNXController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                                                                         const geometry_msgs::msg::Twist &,
                                                                         nav2_core::GoalChecker *) {

    // Start overall timing
    auto t_start = std::chrono::high_resolution_clock::now();

    static const float min_linear_vel = 0.003;
    static const float max_linear_vel = .3;
    static const float min_angular_vel = -1.4;
    static const float max_angular_vel = 1.4;

    // === TIMING: Data validation ===
    auto t0 = std::chrono::high_resolution_clock::now();

    const auto &latest = latest_onnx_input_.data;

    if (latest.size() < 1088) {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = pose.header.frame_id;
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
    }

    const double *lidar_pointer = latest.data() + 5;
    static const size_t num_lidar = 1080;

    auto t1 = std::chrono::high_resolution_clock::now();

    // === TIMING: Data preparation ===
    std::vector<float> input_data_f(1085);
    for (size_t i = 0; i < 1085; i++) {
        input_data_f[i] = latest[i];
    }

    auto t2 = std::chrono::high_resolution_clock::now();

    // === TIMING: Normalization ===
    if (feature_mins_.size() != input_data_f.size() || feature_maxs_.size() != input_data_f.size()) {
        std::cerr << "Error: Scaler dimensions mismatch!" << std::endl;
    } else {
        for (size_t i = 0; i < input_data_f.size(); i++) {
            float range = feature_maxs_[i] - feature_mins_[i];
            if (range > 1e-10) {
                input_data_f[i] = (input_data_f[i] - feature_mins_[i]) / range;
            } else {
                input_data_f[i] = 0;
            }
            input_data_f[i] = std::max(0.0f, std::min(input_data_f[i], 1.0f));
        }
    }

    auto t3 = std::chrono::high_resolution_clock::now();

    // === TIMING: Tensor creation ===
    std::vector<int64_t> inputShape = {1, static_cast<int64_t>(1085)};

    // float current_v = latest[0];
    // float current_w = latest[1];
    float odom_x = latest[1085];
    float odom_y = latest[1086];
    float odom_yaw = latest[1087];

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(memory_info, input_data_f.data(), input_data_f.size(),
                                                             inputShape.data(), inputShape.size());

    auto t4 = std::chrono::high_resolution_clock::now();

    // === TIMING: ONNX INFERENCE (THE CRITICAL PART) ===
    const char *const *input_names = input_name_.data();
    const char *const *output_names = output_name_.data();

    auto outputTensors = session_->Run(Ort::RunOptions{}, input_names, &inputTensor, 1, output_names, 1);

    auto t5 = std::chrono::high_resolution_clock::now();

    // === TIMING: Output extraction ===
    float *out_f = outputTensors[0].GetTensorMutableData<float>();
    double predicted_linear = static_cast<double>(out_f[0]);
    double predicted_angular = static_cast<double>(out_f[1]);

    auto t6 = std::chrono::high_resolution_clock::now();

    // === TIMING: TF transform ===
    geometry_msgs::msg::PoseStamped odom_pose;
    odom_pose.header.frame_id = "odom";
    odom_pose.header.stamp = pose.header.stamp;
    odom_pose.pose.position.x = odom_x;
    odom_pose.pose.position.y = odom_y;
    odom_pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_yaw);
    odom_pose.pose.orientation = tf2::toMsg(q);

    geometry_msgs::msg::PoseStamped map_pose;
    std::pair<float, float> cmds;

    auto t7 = std::chrono::high_resolution_clock::now();

    try {
        map_pose = tf_->transform(odom_pose, "map", tf2::durationFromSec(0.1));

        auto t8 = std::chrono::high_resolution_clock::now();

        float map_x = map_pose.pose.position.x;
        float map_y = map_pose.pose.position.y;
        float map_yaw = tf2::getYaw(map_pose.pose.orientation);

        // === TIMING: Visualization ===
        auto viz_markers = visualize_mpc_projections(map_x, map_y, map_yaw, predicted_linear, predicted_angular,
                                                     lidar_pointer, num_lidar);
        trajectory_pub_->publish(viz_markers);

        auto t9 = std::chrono::high_resolution_clock::now();

        // === TIMING: Modulation (safety layer) ===
        cmds =
            modulation_onnx_lidar(map_x, map_y, map_yaw, predicted_linear, predicted_angular, lidar_pointer, num_lidar);

        auto t10 = std::chrono::high_resolution_clock::now();

        // === Compute timings ===
        auto validation_time = std::chrono::duration<double, std::milli>(t1 - t0).count();
        auto data_prep_time = std::chrono::duration<double, std::milli>(t2 - t1).count();
        auto normalize_time = std::chrono::duration<double, std::milli>(t3 - t2).count();
        auto tensor_create_time = std::chrono::duration<double, std::milli>(t4 - t3).count();
        auto inference_time = std::chrono::duration<double, std::milli>(t5 - t4).count();
        auto output_extract_time = std::chrono::duration<double, std::milli>(t6 - t5).count();
        auto tf_setup_time = std::chrono::duration<double, std::milli>(t8 - t7).count();
        auto viz_time = std::chrono::duration<double, std::milli>(t9 - t8).count();
        auto modulation_time = std::chrono::duration<double, std::milli>(t10 - t9).count();
        auto total_time = std::chrono::duration<double, std::milli>(t10 - t_start).count();

        // === Statistics tracking ===
        static double max_inference = 0.0;
        static double max_total = 0.0;
        static int call_count = 0;
        static double sum_inference = 0.0;
        static double sum_total = 0.0;

        call_count++;
        sum_inference += inference_time;
        sum_total += total_time;
        max_inference = std::max(max_inference, inference_time);
        max_total = std::max(max_total, total_time);

        // === Log every 2 seconds ===
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000,
                             "\n=== ONNX Controller Timing (ms) ===\n"
                             "  Validation:      %6.2f\n"
                             "  Data prep:       %6.2f\n"
                             "  Normalization:   %6.2f\n"
                             "  Tensor create:   %6.2f\n"
                             "  INFERENCE:    %6.2f \n"
                             "  Output extract:  %6.2f\n"
                             "  TF setup:        %6.2f\n"
                             "  Visualization:   %6.2f\n"
                             "  Modulation:      %6.2f\n"
                             "  ─────────────────────\n"
                             "  TOTAL:           %6.2f / 100.0ms budget (10Hz)\n"
                             "\n"
                             "Stats (n=%d):\n"
                             "  Avg inference:   %6.2f ms\n"
                             "  Max inference:   %6.2f ms\n"
                             "  Avg total:       %6.2f ms\n"
                             "  Max total:       %6.2f ms\n"
                             "%s",
                             validation_time, data_prep_time, normalize_time, tensor_create_time, inference_time,
                             output_extract_time, tf_setup_time, viz_time, modulation_time, total_time, call_count,
                             sum_inference / call_count, max_inference, sum_total / call_count, max_total,
                             (total_time > 50.0) ? "OVERRUN for 20Hz!" : "OK for 20Hz");

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(logger_, "Could not transform to map for visualization: %s", ex.what());
        cmds = modulation_onnx_lidar(odom_x, odom_y, odom_yaw, predicted_linear, predicted_angular, lidar_pointer,
                                     num_lidar);
    }

    // === TIMING: Final clamping ===
    cmds.first = std::clamp(cmds.first, min_linear_vel, max_linear_vel);
    cmds.second = std::clamp(cmds.second, min_angular_vel, max_angular_vel);

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.twist.linear.x = cmds.first;
    cmd_vel.twist.angular.z = cmds.second;

    // auto t_end = std::chrono::high_resolution_clock::now();
    // auto total_with_msg = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    return cmd_vel;
}
} // namespace onnx_controller
