
#include "onnx/onnx_controller.hpp"

#include <algorithm>
#include <array>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

#include "mpc.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(onnx_controller::ONNXController, nav2_core::Controller)

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
namespace {

// Ensure the ONNX Runtime Env lives as long as the process
std::unique_ptr<Ort::Env> g_env;

} // anonymous namespace

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

    // Parameters
    declare_parameter_if_not_declared(node, plugin_name_ + ".model_path",
                                      rclcpp::ParameterValue("/path/to/model.onnx"));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));

    node->get_parameter(plugin_name_ + ".model_path", model_path_);
    node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
    node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);

    // Create (or reuse) a global Ort Env
    if (!g_env) {
        g_env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "onnx_controller_env");
    }

    // Session options (tune if needed)
    Ort::SessionOptions session_options;
    // session_options.SetIntraOpNumThreads(1);
    // session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    // Create session
    session_ = std::make_unique<Ort::Session>(*g_env, model_path_.c_str(), session_options);

    RCLCPP_INFO(logger_, "Expecting ONNX inputs: lidar [1,1080,1], state [1,5]; output: cmd_out");
    // Publishers / Subscribers
    global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);

    onnx_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/neuralNetInput", rclcpp::QoS(10), std::bind(&ONNXController::onnxInputCallback, this, std::placeholders::_1));
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
    RCLCPP_INFO(logger_, "Set speed limit (unused in this controller)");
    (void)speed_limit;
    (void)percentage;
}

void ONNXController::setPlan(const nav_msgs::msg::Path &path) {
    RCLCPP_INFO(logger_, "Got a plan with %zu poses", path.poses.size());
    global_pub_->publish(path);
    global_plan_ = path;
}

void ONNXController::onnxInputCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    latest_onnx_input_ = *msg; // store latest packet (Float64MultiArray)
}

geometry_msgs::msg::TwistStamped ONNXController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                                                                         const geometry_msgs::msg::Twist &,
                                                                         nav2_core::GoalChecker *) {
    // Safety limits (can also use params if you prefer)
    static const float min_linear_vel = 0.003f;
    static const float max_linear_vel = 0.3f;
    static const float min_angular_vel = -1.4f;
    static const float max_angular_vel = 1.4f;

    // Expected packet layout from obsValid:
    // [0]=v, [1]=w, [2]=goal_x, [3]=goal_y, [4]=goal_yaw,
    // [5..1084]=1080 lidar beams,
    // [1085]=odom_x, [1086]=odom_y, [1087]=odom_yaw,
    // [1088..]= (optional obstacle info, unused here)
    const auto &latest = latest_onnx_input_.data;

    // Minimal size check (we need at least up to odom_yaw)
    if (latest.size() < 1088) {
        geometry_msgs::msg::TwistStamped zero;
        zero.header.frame_id = pose.header.frame_id;
        zero.twist.linear.x = 0.0;
        zero.twist.angular.z = 0.0;
        return zero;
    }

    // Extract pointers for downstream modulation
    const double *lidar_pointer = latest.data() + 5;
    static constexpr size_t kNumLidar = 1080;

    // Prepare ONNX inputs (NO external normalization; model has Normalization layers)
    // 1) LiDAR input: [1, L, 1]
    std::vector<float> lidar_in(kNumLidar);
    for (size_t i = 0; i < kNumLidar; ++i) {
        // cast double->float, keep raw ranges (the model normalizes internally)
        lidar_in[i] = static_cast<float>(latest[5 + i]);
    }
    const std::array<int64_t, 3> lidar_shape{1, static_cast<int64_t>(kNumLidar), 1};

    // 2) State input: [1, 5] = [v,w,goal_x,goal_y,goal_yaw]
    static constexpr size_t kStateDims = 5;
    std::array<float, kStateDims> state_in = {
        static_cast<float>(latest[0]), static_cast<float>(latest[1]), static_cast<float>(latest[2]),
        static_cast<float>(latest[3]), static_cast<float>(latest[4]),
    };
    const std::array<int64_t, 2> state_shape{1, static_cast<int64_t>(kStateDims)};

    // Also pull robot pose for modulation
    float odom_x = static_cast<float>(latest[1085]);
    float odom_y = static_cast<float>(latest[1086]);
    float odom_yaw = static_cast<float>(latest[1087]);

    // Create Ort tensors
    //
    //
    auto mem = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    // Build tensors as before:
    Ort::Value lidar_tensor =
        Ort::Value::CreateTensor<float>(mem, lidar_in.data(), lidar_in.size(), lidar_shape.data(), lidar_shape.size());
    Ort::Value state_tensor =
        Ort::Value::CreateTensor<float>(mem, state_in.data(), state_in.size(), state_shape.data(), state_shape.size());

    // Just hardcode the names that match your exported graph:
    static const char *INPUT_NAMES[] = {"lidar", "state"};
    static const char *OUTPUT_NAMES[] = {"cmd_out"};

    Ort::Value inputs[] = {std::move(lidar_tensor), std::move(state_tensor)};

    // Run
    auto outputs = session_->Run(Ort::RunOptions{}, INPUT_NAMES, inputs, 2, OUTPUT_NAMES, 1);

    // Output [1,2]
    float *out_f = outputs[0].GetTensorMutableData<float>();
    double predicted_linear = static_cast<double>(out_f[0]);
    double predicted_angular = static_cast<double>(out_f[1]);

    // Optional: log
    // RCLCPP_INFO(logger_, "NN raw (v, w) = (%.3f, %.3f)", predicted_linear, predicted_angular);

    // Your existing safety modulation using raw LiDAR + robot pose
    std::pair<float, float> cmds =
        modulation_onnx_lidar(odom_x, odom_y, odom_yaw, static_cast<float>(predicted_linear),
                              static_cast<float>(predicted_angular), lidar_pointer, kNumLidar);

    // Clamp
    cmds.first = std::clamp(cmds.first, min_linear_vel, max_linear_vel);
    cmds.second = std::clamp(cmds.second, min_angular_vel, max_angular_vel);

    // Pack command
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.twist.linear.x = cmds.first;
    cmd_vel.twist.angular.z = cmds.second;

    return cmd_vel;
}

} // namespace onnx_controller
