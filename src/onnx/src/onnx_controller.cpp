#include <algorithm>
#include <string>
#include <memory>

#include "onnx/onnx_controller.hpp"

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

#include "mpc.hpp"
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace onnx_controller
{
    void ONNXController::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent;

        auto node = node_.lock();

        costmap_ros_ = costmap_ros;
        tf_ = tf;
        plugin_name_ = name;
        logger_ = node->get_logger();
        clock_ = node->get_clock();

        RCLCPP_INFO(logger_, "Configuring ONNX Controller...");

        // Declare parameters
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".model_path", rclcpp::ParameterValue(
                "/pathOfModel.onnx"));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(
                0.5));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(
                1.0));
        
        node->get_parameter(plugin_name_ + ".model_path", model_path_);
        node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
        node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);

        // Load ONNX Model
        Ort::Env env;
        Ort::SessionOptions session_options;

        session_ = std::make_unique<Ort::Session>(env, model_path_.c_str(), session_options);
        
        // Get input and output names
        Ort::AllocatorWithDefaultOptions ort_alloc;
        Ort::AllocatedStringPtr inputName = session_->GetInputNameAllocated(0, ort_alloc);
        Ort::AllocatedStringPtr outputName = session_->GetInputNameAllocated(0, ort_alloc);
        input_name_ = { inputName.get()};
        output_name_ = { outputName.get()};
        inputName.release();
        outputName.release();

        global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
        onnx_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/onnx_input", rclcpp::QoS(10),
            std::bind(&ONNXController::onnxInputCallback, this, std::placeholders::_1)
        );
    }

    void ONNXController::cleanup()
    {
        RCLCPP_INFO(
            logger_,
            "Cleaning up ONNX Controller..."
        );
        session_.reset();
        global_pub_.reset();
    }

    void ONNXController::activate()
    {
        RCLCPP_INFO(
            logger_,
            "Activating ONNX Controller..."
        );
        global_pub_->on_activate();
    }

    void ONNXController::deactivate()
    {
        RCLCPP_INFO(
            logger_,
            "Deactivating ONNX Controller..."
        );
        global_pub_->on_deactivate();
    }

    void ONNXController::setSpeedLimit(const double &speed_limit, const bool &percentage){
        (void) speed_limit;
        (void) percentage;
    }

    geometry_msgs::msg::TwistStamped ONNXController::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &, //do i need this line?
        nav2_core::GoalChecker *goal_checker)
    {
        (void) goal_checker;    // Not needed

        // Define Shape
        // 1080 From LIDAR
        // 3 From Odom (x,y,phi)
        // 3 From local goals (x,y,phi)
        std::vector<int64_t> inputShape = {1, 1086};    // Vector vs Array??
        std::vector<double> input_data = latest_onnx_input_.data;
        
        // Define Array
        std::vector<double> input;

        // Define tensor
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        Ort::Value inputTensor = Ort::Value::CreateTensor<double>(memory_info, input_data.data(),
                input_data.size(), inputShape.data(), inputShape.size()); 

        const char* const* input_names = input_name_.data();
        const char* const* output_names = output_name_.data();

        Ort::RunOptions runOptions;
        std::vector<Ort::Value> outputTensor = session_->Run(runOptions, input_names, &inputTensor, 1, output_names, 1);

        double* output_data = outputTensor[0].GetTensorMutableData<double>();
        double predicted_linear = output_data[0];
        double predicted_angular = output_data[1];

        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = pose.header.frame_id;
        cmd_vel.twist.linear.x = predicted_linear;
        cmd_vel.twist.angular.z = predicted_angular;

        // TODO: Clamp the output 

        return cmd_vel;
    }

    void ONNXController::setPlan(const nav_msgs::msg::Path &path)
    {
        global_pub_->publish(path);
        global_plan_ = path;
    }

    void ONNXController::onnxInputCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // get latest input from model and hallucination node
        latest_onnx_input_ = *msg;
    }


}
