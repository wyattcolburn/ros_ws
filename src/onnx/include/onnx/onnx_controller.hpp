// /**
//  * Header File for a controller connecting an ONNX model to the Nav2 Stack
//  * Based upon Pure Pursuit Controller in Nav2 tutorials
//  */

#ifndef ONNX_CONTROLLER__ONNX_CONTROLLER_HPP_
#define ONNX_CONTROLLER__ONNX_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "onnxruntime_cxx_api.h"

namespace onnx_controller
{
class ONNXController : public nav2_core::Controller
{
    public:
        ONNXController() = default;
        ~ONNXController() override = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
            const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
        
        void cleanup() override;
        void activate() override;
        void deactivate() override;
        void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped &pose,
            const geometry_msgs::msg::Twist &velocity,
            nav2_core::GoalChecker *goal_checker) override;
        
        void setPlan(const nav_msgs::msg::Path &path) override;
    
    protected:
        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
            
        std::string plugin_name_;
        rclcpp::Logger logger_ {rclcpp::get_logger("OnnxController")};
        rclcpp::Clock::SharedPtr clock_;

        nav_msgs::msg::Path current_plan_;
        double speed_limit_;
        double max_angular_vel_;
        double max_linear_vel_;
        
        std::string model_path_;
        std::unique_ptr<Ort::Session> session_;
        std::vector<const char *> input_name_, output_name_;

        nav_msgs::msg::Path global_plan_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr onnx_sub_;
		std_msgs::msg::Float64MultiArray latest_onnx_input_;

        void onnxInputCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

	private:
		std::vector<float> feature_mins_;
		std::vector<float> feature_maxs_;
		// Add this helper function
		std::string scaler_min_path; 
                std::string scaler_max_path;
		};
}   // namespace onnx_controller

#endif  // ONNX_CONTROLLER__ONNX_CONTROLLER_HPP_
