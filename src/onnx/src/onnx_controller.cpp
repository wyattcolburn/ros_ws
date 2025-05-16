#include <algorithm>
#include <string>
#include <memory>
#include <fstream>
#include "onnx/onnx_controller.hpp"

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

#include "mpc.hpp"
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(onnx_controller::ONNXController, nav2_core::Controller)


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

		declare_parameter_if_not_declared(
			node, plugin_name_ + ".scaler_min_path", rclcpp::ParameterValue(
            "/home/wyattcolburn/ros_ws/src/onnx/src/scaler_mins.txt"));
		declare_parameter_if_not_declared(
			node, plugin_name_ + ".scaler_max_path", rclcpp::ParameterValue( "/home/wyattcolburn/ros_ws/src/onnx/src/scaler_max.txt"));


        node->get_parameter(plugin_name_ + ".model_path", model_path_);
        node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
        node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
		

		std::string scaler_min_path, scaler_max_path;
		node->get_parameter(plugin_name_ + ".scaler_min_path", scaler_min_path);
		node->get_parameter(plugin_name_ + ".scaler_max_path", scaler_max_path);
    
		// Load scaler parameters
		if (!loadScalerParameters(scaler_min_path, scaler_max_path)) {
			RCLCPP_WARN(logger_, "Failed to load scaler parameters, using default normalization");
			// Initialize with default values
			feature_mins_.resize(1085, 0.0f);
			feature_maxs_.resize(1085, 1.0f);
		}
        // Load ONNX Model
        Ort::Env env;
        Ort::SessionOptions session_options;

        session_ = std::make_unique<Ort::Session>(env, model_path_.c_str(), session_options);
        
        // Get input and output names
        Ort::AllocatorWithDefaultOptions ort_alloc;
        Ort::AllocatedStringPtr inputName = session_->GetInputNameAllocated(0, ort_alloc);
        Ort::AllocatedStringPtr outputName = session_->GetOutputNameAllocated(0, ort_alloc);
        input_name_ = { inputName.get()};
        output_name_ = { outputName.get()};
        inputName.release();
        outputName.release();

        global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
        onnx_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/neuralNetInput", rclcpp::QoS(10),
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
		RCLCPP_INFO(logger_, "Set speed limit");
        (void) speed_limit;
        (void) percentage;
    }


    void ONNXController::setPlan(const nav_msgs::msg::Path &path)
    {
		RCLCPP_INFO(logger_, "Got a plan");
        global_pub_->publish(path);
        global_plan_ = path;
    }

    void ONNXController::onnxInputCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // get latest input from model and hallucination node
        latest_onnx_input_ = *msg;
        RCLCPP_INFO(
            logger_,
            "Receiving Input from Middle man..."
        );
    }
	geometry_msgs::msg::TwistStamped ONNXController::computeVelocityCommands(
		const geometry_msgs::msg::PoseStamped &pose,
		const geometry_msgs::msg::Twist &, 
		nav2_core::GoalChecker *)
	{
	  
		const float min_linear_vel = 0;
		const float max_linear_vel = .3;
		const float min_angular_vel = -1.4;
		const float max_angular_vel = 1.4;
	  
		RCLCPP_INFO(logger_, "REACHING COMPUTE VELOCITY");

	  // 1) Gather your raw data (doubles) from the Float64MultiArray
	    const auto &latest = latest_onnx_input_.data;  // std::vector<double>, contains data for neural net and obstacle data (20 obstacle data)
	  
	    if (latest.size() < 1088) {  // Minimum size needed for odom_x, odom_y
		  RCLCPP_ERROR(logger_, "Input data size (%zu) is smaller than required (1088)", latest.size());
		  // Return a zero velocity command or use previously valid command
		  geometry_msgs::msg::TwistStamped cmd_vel;
		  cmd_vel.header.frame_id = pose.header.frame_id;
		  cmd_vel.twist.linear.x = 0.0;
		  cmd_vel.twist.angular.z = 0.0;
		  return cmd_vel;
	    }


	    std::vector<float> input_data_f(1085);  // Only allocate what's needed for the model
		// 2) Normalize the data from training values
		for (size_t i = 0; i < 1085; ++i) {
			// Apply MinMaxScaler normalization
			if (feature_maxs_[i] > feature_mins_[i]) {  // Avoid division by zero
				input_data_f[i] = static_cast<float>((latest[i] - feature_mins_[i]) / 
												   (feature_maxs_[i] - feature_mins_[i]));
			} else {
				input_data_f[i] = 0.0f;  // Handle case where min == max
			}
		}

	  // 3) Define your input shape
	    std::vector<int64_t> inputShape = {1, static_cast<int64_t>(1085)};

	    float odom_x = latest[1086];
	    float odom_y = latest[1087];


	    std::vector<Obstacle> obstacle_data; 
	    for (size_t obstacle_counter = 1088; obstacle_counter < latest.size(); obstacle_counter+=2) {
		    Obstacle current_obs;
		    current_obs.center_x = latest[obstacle_counter];
		    current_obs.center_y = latest[obstacle_counter + 1 ];
		    obstacle_data.push_back(current_obs);
	    }
	  // 4) Create the Ort tensor<float>
	    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
	    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
		    memory_info,
		    input_data_f.data(),
		    input_data_f.size(),
		    inputShape.data(),
		    inputShape.size()
	    );

	  // 5) Run inference
	    const char* const* input_names  = input_name_.data();
	    const char* const* output_names = output_name_.data();
	    auto outputTensors = session_->Run(
		  Ort::RunOptions{}, 
		  input_names, &inputTensor, 1, 
		  output_names, 1
	    );

	    // 6) Read back float outputs, then cast to double for ROS msgs
	    float* out_f = outputTensors[0].GetTensorMutableData<float>();
	    double predicted_linear  = static_cast<double>(out_f[0]);
	    double predicted_angular = static_cast<double>(out_f[1]);

	    RCLCPP_INFO(logger_, "Predicted (lin, ang): %.3f, %.3f",
				  predicted_linear, predicted_angular);

	  // Now we want to modulate the output of the network

	    std::pair<float, float> cmds =modulation_onnx(odom_x, odom_y,predicted_linear, predicted_angular, obstacle_data);
	  
	    RCLCPP_INFO(logger_, "Final commands after modulation (lin, ang): %.3f, %.3f",
			  cmds.first, cmds.second);

	  // Add safety clamping on final commands too
	    cmds.first = std::clamp(cmds.first, min_linear_vel, max_linear_vel);
	    cmds.second = std::clamp(cmds.second, min_angular_vel, max_angular_vel);

	    RCLCPP_INFO(logger_, "Final clamped commands (lin, ang): %.3f, %.3f",
	    cmds.first, cmds.second);
	    //7) Pack into TwistStamped
	    geometry_msgs::msg::TwistStamped cmd_vel;
	    cmd_vel.header.frame_id = pose.header.frame_id;
	    cmd_vel.twist.linear.x  = cmds.first;
	    cmd_vel.twist.angular.z = cmds.second;

	    return cmd_vel;
	  }



	bool ONNXController::loadScalerParameters(const std::string& min_file, const std::string& max_file) {
		std::ifstream min_stream(min_file);
		std::ifstream max_stream(max_file);
		
		if (!min_stream.is_open() || !max_stream.is_open()) {
			RCLCPP_ERROR(logger_, "Failed to open scaler parameter files: %s or %s", 
						min_file.c_str(), max_file.c_str());
			return false;
		}
		
		// Clear existing data
		feature_mins_.clear();
		feature_maxs_.clear();
		
		// Read min values
		float value;
		while (min_stream >> value) {
			feature_mins_.push_back(value);
		}
		
		// Read max values
		while (max_stream >> value) {
			feature_maxs_.push_back(value);
		}
		
		// Verify we have the right number of parameters
		if (feature_mins_.size() != 1085 || feature_maxs_.size() != 1085) {
			RCLCPP_ERROR(logger_, "Incorrect number of scaler parameters: mins: %zu, maxs: %zu (expected 1085)",
						feature_mins_.size(), feature_maxs_.size());
			return false;
		}
		
		RCLCPP_INFO(logger_, "Successfully loaded scaler parameters");
		return true;
	}
}
