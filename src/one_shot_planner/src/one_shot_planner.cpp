
#include "one_shot_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>

namespace one_shot_planner {

void OneShotPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                               std::shared_ptr<tf2_ros::Buffer> tf,
                               std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    node_ = parent.lock(); // convert WeakPtr -> SharedPtr on Humble
    name_ = std::move(name);
    tf_ = std::move(tf);
    costmap_ros_ = std::move(costmap_ros);

    plan_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
        "/plan_barn", rclcpp::QoS(1).reliable(), std::bind(&OneShotPlanner::planCallback, this, std::placeholders::_1));

    if (!node_) {
        throw std::runtime_error("OneShotPlanner: Lifecycle node expired in configure()");
    }
    RCLCPP_INFO(node_->get_logger(), "Configuring %s (OneShotPlanner)", name_.c_str());
}

void OneShotPlanner::cleanup() {
    RCLCPP_INFO(node_->get_logger(), "Cleaning up %s (OneShotPlanner)", name_.c_str());
    computed_ = false;
    last_plan_ = nav_msgs::msg::Path();
}

void OneShotPlanner::activate() { RCLCPP_INFO(node_->get_logger(), "Activating %s (OneShotPlanner)", name_.c_str()); }

void OneShotPlanner::deactivate() {
    RCLCPP_INFO(node_->get_logger(), "Deactivating %s (OneShotPlanner)", name_.c_str());
}

nav_msgs::msg::Path OneShotPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                                               const geometry_msgs::msg::PoseStamped &goal) {
    if (computed_) {
        RCLCPP_WARN(node_->get_logger(), "[%s] Returning previously computed plan", name_.c_str());
        return last_plan_;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = node_->now();
    path.header.frame_id = start.header.frame_id; // "map" typically

    // Simple straight-line interpolation: replace with your BARN-loaded path
    const int N = 50;
    for (int i = 0; i <= N; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(N);
        geometry_msgs::msg::PoseStamped p;
        p.header = path.header;
        p.pose.position.x = start.pose.position.x + t * (goal.pose.position.x - start.pose.position.x);
        p.pose.position.y = start.pose.position.y + t * (goal.pose.position.y - start.pose.position.y);
        p.pose.position.z = 0.0;
        p.pose.orientation = start.pose.orientation; // keep simple
        path.poses.push_back(std::move(p));
    }

    last_plan_ = path;
    computed_ = true;
    RCLCPP_INFO(node_->get_logger(), "[%s] Computed plan with %zu poses", name_.c_str(), path.poses.size());
    return path;
}
void OneShotPlanner::planCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    external_plan_ = *msg;
    has_external_plan_ = true;
    RCLCPP_INFO(node_->get_logger(), "[%s] Received /plan_barn with %zu poses", name_.c_str(), msg->poses.size());
}
} // namespace one_shot_planner

// Export type must match your XML's <class type="...">
PLUGINLIB_EXPORT_CLASS(one_shot_planner::OneShotPlanner, nav2_core::GlobalPlanner)
