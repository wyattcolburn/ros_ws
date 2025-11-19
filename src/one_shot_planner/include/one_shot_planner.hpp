
#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace one_shot_planner {

class OneShotPlanner : public nav2_core::GlobalPlanner {
  public:
    OneShotPlanner() = default;
    ~OneShotPlanner() override = default;

    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                   std::shared_ptr<tf2_ros::Buffer> tf,
                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                                   const geometry_msgs::msg::PoseStamped &goal) override;

  private:
    void planCallback(const nav_msgs::msg::Path::SharedPtr msg);

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string name_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    bool computed_{false};
    nav_msgs::msg::Path last_plan_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    bool has_external_plan_{false};
    nav_msgs::msg::Path external_plan_;
};

} // namespace one_shot_planner
