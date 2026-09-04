// Copyright 2026 jiewang

#ifndef GLOBAL_PLANNER__GLOBAL_PLANNER_NODE_HPP_
#define GLOBAL_PLANNER__GLOBAL_PLANNER_NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "global_planner/terrain_map.hpp"
#include "global_planner/weighted_astar.hpp"

namespace global_planner
{

class GlobalPlannerNode : public rclcpp::Node
{
public:
  GlobalPlannerNode();
  ~GlobalPlannerNode() override;

private:
  void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  bool transformToMap(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped * output);
  void workerLoop();
  nav_msgs::msg::Path makePath(
    const PlanResult & result,
    const geometry_msgs::msg::PoseStamped & goal) const;

  std::string map_frame_;
  double snap_radius_{1.0};
  double snap_vertical_tolerance_{1.0};
  double path_z_offset_{0.0};

  TerrainMap terrain_map_;
  std::unique_ptr<WeightedAStar> planner_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex request_mutex_;
  std::condition_variable request_cv_;
  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped pending_goal_;
  bool have_start_{false};
  bool have_request_{false};
  bool stopping_{false};
  std::atomic<uint64_t> request_generation_{0};
  std::thread worker_;
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__GLOBAL_PLANNER_NODE_HPP_
