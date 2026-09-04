// Copyright 2026 jiewang

#include "global_planner/global_planner_node.hpp"

#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace global_planner
{

GlobalPlannerNode::GlobalPlannerNode()
: Node("global_planner")
{
  const std::string map_path = declare_parameter<std::string>(
    "map_path", "/home/jiewang/nav_ros2_ws/src/mapping/map_data/pointcloud2.pcd");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  const std::string start_topic = declare_parameter<std::string>("start_topic", "/initialpose");
  const std::string goal_topic = declare_parameter<std::string>("goal_topic", "/goal_3d");
  const std::string path_topic = declare_parameter<std::string>("path_topic", "/global_path");

  TerrainMapConfig map_config;
  map_config.resolution_xy = declare_parameter<double>("resolution_xy", 0.20);
  map_config.resolution_z = declare_parameter<double>("resolution_z", 0.10);
  map_config.layer_merge_height = declare_parameter<double>("layer_merge_height", 0.12);
  map_config.min_traversability = declare_parameter<double>("min_traversability", 128.0);
  map_config.min_traversable_fraction =
    declare_parameter<double>("min_traversable_fraction", 0.60);
  map_config.robot_radius = declare_parameter<double>("robot_radius", 0.25);
  map_config.inflation_vertical_tolerance =
    declare_parameter<double>("inflation_vertical_tolerance", 0.30);
  map_config.min_inflation_obstacle_fraction =
    declare_parameter<double>("min_inflation_obstacle_fraction", 0.50);
  snap_radius_ = declare_parameter<double>("snap_radius", 1.0);
  snap_vertical_tolerance_ = declare_parameter<double>("snap_vertical_tolerance", 1.0);
  path_z_offset_ = declare_parameter<double>("path_z_offset", 0.0);

  PlannerConfig planner_config;
  planner_config.neighbor_cell_radius = declare_parameter<int>("neighbor_cell_radius", 1);
  planner_config.max_neighbor_distance = declare_parameter<double>("max_neighbor_distance", 0.45);
  planner_config.max_step_height = declare_parameter<double>("max_step_height", 0.28);
  planner_config.max_slope_degrees = declare_parameter<double>("max_slope_degrees", 55.0);
  planner_config.traversability_weight =
    declare_parameter<double>("traversability_weight", 2.0);
  planner_config.slope_weight = declare_parameter<double>("slope_weight", 0.20);
  planner_config.step_weight = declare_parameter<double>("step_weight", 1.0);
  planner_config.heuristic_weight = declare_parameter<double>("heuristic_weight", 1.15);
  planner_config.planning_timeout = declare_parameter<double>("planning_timeout", 2.0);
  planner_config.enable_connectivity_precheck =
    declare_parameter<bool>("enable_connectivity_precheck", true);
  planner_config.enable_path_smoothing = declare_parameter<bool>("enable_path_smoothing", true);
  planner_config.smoothing_sample_step =
    declare_parameter<double>("smoothing_sample_step", 0.10);
  planner_config.smoothing_xy_tolerance =
    declare_parameter<double>("smoothing_xy_tolerance", 0.16);
  planner_config.smoothing_height_tolerance =
    declare_parameter<double>("smoothing_height_tolerance", 0.10);
  planner_config.smoothing_max_segment_length =
    declare_parameter<double>("smoothing_max_segment_length", 5.0);

  if (planner_config.neighbor_cell_radius < 1 || planner_config.max_neighbor_distance <= 0.0 ||
    planner_config.max_step_height < 0.0 || planner_config.max_slope_degrees <= 0.0 ||
    planner_config.traversability_weight < 0.0 || planner_config.slope_weight < 0.0 ||
    planner_config.step_weight < 0.0 || planner_config.heuristic_weight < 1.0 ||
    snap_radius_ < 0.0 || snap_vertical_tolerance_ < 0.0 ||
    (planner_config.enable_path_smoothing &&
    (planner_config.smoothing_sample_step <= 0.0 ||
    planner_config.smoothing_xy_tolerance <= 0.0 ||
    planner_config.smoothing_height_tolerance < 0.0 ||
    planner_config.smoothing_max_segment_length <= 0.0)))
  {
    throw std::runtime_error("invalid planner parameters");
  }

  terrain_map_ = TerrainMap(map_config);
  RCLCPP_INFO(get_logger(), "Loading traversability map: %s", map_path.c_str());
  std::string error;
  const auto load_begin = std::chrono::steady_clock::now();
  if (!terrain_map_.loadPcd(map_path, &error)) {
    throw std::runtime_error(error);
  }
  const double load_ms = 1000.0 * std::chrono::duration<double>(
    std::chrono::steady_clock::now() - load_begin).count();
  RCLCPP_INFO(
    get_logger(),
    "Map ready: raw=%zu, traversable_raw=%zu, inflated=%zu, sparse_nodes=%zu, load=%.1f ms",
    terrain_map_.rawPointCount(), terrain_map_.traversablePointCount(),
    terrain_map_.inflatedNodeCount(), terrain_map_.nodes().size(), load_ms);

  planner_ = std::make_unique<WeightedAStar>(terrain_map_, planner_config);
  RCLCPP_INFO(
    get_logger(), "Topology ready: components=%zu largest=%zu build=%.1f ms",
    planner_->componentCount(), planner_->largestComponentSize(),
    planner_->connectivityBuildTimeMs());
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::QoS path_qos(rclcpp::KeepLast(1));
  path_qos.reliable().transient_local();
  path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, path_qos);
  start_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    start_topic, rclcpp::QoS(10),
    std::bind(&GlobalPlannerNode::startCallback, this, std::placeholders::_1));
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic, rclcpp::QoS(10),
    std::bind(&GlobalPlannerNode::goalCallback, this, std::placeholders::_1));

  worker_ = std::thread(&GlobalPlannerNode::workerLoop, this);
  RCLCPP_INFO(
    get_logger(), "Ready: start=%s goal=%s path=%s frame=%s",
    start_topic.c_str(), goal_topic.c_str(), path_topic.c_str(), map_frame_.c_str());
}

GlobalPlannerNode::~GlobalPlannerNode()
{
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    stopping_ = true;
    have_request_ = false;
  }
  request_generation_.fetch_add(1U, std::memory_order_relaxed);
  request_cv_.notify_one();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void GlobalPlannerNode::startCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped input;
  input.header = msg->header;
  input.pose = msg->pose.pose;
  geometry_msgs::msg::PoseStamped transformed;
  if (!transformToMap(input, &transformed)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    start_pose_ = transformed;
    have_start_ = true;
  }
  RCLCPP_INFO(
    get_logger(), "Start updated: [%.3f, %.3f, %.3f]",
    transformed.pose.position.x, transformed.pose.position.y, transformed.pose.position.z);
}

void GlobalPlannerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped transformed;
  if (!transformToMap(*msg, &transformed)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    if (!have_start_) {
      RCLCPP_WARN(get_logger(), "Goal ignored because no valid /initialpose has been received");
      return;
    }
    pending_goal_ = transformed;
    have_request_ = true;
    request_generation_.fetch_add(1U, std::memory_order_relaxed);
  }
  request_cv_.notify_one();
}

bool GlobalPlannerNode::transformToMap(
  const geometry_msgs::msg::PoseStamped & input,
  geometry_msgs::msg::PoseStamped * output)
{
  if (input.header.frame_id.empty() || input.header.frame_id == map_frame_) {
    *output = input;
    output->header.frame_id = map_frame_;
    return true;
  }
  try {
    *output = tf_buffer_->transform(input, map_frame_, tf2::durationFromSec(0.2));
    return true;
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN(
      get_logger(), "Cannot transform pose from '%s' to '%s': %s",
      input.header.frame_id.c_str(), map_frame_.c_str(), exception.what());
    return false;
  }
}

void GlobalPlannerNode::workerLoop()
{
  while (rclcpp::ok()) {
    geometry_msgs::msg::PoseStamped start;
    geometry_msgs::msg::PoseStamped goal;
    uint64_t generation = 0U;
    {
      std::unique_lock<std::mutex> lock(request_mutex_);
      request_cv_.wait(lock, [this]() {return stopping_ || have_request_;});
      if (stopping_) {
        return;
      }
      start = start_pose_;
      goal = pending_goal_;
      have_request_ = false;
      generation = request_generation_.load(std::memory_order_relaxed);
    }

    const auto & sp = start.pose.position;
    const auto & gp = goal.pose.position;
    const int32_t start_id = terrain_map_.nearestNode(
      sp.x, sp.y, sp.z, snap_radius_, snap_vertical_tolerance_);
    const int32_t goal_id = terrain_map_.nearestNode(
      gp.x, gp.y, gp.z, snap_radius_, snap_vertical_tolerance_);
    if (start_id < 0 || goal_id < 0) {
      RCLCPP_WARN(
        get_logger(), "Planning failed: cannot snap %s to traversable terrain",
        start_id < 0 ? "start" : "goal");
      continue;
    }

    const auto & snapped_start = terrain_map_.nodes()[static_cast<uint32_t>(start_id)];
    const auto & snapped_goal = terrain_map_.nodes()[static_cast<uint32_t>(goal_id)];
    RCLCPP_INFO(
      get_logger(),
      "Planning %u -> %u, snapped start=[%.2f %.2f %.2f], goal=[%.2f %.2f %.2f]",
      static_cast<uint32_t>(start_id), static_cast<uint32_t>(goal_id),
      snapped_start.x, snapped_start.y, snapped_start.z,
      snapped_goal.x, snapped_goal.y, snapped_goal.z);

    PlanResult result = planner_->plan(
      static_cast<uint32_t>(start_id), static_cast<uint32_t>(goal_id),
      &request_generation_, generation);
    if (result.canceled) {
      continue;
    }
    if (!result.success) {
      RCLCPP_WARN(
        get_logger(), "Planning failed: %s (expanded=%zu, %.1f ms)",
        result.message.c_str(), result.expanded_nodes, result.planning_time_ms);
      continue;
    }
    const auto path = makePath(result, goal);
    path_pub_->publish(path);
    RCLCPP_INFO(
      get_logger(),
      "Path published: raw=%zu smoothed=%zu poses=%zu length=%.2f m expanded=%zu time=%.1f ms",
      result.raw_node_count, result.node_ids.size(), path.poses.size(), result.path_length,
      result.expanded_nodes, result.planning_time_ms);
  }
}

nav_msgs::msg::Path GlobalPlannerNode::makePath(
  const PlanResult & result, const geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = map_frame_;
  path.header.stamp = now();
  path.poses.reserve(result.node_ids.size() + 1U);
  const auto & nodes = terrain_map_.nodes();
  for (std::size_t i = 0; i < result.node_ids.size(); ++i) {
    const auto & point = nodes[result.node_ids[i]];
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z + path_z_offset_;
    if (i + 1U < result.node_ids.size()) {
      const auto & next = nodes[result.node_ids[i + 1U]];
      const double yaw = std::atan2(next.y - point.y, next.x - point.x);
      pose.pose.orientation.z = std::sin(0.5 * yaw);
      pose.pose.orientation.w = std::cos(0.5 * yaw);
    } else {
      const double dx = goal.pose.position.x - point.x;
      const double dy = goal.pose.position.y - point.y;
      if (std::hypot(dx, dy) > 1e-6) {
        const double yaw = std::atan2(dy, dx);
        pose.pose.orientation.z = std::sin(0.5 * yaw);
        pose.pose.orientation.w = std::cos(0.5 * yaw);
      } else {
        pose.pose.orientation = goal.pose.orientation;
      }
    }
    path.poses.push_back(std::move(pose));
  }

  // Keep the snapped terrain node for a safe approach, then append the exact
  // commanded target so the downstream controller receives its position/yaw.
  geometry_msgs::msg::PoseStamped exact_goal = goal;
  exact_goal.header = path.header;
  exact_goal.pose.position.z += path_z_offset_;
  path.poses.push_back(std::move(exact_goal));
  return path;
}

}  // namespace global_planner
