// Copyright 2026 jiewang

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "global_planner/terrain_map.hpp"
#include "global_planner/weighted_astar.hpp"

namespace gp = global_planner;

TEST(TerrainMap, FiltersRiskAndKeepsSeparateLayers)
{
  gp::TerrainMapConfig config;
  config.resolution_xy = 0.2;
  config.resolution_z = 0.1;
  config.layer_merge_height = 0.11;
  config.min_traversable_fraction = 0.6;
  gp::TerrainMap map(config);

  const std::vector<gp::TerrainPoint> points{
    {0.01F, 0.01F, 0.01F, 255.0F},
    {0.01F, 0.01F, 1.01F, 255.0F},
    {0.41F, 0.01F, 0.01F, 0.0F}};
  std::string error;
  ASSERT_TRUE(map.buildFromPoints(points, &error)) << error;
  EXPECT_EQ(map.nodes().size(), 2U);
  const gp::NodeRange * layers = map.findColumn(0, 0);
  ASSERT_NE(layers, nullptr);
  EXPECT_EQ(layers->end - layers->begin, 2U);
  EXPECT_EQ(map.findColumn(2, 0), nullptr);
}

TEST(WeightedAStar, PlansAcrossTraversableSteps)
{
  gp::TerrainMapConfig map_config;
  map_config.resolution_xy = 0.2;
  map_config.resolution_z = 0.05;
  map_config.layer_merge_height = 0.01;
  gp::TerrainMap map(map_config);
  std::vector<gp::TerrainPoint> points;
  for (int i = 0; i < 8; ++i) {
    points.push_back(
      {0.01F + 0.2F * i, 0.01F, 0.08F * static_cast<float>(i), 255.0F});
  }
  std::string error;
  ASSERT_TRUE(map.buildFromPoints(points, &error)) << error;

  gp::PlannerConfig planner_config;
  planner_config.max_step_height = 0.10;
  planner_config.max_slope_degrees = 45.0;
  planner_config.planning_timeout = 1.0;
  gp::WeightedAStar planner(map, planner_config);
  const auto result = planner.plan(0U, static_cast<uint32_t>(map.nodes().size() - 1U));
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.raw_node_count, 8U);
  EXPECT_EQ(result.node_ids.size(), 2U);
}

TEST(WeightedAStar, RejectsStepAboveRobotLimit)
{
  gp::TerrainMapConfig map_config;
  map_config.resolution_xy = 0.2;
  map_config.resolution_z = 0.1;
  map_config.layer_merge_height = 0.01;
  gp::TerrainMap map(map_config);
  const std::vector<gp::TerrainPoint> points{
    {0.01F, 0.01F, 0.01F, 255.0F},
    {0.21F, 0.01F, 0.41F, 255.0F}};
  std::string error;
  ASSERT_TRUE(map.buildFromPoints(points, &error)) << error;

  gp::PlannerConfig planner_config;
  planner_config.max_step_height = 0.28;
  planner_config.max_slope_degrees = 89.0;
  gp::WeightedAStar planner(map, planner_config);
  const auto result = planner.plan(0U, 1U);
  EXPECT_FALSE(result.success);
}

TEST(TerrainMap, InflationIsLimitedToNearbyHeightLayer)
{
  gp::TerrainMapConfig config;
  config.resolution_xy = 0.2;
  config.resolution_z = 0.1;
  config.layer_merge_height = 0.01;
  config.robot_radius = 0.25;
  config.inflation_vertical_tolerance = 0.20;
  gp::TerrainMap map(config);
  const std::vector<gp::TerrainPoint> points{
    {0.01F, 0.01F, 0.01F, 255.0F},
    {0.21F, 0.01F, 0.01F, 0.0F},
    {0.01F, 0.01F, 1.01F, 255.0F},
    {1.01F, 0.01F, 0.01F, 255.0F}};
  std::string error;
  ASSERT_TRUE(map.buildFromPoints(points, &error)) << error;
  EXPECT_EQ(map.inflatedNodeCount(), 1U);
  EXPECT_EQ(map.nodes().size(), 2U);
  EXPECT_LT(map.nearestNode(0.01, 0.01, 0.01, 0.05, 0.05), 0);
  EXPECT_GE(map.nearestNode(0.01, 0.01, 1.01, 0.05, 0.05), 0);
}

TEST(WeightedAStar, SmoothingRemovesGridZigzagsButDoesNotCrossHazard)
{
  gp::TerrainMapConfig map_config;
  map_config.resolution_xy = 0.2;
  map_config.resolution_z = 0.1;
  map_config.layer_merge_height = 0.01;
  map_config.robot_radius = 0.0;
  gp::TerrainMap map(map_config);
  std::vector<gp::TerrainPoint> points;
  for (int x = 0; x < 7; ++x) {
    for (int y = 0; y < 5; ++y) {
      const bool blocked = x == 3 && y == 2;
      points.push_back(
        {0.01F + 0.2F * x, 0.01F + 0.2F * y, 0.01F, blocked ? 0.0F : 255.0F});
    }
  }
  std::string error;
  ASSERT_TRUE(map.buildFromPoints(points, &error)) << error;
  const int32_t start = map.nearestNode(0.01, 0.41, 0.01, 0.05, 0.05);
  const int32_t goal = map.nearestNode(1.21, 0.41, 0.01, 0.05, 0.05);
  ASSERT_GE(start, 0);
  ASSERT_GE(goal, 0);

  gp::PlannerConfig planner_config;
  planner_config.smoothing_sample_step = 0.10;
  planner_config.smoothing_xy_tolerance = 0.11;
  planner_config.smoothing_height_tolerance = 0.05;
  gp::WeightedAStar planner(map, planner_config);
  const auto result = planner.plan(static_cast<uint32_t>(start), static_cast<uint32_t>(goal));
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_LT(result.node_ids.size(), result.raw_node_count);
  EXPECT_GT(result.node_ids.size(), 2U);
}
