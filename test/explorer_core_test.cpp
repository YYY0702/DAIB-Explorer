#include "daib_explorer/explorer_core.h"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

namespace daib_explorer
{

TEST(ExplorerCore, BuildsFrontiersAndSelectsGoal)
{
  ExplorerConfig config;
  config.min_goal_distance_m = 1.0;
  config.max_goal_distance_m = 10.0;
  config.max_raycasts_per_update = 64;
  ExplorerCore explorer(config);
  explorer.setHealth(false, 0.2, 50.0);

  const std::vector<Vec3> points{{8.0, 0.0, 0.0}};
  explorer.update({0.0, 0.0, 0.0}, {}, points, 1.0);

  EXPECT_GT(explorer.stats().free_cells, 0U);
  EXPECT_GT(explorer.stats().occupied_cells, 0U);
  EXPECT_GT(explorer.stats().frontier_cells, 0U);
  EXPECT_GT(explorer.stats().observed_cells, 0U);
  GoalDecision decision;
  ASSERT_TRUE(explorer.consumeDecision(decision));
  EXPECT_TRUE(decision.valid);
  EXPECT_EQ(decision.generation, 1U);
}

TEST(ExplorerCore, ScalesBudgetFromLioRuntime)
{
  ExplorerConfig config;
  ExplorerCore explorer(config);
  explorer.setHealth(false, 0.2, 120.0);
  explorer.update({0.0, 0.0, 0.0}, {}, {{5.0, 0.0, 0.0}}, 1.0);
  EXPECT_DOUBLE_EQ(explorer.stats().budget_scale,
                   config.overload_budget_scale);
  EXPECT_EQ(explorer.stats().effective_raycasts,
            static_cast<int>(std::lround(
                config.max_raycasts_per_update *
                config.overload_budget_scale)));
}

TEST(ExplorerCore, PreservesObservedCoverageAndFullRotationSubmaps)
{
  ExplorerConfig config;
  config.submap_translation_threshold_m = 1000.0;
  config.submap_rotation_threshold_deg = 45.0;
  ExplorerCore explorer(config);
  const std::vector<Vec3> points{{4.0, 0.0, 0.0}, {0.0, 4.0, 1.0}};

  explorer.update({0.0, 0.0, 0.0}, {}, points, 1.0);
  EXPECT_GT(explorer.stats().observed_cells, 0U);
  EXPECT_EQ(explorer.submaps().size(), 1U);
  EXPECT_GT(explorer.submaps().front().covered_cells, 1U);

  // Sixty degrees around the X axis must create a submap even with unchanged
  // yaw and position. This protects the old full-3D rotation behavior.
  const double half_angle = 30.0 * 3.14159265358979323846 / 180.0;
  const Quaternion rolled{std::sin(half_angle), 0.0, 0.0,
                          std::cos(half_angle)};
  explorer.update({0.0, 0.0, 0.0}, rolled, points, 2.0);
  EXPECT_EQ(explorer.submaps().size(), 2U);
}

TEST(ExplorerCore, HoldsGoalAndSuppressesSameTimedOutGoal)
{
  ExplorerConfig config;
  config.min_goal_distance_m = 1.0;
  config.max_goal_distance_m = 10.0;
  config.goal_min_hold_time_s = 3.0;
  config.goal_timeout_s = 5.0;
  ExplorerCore explorer(config);
  const std::vector<Vec3> points{{8.0, 0.0, 0.0}};

  explorer.update({0.0, 0.0, 0.0}, {}, points, 1.0);
  GoalDecision initial;
  ASSERT_TRUE(explorer.consumeDecision(initial));
  ASSERT_TRUE(initial.valid);

  explorer.update({0.0, 0.0, 0.0}, {}, points, 2.0);
  GoalDecision held;
  EXPECT_FALSE(explorer.consumeDecision(held));

  explorer.update({0.0, 0.0, 0.0}, {}, {}, 7.0);
  GoalDecision republished;
  EXPECT_FALSE(explorer.consumeDecision(republished));
  EXPECT_EQ(explorer.stats().suppressed_goal_republishes, 1U);
}

TEST(ExplorerCore, RequiresConsecutiveObstacleConfirmation)
{
  ExplorerConfig config;
  config.min_goal_distance_m = 1.0;
  config.max_goal_distance_m = 10.0;
  config.goal_min_hold_time_s = 10.0;
  config.goal_blocked_confirm_updates = 3;
  ExplorerCore explorer(config);
  std::vector<Vec3> ring;
  for (int degree = 0; degree < 360; degree += 10)
  {
    const double angle = degree * 3.14159265358979323846 / 180.0;
    ring.push_back({8.0 * std::cos(angle), 8.0 * std::sin(angle), 0.0});
  }
  explorer.update({0.0, 0.0, 0.0}, {}, ring, 1.0);
  GoalDecision initial;
  ASSERT_TRUE(explorer.consumeDecision(initial));
  ASSERT_TRUE(initial.valid);

  const Vec3 obstacle = initial.position;
  explorer.update({0.0, 0.0, 0.0}, {}, {obstacle, obstacle}, 2.0);
  GoalDecision decision;
  EXPECT_FALSE(explorer.consumeDecision(decision));
  explorer.update({0.0, 0.0, 0.0}, {}, {obstacle, obstacle}, 3.0);
  EXPECT_FALSE(explorer.consumeDecision(decision));
  explorer.update({0.0, 0.0, 0.0}, {}, {obstacle, obstacle}, 4.0);
  EXPECT_TRUE(explorer.consumeDecision(decision));
}

TEST(ExplorerCore, RunsIndependentMultiRateSchedule)
{
  ExplorerConfig config;
  config.frontier_update_rate_hz = 2.0;
  config.goal_evaluation_rate_hz = 1.0;
  config.long_term_update_rate_hz = 1.0;
  ExplorerCore explorer(config);
  const std::vector<Vec3> points{{8.0, 0.0, 0.0}};

  for (int update = 0; update < 10; ++update)
  {
    explorer.update({0.0, 0.0, 0.0}, {}, points,
                    1.0 + 0.1 * update);
  }

  EXPECT_EQ(explorer.stats().map_updates, 10U);
  EXPECT_EQ(explorer.stats().goal_status_checks, 10U);
  EXPECT_EQ(explorer.stats().frontier_update_cycles, 2U);
  EXPECT_EQ(explorer.stats().goal_evaluation_cycles, 1U);
  EXPECT_EQ(explorer.stats().long_term_update_cycles, 1U);
}

TEST(ExplorerCore, BoundsPlannerCloudAroundCurrentPosition)
{
  ExplorerConfig config;
  ExplorerCore explorer(config);
  explorer.update(
      {0.0, 0.0, 0.0}, {},
      {{4.0, 0.0, 0.0}, {10.0, 0.0, 0.0}}, 1.0);

  const std::vector<Vec3> local =
      explorer.occupiedPoints({0.0, 0.0, 0.0}, 6.0, 100);
  ASSERT_FALSE(local.empty());
  for (const Vec3 &point : local)
  {
    const double range =
        std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    EXPECT_LE(range, 6.0);
  }
}

}  // namespace daib_explorer

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
