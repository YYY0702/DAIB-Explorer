#include "daib_explorer/explorer_core.h"
#include "daib_explorer/pvbsm_memory.h"

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

TEST(ExplorerCore, EntersBusyBudgetAtValidatedBoardRuntime)
{
  ExplorerConfig config;
  ExplorerCore explorer(config);
  explorer.setHealth(false, 0.2, 30.0);
  explorer.update({0.0, 0.0, 0.0}, {}, {{5.0, 0.0, 0.0}}, 1.0);
  EXPECT_DOUBLE_EQ(explorer.stats().budget_scale,
                   config.busy_budget_scale);
  EXPECT_EQ(explorer.stats().effective_raycasts,
            static_cast<int>(std::lround(
                config.max_raycasts_per_update *
                config.busy_budget_scale)));
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
      {{4.0, 0.0, 0.0}, {0.0, 10.0, 0.0}}, 1.0);

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

TEST(ExplorerCore, ClearsStaleOccupancyAtCurrentVehicleVoxel)
{
  ExplorerConfig config;
  config.planning_voxel_size_m = 0.5;
  config.max_raycasts_per_update = 64;
  ExplorerCore explorer(config);
  const Vec3 future_position{2.25, 0.25, 0.25};

  // Accumulate a strong endpoint hit in the voxel that the vehicle will
  // occupy later.
  for (int update = 0; update < 3; ++update)
  {
    explorer.update({0.25, 0.25, 0.25}, {}, {future_position},
                    1.0 + update);
  }
  ASSERT_EQ(explorer.stats().occupied_cells, 1U);

  // One vehicle observation must immediately override that stale hit. The old
  // one-step miss update needed several cycles and leaked the voxel to EGO.
  explorer.update(future_position, {}, {}, 4.0);
  const std::vector<Vec3> occupied =
      explorer.occupiedPoints(future_position, 1.0, 100);
  EXPECT_TRUE(occupied.empty());
  EXPECT_EQ(explorer.stats().occupied_cells, 0U);
}

TEST(PvbsmMemory, AppliesVersionsDeletionAndNegativeSubmaps)
{
  PvbsmMemory memory(100);
  PvbsmRecord positive;
  positive.source_id = 3;
  positive.root[0] = -1;
  positive.revision = 1;
  positive.kind = 0;
  positive.submap_edge_roots = 8;
  memory.applyDelta({positive});
  EXPECT_EQ(memory.stats().root_count, 1U);
  EXPECT_EQ(memory.stats().plane_count, 1U);
  EXPECT_EQ(memory.stats().submap_count, 1U);

  // Same and older revisions are idempotently rejected per root.
  memory.applyDelta({positive});
  EXPECT_EQ(memory.stats().rejected_stale_root_updates, 1U);
  EXPECT_EQ(memory.stats().record_count, 1U);

  PvbsmRecord zero_side = positive;
  zero_side.root[0] = 0;
  memory.applyDelta({zero_side});
  // floor(-1 / 8)=-1, not zero: the roots belong to different submaps.
  EXPECT_EQ(memory.stats().submap_count, 2U);

  PvbsmRecord deletion = positive;
  deletion.revision = 2;
  deletion.kind = 2;
  memory.applyDelta({deletion});
  EXPECT_EQ(memory.stats().root_count, 1U);
  EXPECT_EQ(memory.stats().deleted_roots, 1U);
}

TEST(PvbsmMemory, EvictsOldestRootsAtRecordCapacity)
{
  PvbsmMemory memory(1);
  PvbsmRecord first;
  first.revision = 1;
  first.kind = 1;
  PvbsmRecord second = first;
  second.root[0] = 1;
  second.revision = 2;
  memory.applyDelta({first, second});
  EXPECT_EQ(memory.stats().record_count, 1U);
  EXPECT_EQ(memory.stats().root_count, 1U);
  EXPECT_EQ(memory.stats().capacity_evictions, 1U);
}

TEST(PvbsmMemory, AcceptsRevisionResetFromNewSenderSession)
{
  PvbsmMemory memory(10);
  PvbsmRecord old_session;
  old_session.source_id = 2;
  old_session.revision = 20;
  old_session.kind = 0;
  memory.applyDelta({old_session});

  PvbsmRecord new_session = old_session;
  new_session.revision = 1;
  new_session.kind = 1;
  new_session.flags = 2U;
  memory.applyDelta({new_session});
  EXPECT_EQ(memory.stats().source_session_resets, 1U);
  EXPECT_EQ(memory.stats().record_count, 1U);
  EXPECT_EQ(memory.stats().plane_count, 0U);
  EXPECT_EQ(memory.stats().residual_count, 1U);
}

}  // namespace daib_explorer

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
