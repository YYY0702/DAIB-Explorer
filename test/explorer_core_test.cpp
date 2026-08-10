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
  EXPECT_GT(explorer.stats().visited_cells, 0U);
  GoalDecision decision;
  ASSERT_TRUE(explorer.consumeDecision(decision));
  EXPECT_TRUE(decision.valid);
  EXPECT_EQ(decision.generation, 1U);
  EXPECT_GT(explorer.stats().candidates_scored, 0U);
  EXPECT_EQ(explorer.stats().reachability_checks, 0U);
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

TEST(ExplorerCore, MaintainsOnlyLightweightVisitMemory)
{
  ExplorerConfig config;
  config.coverage_voxel_size_m = 2.0;
  ExplorerCore explorer(config);
  const std::vector<Vec3> points{{4.0, 0.0, 0.0}, {0.0, 4.0, 1.0}};

  explorer.update({0.0, 0.0, 0.0}, {}, points, 1.0);
  EXPECT_EQ(explorer.stats().visited_cells, 1U);
  EXPECT_EQ(explorer.stats().observed_cells, 0U);
  EXPECT_EQ(explorer.stats().submap_count, 0U);

  // Moving into another coverage voxel adds one compact visit entry. Point
  // geometry and vehicle rotation no longer create a second long-term map.
  explorer.update({3.0, 0.0, 0.0}, {0.5, 0.0, 0.0, 0.866}, points, 2.0);
  EXPECT_EQ(explorer.stats().visited_cells, 2U);
  EXPECT_EQ(explorer.stats().observed_cells, 0U);
  EXPECT_EQ(explorer.stats().submap_count, 0U);
}

TEST(ExplorerCore, KeepsAcceptedGoalWhileVehicleMakesProgress)
{
  ExplorerConfig config;
  config.min_goal_distance_m = 1.0;
  config.max_goal_distance_m = 10.0;
  config.goal_min_hold_time_s = 1.0;
  config.goal_timeout_s = 0.0;
  config.goal_progress_epsilon_m = 0.25;
  config.goal_stall_timeout_s = 15.0;
  config.allow_periodic_goal_switch = false;
  ExplorerCore explorer(config);
  const std::vector<Vec3> points{{8.0, 0.0, 0.0}};

  explorer.update({0.0, 0.0, 0.0}, {}, points, 1.0);
  GoalDecision initial;
  ASSERT_TRUE(explorer.consumeDecision(initial));
  ASSERT_TRUE(initial.valid);

  GoalDecision replacement;
  for (int step = 1; step <= 5; ++step)
  {
    explorer.update({0.3 * step, 0.0, 0.0}, {}, points,
                    1.0 + 11.0 * step);
    EXPECT_FALSE(explorer.consumeDecision(replacement));
  }
  EXPECT_EQ(explorer.stats().stalled_goals, 0U);
}

TEST(ExplorerCore, StalledGoalEntersCooldownBeforeSameAreaCanReturn)
{
  ExplorerConfig config;
  config.min_goal_distance_m = 1.0;
  config.max_goal_distance_m = 4.0;
  config.goal_min_hold_time_s = 0.0;
  config.goal_timeout_s = 0.0;
  config.goal_progress_epsilon_m = 0.25;
  config.goal_stall_timeout_s = 15.0;
  config.failed_goal_exclusion_radius_m = 2.0;
  config.failed_goal_cooldown_s = 30.0;
  ExplorerCore explorer(config);
  const std::vector<Vec3> points{{8.0, 0.0, 0.0}};

  explorer.update({0.0, 0.0, 0.0}, {}, points, 1.0);
  GoalDecision initial;
  ASSERT_TRUE(explorer.consumeDecision(initial));
  ASSERT_TRUE(initial.valid);

  explorer.update({0.0, 0.0, 0.0}, {}, points, 17.0);
  GoalDecision stalled;
  ASSERT_TRUE(explorer.consumeDecision(stalled));
  EXPECT_FALSE(stalled.valid);
  EXPECT_EQ(stalled.state, "WAIT_FOR_FRONTIER");
  EXPECT_EQ(stalled.reason, "goal_stalled_no_safe_frontier");
  EXPECT_EQ(explorer.stats().stalled_goals, 1U);
  EXPECT_EQ(explorer.stats().failed_goals_in_cooldown, 1U);

  explorer.update({0.0, 0.0, 0.0}, {}, points, 30.0);
  GoalDecision cooling;
  EXPECT_FALSE(explorer.consumeDecision(cooling));

  explorer.update({0.0, 0.0, 0.0}, {}, points, 48.0);
  GoalDecision retried;
  ASSERT_TRUE(explorer.consumeDecision(retried));
  EXPECT_TRUE(retried.valid);
  EXPECT_EQ(explorer.stats().failed_goals_in_cooldown, 0U);
}

TEST(ExplorerCore, KeepsAcceptedGoalWhenPeriodicSwitchingIsDisabled)
{
  ExplorerConfig config;
  config.min_goal_distance_m = 1.0;
  config.max_goal_distance_m = 10.0;
  config.goal_min_hold_time_s = 1.0;
  config.allow_periodic_goal_switch = false;
  ExplorerCore explorer(config);

  explorer.update({0.0, 0.0, 0.0}, {}, {{8.0, 0.0, 0.0}}, 1.0);
  GoalDecision initial;
  ASSERT_TRUE(explorer.consumeDecision(initial));
  ASSERT_TRUE(initial.valid);

  explorer.update(
      {0.0, 0.0, 0.0}, {},
      {{8.0, 0.0, 0.0}, {0.0, 8.0, 0.0}}, 5.0);
  GoalDecision replacement;
  EXPECT_FALSE(explorer.consumeDecision(replacement));
}

TEST(ExplorerCore, ReachedGoalAllowsNextGeneration)
{
  ExplorerConfig config;
  config.min_goal_distance_m = 1.0;
  config.max_goal_distance_m = 10.0;
  ExplorerCore explorer(config);

  explorer.update({0.0, 0.0, 0.0}, {}, {{8.0, 0.0, 0.0}}, 1.0);
  GoalDecision first;
  ASSERT_TRUE(explorer.consumeDecision(first));
  ASSERT_TRUE(first.valid);

  explorer.update(first.position, {},
                  {{first.position.x + 8.0,
                    first.position.y,
                    first.position.z}},
                  2.0);
  GoalDecision second;
  ASSERT_TRUE(explorer.consumeDecision(second));
  ASSERT_TRUE(second.valid);
  EXPECT_EQ(second.generation, 2U);
}

TEST(ExplorerCore, AcceptsSingleVoxelClusterAndStandoffFallback)
{
  ExplorerConfig config;
  config.min_goal_distance_m = 1.0;
  config.max_goal_distance_m = 10.0;
  config.min_frontier_cluster_cells = 1;
  config.viewpoint_standoff_m = 10.0;
  config.viewpoint_search_radius_m = 0.5;
  config.min_wall_clearance_m = 0.0;
  ExplorerCore explorer(config);

  explorer.update({0.0, 0.0, 0.0}, {}, {{8.0, 0.0, 0.0}}, 1.0);
  EXPECT_GT(explorer.stats().frontier_clusters, 0U);
  EXPECT_GT(explorer.stats().safe_viewpoint_candidates, 0U);
  GoalDecision decision;
  ASSERT_TRUE(explorer.consumeDecision(decision));
  EXPECT_TRUE(decision.valid);
}

TEST(ExplorerCore, EnforcesFinalDistanceAndHeadingBounds)
{
  ExplorerConfig config;
  config.min_goal_distance_m = 1.0;
  config.max_goal_distance_m = 8.0;
  config.max_heading_change_deg = 120.0;
  ExplorerCore bounded(config);
  bounded.update({0.0, 0.0, 0.0}, {}, {{12.0, 0.0, 0.0}}, 1.0);
  GoalDecision bounded_goal;
  ASSERT_TRUE(bounded.consumeDecision(bounded_goal));
  ASSERT_TRUE(bounded_goal.valid);
  EXPECT_LE(std::sqrt(bounded_goal.position.x * bounded_goal.position.x +
                      bounded_goal.position.y * bounded_goal.position.y +
                      bounded_goal.position.z * bounded_goal.position.z),
            config.max_goal_distance_m);

  ExplorerCore reverse(config);
  reverse.update({0.0, 0.0, 0.0}, {}, {{-8.0, 0.0, 0.0}}, 1.0);
  GoalDecision heading_rejected;
  ASSERT_TRUE(reverse.consumeDecision(heading_rejected));
  EXPECT_FALSE(heading_rejected.valid);
  EXPECT_GT(reverse.stats().rejected_heading, 0U);

}

TEST(ExplorerCore, UsesSymmetricRelativeVerticalBound)
{
  ExplorerConfig config;
  config.min_goal_distance_m = 1.0;
  config.max_goal_distance_m = 10.0;
  config.max_goal_vertical_distance_m = 3.0;

  ExplorerCore above(config);
  const Vec3 high_position{0.0, 0.0, 10.0};
  above.update(high_position, {}, {{8.0, 0.0, 12.0}}, 1.0);
  GoalDecision high_goal;
  ASSERT_TRUE(above.consumeDecision(high_goal));
  ASSERT_TRUE(high_goal.valid);
  EXPECT_LE(std::fabs(high_goal.position.z - high_position.z), 3.0);

  ExplorerCore below(config);
  below.update(high_position, {}, {{8.0, 0.0, 8.0}}, 1.0);
  GoalDecision low_goal;
  ASSERT_TRUE(below.consumeDecision(low_goal));
  ASSERT_TRUE(low_goal.valid);
  EXPECT_LE(std::fabs(low_goal.position.z - high_position.z), 3.0);
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

TEST(ExplorerCore, RejectsGoalsBeyondMaximumHeadingChange)
{
  ExplorerConfig config;
  config.min_goal_distance_m = 1.0;
  config.max_goal_distance_m = 10.0;

  ExplorerCore facing_positive_x(config);
  facing_positive_x.update(
      {0.0, 0.0, 0.0}, {}, {{-8.0, 0.0, 0.0}}, 1.0);
  GoalDecision rejected;
  ASSERT_TRUE(facing_positive_x.consumeDecision(rejected));
  EXPECT_FALSE(rejected.valid);

  ExplorerCore facing_negative_x(config);
  facing_negative_x.update(
      {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0},
      {{-8.0, 0.0, 0.0}}, 1.0);
  GoalDecision accepted;
  ASSERT_TRUE(facing_negative_x.consumeDecision(accepted));
  ASSERT_TRUE(accepted.valid);
  EXPECT_LT(accepted.position.x, 0.0);
  EXPECT_GT(facing_negative_x.stats().frontier_clusters, 0U);
  EXPECT_GT(facing_negative_x.stats().safe_viewpoint_candidates, 0U);
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
  memory.applyDelta({first});
  memory.applyDelta({second});
  EXPECT_EQ(memory.stats().record_count, 1U);
  EXPECT_EQ(memory.stats().root_count, 2U);
  EXPECT_EQ(memory.stats().detailed_root_count, 1U);
  EXPECT_EQ(memory.stats().capacity_evictions, 1U);

  const std::vector<PvbsmExplorationHint> hints =
      memory.queryExplorationHints(
          {{0.2, 0.2, 0.2}, {1.2, 0.2, 0.2}},
          0, 1.0, 8, 2);
  ASSERT_EQ(hints.size(), 2U);
  EXPECT_TRUE(hints[0].root_observed);
  EXPECT_TRUE(hints[1].root_observed);
}

TEST(PvbsmMemory, HardDeletionClearsDemotedCoverage)
{
  PvbsmMemory memory(1);
  PvbsmRecord first;
  first.revision = 1;
  first.kind = 0;
  PvbsmRecord second = first;
  second.root[0] = 1;
  second.revision = 2;
  memory.applyDelta({first});
  memory.applyDelta({second});
  ASSERT_EQ(memory.stats().root_count, 2U);

  PvbsmRecord deletion = first;
  deletion.revision = 3;
  deletion.kind = 2;
  memory.applyDelta({deletion});
  EXPECT_EQ(memory.stats().root_count, 1U);
  const std::vector<PvbsmExplorationHint> hint =
      memory.queryExplorationHints({{0.2, 0.2, 0.2}}, 0, 1.0, 8, 1);
  ASSERT_EQ(hint.size(), 1U);
  EXPECT_FALSE(hint.front().root_observed);
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

TEST(PvbsmMemory, QueriesRootCoverageAndUnseenSubmaps)
{
  PvbsmMemory memory(10);
  PvbsmRecord observed;
  observed.source_id = 5;
  observed.root[0] = 0;
  observed.revision = 1;
  observed.kind = 0;
  observed.confidence = 1.0F;
  observed.submap_edge_roots = 8;
  memory.applyDelta({observed});

  const std::vector<PvbsmExplorationHint> hints =
      memory.queryExplorationHints(
          {{0.2, 0.2, 0.2}, {8.2, 0.2, 0.2}},
          5, 1.0, 8, 1);
  ASSERT_EQ(hints.size(), 2U);
  EXPECT_TRUE(hints[0].root_observed);
  EXPECT_TRUE(hints[0].submap_observed);
  EXPECT_DOUBLE_EQ(hints[0].submap_coverage, 1.0);
  EXPECT_DOUBLE_EQ(hints[0].structural_support, 1.0);
  EXPECT_FALSE(hints[1].root_observed);
  EXPECT_FALSE(hints[1].submap_observed);
}

TEST(PvbsmMemory, ReplacesIncrementalSubmapEvidence)
{
  PvbsmMemory memory(10);
  PvbsmRecord plane;
  plane.revision = 1;
  plane.kind = 0;
  plane.confidence = 0.8F;
  memory.applyDelta({plane});
  ASSERT_EQ(memory.stats().plane_count, 1U);

  PvbsmRecord residual = plane;
  residual.revision = 2;
  residual.kind = 1;
  memory.applyDelta({residual});
  EXPECT_EQ(memory.stats().root_count, 1U);
  EXPECT_EQ(memory.stats().record_count, 1U);
  EXPECT_EQ(memory.stats().plane_count, 0U);
  EXPECT_EQ(memory.stats().residual_count, 1U);
  EXPECT_EQ(memory.stats().submap_count, 1U);
}

TEST(ExplorerCore, PrefersPvbsmUnseenFrontier)
{
  ExplorerConfig config;
  config.min_goal_distance_m = 1.0;
  config.max_goal_distance_m = 10.0;
  config.pvbsm_unseen_submap_bonus = 10.0;
  config.pvbsm_submap_coverage_penalty = 10.0;
  ExplorerCore explorer(config);
  explorer.setHealth(false, 0.2, 10.0);
  explorer.setPvbsmBatchQuery(
      [](const std::vector<PvbsmQueryPoint> &points)
      {
        std::vector<PvbsmExplorationHint> hints;
        hints.reserve(points.size());
        for (const PvbsmQueryPoint &point : points)
        {
          PvbsmExplorationHint hint;
          if (point.x >= 0.0)
          {
            hint.submap_observed = true;
            hint.submap_coverage = 1.0;
          }
          hints.push_back(hint);
        }
        return hints;
      });

  explorer.update(
      {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0},
      {{8.0, 0.0, 0.0}, {-8.0, 0.0, 0.0}}, 1.0);
  GoalDecision decision;
  ASSERT_TRUE(explorer.consumeDecision(decision));
  ASSERT_TRUE(decision.valid);
  EXPECT_LT(decision.position.x, 0.0);
  EXPECT_GT(explorer.stats().pvbsm_scored_candidates, 0U);
  EXPECT_GT(explorer.stats().pvbsm_unseen_candidates, 0U);
  EXPECT_GT(explorer.stats().pvbsm_best_adjustment, 0.0);
}

}  // namespace daib_explorer

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
