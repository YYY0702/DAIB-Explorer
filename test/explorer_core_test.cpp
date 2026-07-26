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

  std::vector<Vec3> points;
  for (int degree = 0; degree < 360; degree += 10)
  {
    const double angle = degree * 3.14159265358979323846 / 180.0;
    points.push_back({8.0 * std::cos(angle), 8.0 * std::sin(angle), 0.0});
  }
  explorer.update({0.0, 0.0, 0.0}, 0.0, points, 1.0);

  EXPECT_GT(explorer.stats().free_cells, 0U);
  EXPECT_GT(explorer.stats().occupied_cells, 0U);
  EXPECT_GT(explorer.stats().frontier_cells, 0U);
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
  explorer.update({0.0, 0.0, 0.0}, 0.0, {{5.0, 0.0, 0.0}}, 1.0);
  EXPECT_DOUBLE_EQ(explorer.stats().budget_scale,
                   config.overload_budget_scale);
  EXPECT_EQ(explorer.stats().effective_raycasts,
            static_cast<int>(std::lround(
                config.max_raycasts_per_update *
                config.overload_budget_scale)));
}

}  // namespace daib_explorer

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
