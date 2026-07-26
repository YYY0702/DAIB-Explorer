#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace daib_explorer
{

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quaternion
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct VoxelKey
{
  int64_t x = 0;
  int64_t y = 0;
  int64_t z = 0;
  bool operator==(const VoxelKey &other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey &key) const;
};

struct ExplorerConfig
{
  int robot_id = 0;
  double planning_voxel_size_m = 0.5;
  double planning_sensor_range_m = 20.0;
  double planning_map_radius_m = 40.0;
  int planning_prune_budget = 256;
  int max_raycasts_per_update = 64;
  int max_ray_steps = 64;
  int frontier_update_budget = 512;
  int frontier_evaluation_budget = 1200;

  double coverage_voxel_size_m = 2.0;
  int max_coverage_points_per_update = 256;
  double submap_translation_threshold_m = 20.0;
  double submap_rotation_threshold_deg = 45.0;

  double replan_interval_s = 1.0;
  double goal_min_hold_time_s = 3.0;
  int goal_blocked_confirm_updates = 3;
  double same_goal_tolerance_m = 0.5;
  double goal_timeout_s = 12.0;
  double goal_reached_distance_m = 1.0;
  double min_goal_distance_m = 2.0;
  double max_goal_distance_m = 15.0;
  double max_goal_vertical_distance_m = 3.0;
  double goal_switch_margin = 0.15;

  bool dynamic_budget_enabled = true;
  double lio_busy_threshold_ms = 90.0;
  double lio_overload_threshold_ms = 110.0;
  double lio_time_ema_alpha = 0.20;
  double busy_budget_scale = 0.50;
  double overload_budget_scale = 0.25;

  double degenerate_max_goal_distance_m = 8.0;
  double degenerate_goal_switch_margin = 0.30;
  double degenerate_safe_path_weight = 4.0;
};

struct GoalDecision
{
  bool valid = false;
  bool updated = false;
  uint64_t generation = 0;
  Vec3 position;
  double yaw = 0.0;
  double score = 0.0;
  double planning_time_ms = 0.0;
  std::string state = "WAIT_FOR_MAP";
  std::string reason = "not_initialized";
};

struct SubmapSummary
{
  uint64_t id = 0;
  uint64_t version = 1;
  uint64_t start_update = 0;
  uint64_t end_update = 0;
  Vec3 anchor;
  Quaternion anchor_orientation;
  Vec3 min_bound;
  Vec3 max_bound;
  std::size_t covered_cells = 0;
};

struct ExplorerStats
{
  std::size_t free_cells = 0;
  std::size_t occupied_cells = 0;
  std::size_t frontier_cells = 0;
  std::size_t visited_cells = 0;
  std::size_t observed_cells = 0;
  std::size_t submap_count = 0;
  double smoothed_lio_time_ms = 0.0;
  double budget_scale = 1.0;
  double last_plan_ms = 0.0;
  double last_update_ms = 0.0;
  int effective_raycasts = 0;
  int effective_frontier_updates = 0;
  int effective_frontier_evaluations = 0;
  uint64_t suppressed_goal_republishes = 0;
};

class ExplorerCore
{
public:
  explicit ExplorerCore(ExplorerConfig config);

  void setHealth(bool degenerate, double degeneracy_score, double lio_runtime_ms);
  void update(const Vec3 &position, const Quaternion &orientation,
              const std::vector<Vec3> &points, double timestamp);
  bool consumeDecision(GoalDecision &decision);
  std::vector<Vec3> frontierPoints(std::size_t limit) const;
  std::vector<Vec3> occupiedPoints(std::size_t limit) const;

  const ExplorerStats &stats() const { return stats_; }
  const std::vector<SubmapSummary> &submaps() const { return submaps_; }

private:
  struct Cell
  {
    int16_t log_odds = 0;
    uint64_t last_update = 0;
  };

  ExplorerConfig config_;
  ExplorerStats stats_;
  std::unordered_map<VoxelKey, Cell, VoxelKeyHash> map_;
  std::deque<VoxelKey> map_queue_;
  std::unordered_set<VoxelKey, VoxelKeyHash> dirty_frontiers_;
  std::unordered_set<VoxelKey, VoxelKeyHash> frontiers_;
  std::unordered_map<VoxelKey, uint32_t, VoxelKeyHash> visits_;
  std::unordered_map<VoxelKey, uint32_t, VoxelKeyHash> observations_;
  std::unordered_set<VoxelKey, VoxelKeyHash> active_submap_cells_;
  std::vector<SubmapSummary> submaps_;

  GoalDecision decision_;
  uint64_t update_id_ = 0;
  double last_plan_time_ = -1.0;
  double goal_set_time_ = -1.0;
  int blocked_streak_ = 0;
  bool degenerate_ = true;
  double degeneracy_score_ = 0.0;
  double smoothed_lio_ms_ = -1.0;

  static double distance(const Vec3 &a, const Vec3 &b);
  VoxelKey key(const Vec3 &point, double voxel_size) const;
  Vec3 center(const VoxelKey &key, double voxel_size) const;
  int cellState(const VoxelKey &key) const;
  void markFrontierDirty(const VoxelKey &key);
  void updateCell(const VoxelKey &key, int delta);
  void integrateCloud(const Vec3 &origin, const std::vector<Vec3> &points);
  void prune(const Vec3 &position);
  void updateFrontiers();
  bool segmentBlocked(const Vec3 &start, const Vec3 &end,
                      double *known_free_ratio = nullptr) const;
  double frontierScore(const VoxelKey &key, const Vec3 &position) const;
  void updateSubmap(const Vec3 &position, const Quaternion &orientation,
                    const std::vector<Vec3> &points);
  void updateDecision(const Vec3 &position, double timestamp);
  void sanitizeConfig();
};

}  // namespace daib_explorer
