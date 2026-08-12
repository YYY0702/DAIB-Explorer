#pragma once

#include "daib_explorer/pvbsm_types.h"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
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
  double frontier_update_rate_hz = 2.0;
  double goal_evaluation_rate_hz = 1.0;
  double long_term_update_rate_hz = 1.0;

  double coverage_voxel_size_m = 2.0;

  double replan_interval_s = 1.0;
  double goal_min_hold_time_s = 5.0;
  int goal_blocked_confirm_updates = 10;
  // Deprecated compatibility timeout. Zero disables absolute goal expiry;
  // progress-based stalling is the normal replacement policy.
  double goal_timeout_s = 0.0;
  double goal_reached_distance_m = 0.5;
  double goal_progress_epsilon_m = 0.25;
  double goal_stall_timeout_s = 15.0;
  double failed_goal_exclusion_radius_m = 1.0;
  double failed_goal_cooldown_s = 30.0;
  bool allow_periodic_goal_switch = false;
  double min_goal_distance_m = 1.5;
  double max_goal_distance_m = 8.0;
  double max_goal_vertical_distance_m = 3.0;
  double min_known_free_path_ratio = 0.5;
  double goal_switch_margin = 0.15;

  // DAIB-MCSVF: cluster connected frontier voxels, then place one safe
  // viewpoint in known free space for each cluster before scoring.
  // frontier_cluster_size_m is retained for configuration compatibility; the
  // current cluster definition is based on planning-voxel connectivity.
  double frontier_cluster_size_m = 2.0;
  int min_frontier_cluster_cells = 10;
  double viewpoint_standoff_m = 1.0;
  double viewpoint_search_radius_m = 2.0;
  double min_wall_clearance_m = 0.5;
  int max_safe_viewpoint_candidates = 64;
  double preferred_heading_change_deg = 60.0;
  double max_heading_change_deg = 120.0;
  double distance_cost_weight = 0.5;
  double heading_cost_weight = 3.0;

  // Bounded reachability prevents straight-line visibility from rejecting a
  // valid goal around a corner. Direct free lines do not invoke A*.
  bool reachability_enabled = true;
  int reachability_max_expansions = 2500;
  double goal_reachability_check_rate_hz = 2.0;

  bool dynamic_budget_enabled = true;
  double lio_busy_threshold_ms = 25.0;
  double lio_overload_threshold_ms = 35.0;
  double lio_time_ema_alpha = 0.20;
  double busy_budget_scale = 0.50;
  double overload_budget_scale = 0.25;

  double degenerate_max_goal_distance_m = 8.0;
  double degenerate_goal_switch_margin = 0.30;
  double degenerate_safe_path_weight = 4.0;

  bool pvbsm_scoring_enabled = true;
  double pvbsm_root_voxel_size_m = 1.0;
  int pvbsm_submap_edge_roots = 8;
  int pvbsm_covered_root_target = 64;
  double pvbsm_unseen_submap_bonus = 1.0;
  double pvbsm_submap_coverage_penalty = 2.0;
  double pvbsm_observed_root_penalty = 1.5;
  double pvbsm_degenerate_structure_bonus = 0.75;
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
  int constraint_tier = -1;
  double heading_change_deg = 0.0;
  std::string state = "WAIT_FOR_MAP";
  std::string reason = "not_initialized";
};

struct ExplorerStats
{
  std::size_t free_cells = 0;
  std::size_t occupied_cells = 0;
  std::size_t frontier_cells = 0;
  std::size_t visited_cells = 0;
  // Deprecated compatibility slots. ExplorerNode exposes the corresponding
  // observed/submaps log values from PvbsmMemory instead of this core.
  std::size_t observed_cells = 0;
  std::size_t submap_count = 0;
  double smoothed_lio_time_ms = 0.0;
  double budget_scale = 1.0;
  double last_plan_ms = 0.0;
  double last_update_ms = 0.0;
  int effective_raycasts = 0;
  int effective_frontier_updates = 0;
  int effective_frontier_evaluations = 0;
  uint64_t map_updates = 0;
  uint64_t goal_status_checks = 0;
  uint64_t frontier_update_cycles = 0;
  uint64_t goal_evaluation_cycles = 0;
  uint64_t long_term_update_cycles = 0;
  std::size_t pvbsm_scored_candidates = 0;
  std::size_t pvbsm_unseen_candidates = 0;
  double pvbsm_best_adjustment = 0.0;
  std::size_t valid_frontier_cells = 0;
  std::size_t rejected_stale_frontiers = 0;
  std::size_t frontier_components = 0;
  std::size_t rejected_small_clusters = 0;
  std::size_t frontier_clusters = 0;
  double last_cluster_ms = 0.0;
  std::size_t safe_viewpoint_candidates = 0;
  std::size_t rejected_no_viewpoint = 0;
  std::size_t rejected_distance = 0;
  std::size_t rejected_vertical_distance = 0;
  std::size_t rejected_heading = 0;
  std::size_t rejected_known_free_path = 0;
  std::size_t rejected_failed_goal = 0;
  std::size_t candidates_scored = 0;
  uint64_t reachability_checks = 0;
  uint64_t reachability_budget_exhaustions = 0;
  uint64_t stalled_goals = 0;
  std::size_t failed_goals_in_cooldown = 0;
};

class ExplorerCore
{
public:
  explicit ExplorerCore(ExplorerConfig config);

  using PvbsmBatchQuery = std::function<std::vector<PvbsmExplorationHint>(
      const std::vector<PvbsmQueryPoint> &)>;

  void setHealth(bool degenerate, double degeneracy_score, double lio_runtime_ms);
  void setPvbsmBatchQuery(PvbsmBatchQuery query);
  void update(const Vec3 &position, const Quaternion &orientation,
              const std::vector<Vec3> &points, double timestamp);
  bool consumeDecision(GoalDecision &decision);
  std::vector<Vec3> frontierPoints(std::size_t limit) const;
  std::vector<Vec3> selectedFrontierPoints() const;
  uint64_t selectedClusterGeneration() const
  {
    return selected_cluster_generation_;
  }
  std::vector<Vec3> occupiedPoints(const Vec3 &position, double radius,
                                   std::size_t limit) const;

  const ExplorerStats &stats() const { return stats_; }
private:
  friend class ExplorerCoreTestPeer;

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
  std::vector<VoxelKey> selected_frontier_cluster_;
  uint64_t selected_cluster_generation_ = 0;

  GoalDecision decision_;
  uint64_t update_id_ = 0;
  double last_plan_time_ = -1.0;
  double goal_set_time_ = -1.0;
  int blocked_streak_ = 0;
  bool degenerate_ = true;
  double degeneracy_score_ = 0.0;
  double smoothed_lio_ms_ = -1.0;
  double last_frontier_update_time_ = -1.0;
  double last_goal_evaluation_time_ = -1.0;
  double last_long_term_update_time_ = -1.0;
  bool goal_reached_ = false;
  bool goal_blocked_ = false;
  bool goal_stalled_ = false;
  bool goal_timeout_ = false;
  Quaternion current_orientation_;
  double last_goal_reachability_check_time_ = -1.0;
  bool cached_goal_reachable_ = true;
  double best_goal_distance_m_ = std::numeric_limits<double>::infinity();
  double last_goal_progress_time_ = -1.0;
  uint64_t last_failed_generation_ = 0;
  struct FailedGoal
  {
    Vec3 position;
    double expires_at = 0.0;
  };
  std::deque<FailedGoal> failed_goals_;
  PvbsmBatchQuery pvbsm_batch_query_;

  static double distance(const Vec3 &a, const Vec3 &b);
  VoxelKey key(const Vec3 &point, double voxel_size) const;
  Vec3 center(const VoxelKey &key, double voxel_size) const;
  int cellState(const VoxelKey &key) const;
  bool isFrontierVoxel(const VoxelKey &key) const;
  void markFrontierDirty(const VoxelKey &key);
  void updateCell(const VoxelKey &key, int delta);
  void integrateCloud(const Vec3 &origin, const std::vector<Vec3> &points);
  void prune(const Vec3 &position);
  void updateFrontiers();
  bool segmentBlocked(const Vec3 &start, const Vec3 &end,
                      double *known_free_ratio = nullptr) const;
  bool pathReachable(const Vec3 &start, const Vec3 &end,
                     int max_expansions, bool *budget_exhausted = nullptr) const;
  bool hasWallClearance(const VoxelKey &voxel) const;
  bool makeSafeViewpoint(const std::vector<VoxelKey> &cluster,
                         Vec3 &viewpoint, VoxelKey &representative) const;
  std::vector<std::vector<VoxelKey>> frontierClusters();
  double currentYaw() const;
  double headingChange(const Vec3 &position, const Vec3 &goal) const;
  bool nearFailedGoal(const Vec3 &point) const;
  void pruneFailedGoals(double timestamp);
  void recordFailedGoal(double timestamp);
  void resetGoalProgress(const Vec3 &position, double timestamp);
  double frontierScore(const VoxelKey &key) const;
  double pvbsmScoreAdjustment(const PvbsmExplorationHint &hint) const;
  void updateVisitMemory(const Vec3 &position);
  void updateGoalStatus(const Vec3 &position, double timestamp);
  void updateDecision(const Vec3 &position, double timestamp);
  bool isDue(double timestamp, double rate_hz, double &last_time);
  void sanitizeConfig();
};

}  // namespace daib_explorer
