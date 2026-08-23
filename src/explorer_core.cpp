#include "daib_explorer/explorer_core.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace daib_explorer
{
namespace
{
// Face adjacency is used for frontier semantics and local surface direction.
constexpr int kFaceNeighbors[6][3] = {
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
    {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

// Cluster connectivity also accepts edge-touching voxels. Corner-only
// contact is intentionally excluded so unrelated boundaries are not merged.
constexpr int kClusterNeighbors[18][3] = {
    {1, 0, 0},   {-1, 0, 0},  {0, 1, 0},   {0, -1, 0},
    {0, 0, 1},   {0, 0, -1}, {1, 1, 0},   {1, -1, 0},
    {-1, 1, 0},  {-1, -1, 0}, {1, 0, 1},  {1, 0, -1},
    {-1, 0, 1},  {-1, 0, -1}, {0, 1, 1},  {0, 1, -1},
    {0, -1, 1}, {0, -1, -1}};

Vec3 subtract(const Vec3 &a, const Vec3 &b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 addScaled(const Vec3 &origin, const Vec3 &direction, double scale)
{
  return {origin.x + scale * direction.x,
          origin.y + scale * direction.y,
          origin.z + scale * direction.z};
}

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

double yawBetween(const Vec3 &from, const Vec3 &to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

double yawDifferenceDeg(double left, double right)
{
  constexpr double kPi = 3.14159265358979323846;
  return std::fabs(std::remainder(left - right, 2.0 * kPi)) *
         180.0 / kPi;
}
}  // namespace

std::size_t VoxelKeyHash::operator()(const VoxelKey &key) const
{
  std::size_t seed = std::hash<int64_t>{}(key.x);
  seed ^= std::hash<int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  seed ^= std::hash<int64_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  return seed;
}

ExplorerCore::ExplorerCore(ExplorerConfig config) : config_(std::move(config))
{
  sanitizeConfig();
}

void ExplorerCore::setPvbsmBatchQuery(PvbsmBatchQuery query)
{
  pvbsm_batch_query_ = std::move(query);
}

void ExplorerCore::sanitizeConfig()
{
  config_.robot_id = std::max(0, std::min(65535, config_.robot_id));
  config_.planning_voxel_size_m = std::max(0.1, config_.planning_voxel_size_m);
  config_.planning_sensor_range_m =
      std::max(config_.planning_voxel_size_m, config_.planning_sensor_range_m);
  config_.planning_map_radius_m =
      std::max(config_.planning_sensor_range_m, config_.planning_map_radius_m);
  config_.planning_prune_budget = std::max(1, config_.planning_prune_budget);
  config_.max_raycasts_per_update = std::max(1, config_.max_raycasts_per_update);
  config_.max_ray_steps = std::max(2, config_.max_ray_steps);
  config_.frontier_update_budget = std::max(1, config_.frontier_update_budget);
  config_.frontier_evaluation_budget =
      std::max(1, config_.frontier_evaluation_budget);
  config_.frontier_update_rate_hz =
      std::max(0.1, config_.frontier_update_rate_hz);
  config_.goal_evaluation_rate_hz =
      std::max(0.1, config_.goal_evaluation_rate_hz);
  config_.long_term_update_rate_hz =
      std::max(0.1, config_.long_term_update_rate_hz);
  config_.coverage_voxel_size_m = std::max(0.1, config_.coverage_voxel_size_m);
  config_.exploration_memory_voxel_size_m =
      std::max(0.1, config_.exploration_memory_voxel_size_m);
  config_.exploration_memory_min_observations =
      std::max(1, std::min(65535,
                           config_.exploration_memory_min_observations));
  config_.exploration_memory_max_range_m =
      std::max(config_.exploration_memory_voxel_size_m,
               config_.exploration_memory_max_range_m);
  config_.frontier_history_probe_step_m =
      std::max(config_.exploration_memory_voxel_size_m,
               config_.frontier_history_probe_step_m);
  config_.frontier_history_probe_distance_m =
      std::max(config_.frontier_history_probe_step_m,
               config_.frontier_history_probe_distance_m);
  config_.frontier_history_observed_ratio =
      clamp(config_.frontier_history_observed_ratio, 0.0, 1.0);
  config_.replan_interval_s = std::max(0.1, config_.replan_interval_s);
  config_.goal_timeout_s = std::max(0.0, config_.goal_timeout_s);
  config_.goal_min_hold_time_s =
      std::max(0.0, config_.goal_min_hold_time_s);
  config_.goal_blocked_confirm_updates =
      std::max(1, config_.goal_blocked_confirm_updates);
  config_.goal_reached_distance_m =
      std::max(0.1, config_.goal_reached_distance_m);
  config_.goal_progress_epsilon_m =
      std::max(0.01, config_.goal_progress_epsilon_m);
  config_.goal_stall_timeout_s =
      std::max(0.1, config_.goal_stall_timeout_s);
  config_.failed_goal_exclusion_radius_m =
      std::max(config_.planning_voxel_size_m,
               config_.failed_goal_exclusion_radius_m);
  config_.failed_goal_cooldown_s =
      std::max(0.1, config_.failed_goal_cooldown_s);
  config_.min_goal_distance_m =
      std::max(config_.goal_reached_distance_m, config_.min_goal_distance_m);
  config_.max_goal_distance_m =
      std::max(config_.min_goal_distance_m, config_.max_goal_distance_m);
  config_.max_goal_vertical_distance_m =
      std::max(config_.planning_voxel_size_m,
               config_.max_goal_vertical_distance_m);
  config_.min_known_free_path_ratio =
      clamp(config_.min_known_free_path_ratio, 0.0, 1.0);
  config_.goal_switch_margin = clamp(config_.goal_switch_margin, 0.0, 1.0);
  config_.frontier_cluster_size_m =
      std::max(config_.planning_voxel_size_m,
               config_.frontier_cluster_size_m);
  config_.min_frontier_cluster_cells =
      std::max(1, config_.min_frontier_cluster_cells);
  config_.viewpoint_standoff_m =
      std::max(config_.planning_voxel_size_m, config_.viewpoint_standoff_m);
  config_.viewpoint_search_radius_m =
      std::max(config_.planning_voxel_size_m,
               config_.viewpoint_search_radius_m);
  config_.viewpoint_same_height_tolerance_m =
      std::max(0.0, config_.viewpoint_same_height_tolerance_m);
  config_.min_wall_clearance_m =
      std::max(0.0, config_.min_wall_clearance_m);
  config_.max_viewpoints_per_cluster =
      std::max(1, config_.max_viewpoints_per_cluster);
  config_.max_safe_viewpoint_candidates =
      std::max(1, config_.max_safe_viewpoint_candidates);
  config_.preferred_heading_change_deg =
      clamp(config_.preferred_heading_change_deg, 0.0, 180.0);
  config_.max_heading_change_deg =
      clamp(config_.max_heading_change_deg,
            std::max(1.0, config_.preferred_heading_change_deg), 180.0);
  config_.distance_cost_weight = std::max(0.0, config_.distance_cost_weight);
  config_.heading_cost_weight = std::max(0.0, config_.heading_cost_weight);
  config_.arrival_yaw_cost_weight =
      std::max(0.0, config_.arrival_yaw_cost_weight);
  config_.reachability_max_expansions =
      std::max(32, config_.reachability_max_expansions);
  config_.goal_reachability_check_rate_hz =
      std::max(0.1, config_.goal_reachability_check_rate_hz);
  config_.lio_busy_threshold_ms = std::max(1.0, config_.lio_busy_threshold_ms);
  config_.lio_overload_threshold_ms =
      std::max(config_.lio_busy_threshold_ms, config_.lio_overload_threshold_ms);
  config_.lio_time_ema_alpha = clamp(config_.lio_time_ema_alpha, 0.01, 1.0);
  config_.busy_budget_scale = clamp(config_.busy_budget_scale, 0.05, 1.0);
  config_.overload_budget_scale =
      clamp(config_.overload_budget_scale, 0.05, config_.busy_budget_scale);
  config_.degenerate_max_goal_distance_m =
      clamp(config_.degenerate_max_goal_distance_m,
            config_.min_goal_distance_m, config_.max_goal_distance_m);
  config_.degenerate_goal_switch_margin =
      clamp(config_.degenerate_goal_switch_margin,
            config_.goal_switch_margin, 1.0);
  config_.degenerate_safe_path_weight =
      std::max(0.0, config_.degenerate_safe_path_weight);
  config_.pvbsm_root_voxel_size_m =
      std::max(0.1, config_.pvbsm_root_voxel_size_m);
  config_.pvbsm_submap_edge_roots =
      std::max(1, std::min(255, config_.pvbsm_submap_edge_roots));
  config_.pvbsm_covered_root_target =
      std::max(1, config_.pvbsm_covered_root_target);
  config_.pvbsm_unseen_submap_bonus =
      std::max(0.0, config_.pvbsm_unseen_submap_bonus);
  config_.pvbsm_submap_coverage_penalty =
      std::max(0.0, config_.pvbsm_submap_coverage_penalty);
  config_.pvbsm_observed_root_penalty =
      std::max(0.0, config_.pvbsm_observed_root_penalty);
  config_.pvbsm_degenerate_structure_bonus =
      std::max(0.0, config_.pvbsm_degenerate_structure_bonus);
}

double ExplorerCore::distance(const Vec3 &a, const Vec3 &b)
{
  const Vec3 delta = subtract(a, b);
  return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
}

VoxelKey ExplorerCore::key(const Vec3 &point, double voxel_size) const
{
  return {static_cast<int64_t>(std::floor(point.x / voxel_size)),
          static_cast<int64_t>(std::floor(point.y / voxel_size)),
          static_cast<int64_t>(std::floor(point.z / voxel_size))};
}

Vec3 ExplorerCore::center(const VoxelKey &voxel, double voxel_size) const
{
  return {(voxel.x + 0.5) * voxel_size,
          (voxel.y + 0.5) * voxel_size,
          (voxel.z + 0.5) * voxel_size};
}

int ExplorerCore::cellState(const VoxelKey &voxel) const
{
  const auto iter = map_.find(voxel);
  if (iter == map_.end()) return -1;
  if (iter->second.log_odds >= 2) return 1;
  if (iter->second.log_odds <= -1) return 0;
  return -1;
}

bool ExplorerCore::isFrontierVoxel(const VoxelKey &voxel) const
{
  if (cellState(voxel) != 0) return false;

  bool has_unknown_neighbor = false;
  int free_neighbors = 0;
  for (const auto &offset : kFaceNeighbors)
  {
    const int neighbor_state =
        cellState({voxel.x + offset[0], voxel.y + offset[1],
                   voxel.z + offset[2]});
    if (neighbor_state < 0)
      has_unknown_neighbor = true;
    else if (neighbor_state == 0)
      ++free_neighbors;
  }
  if (!has_unknown_neighbor || free_neighbors < 2) return false;

  for (int dx = -1; dx <= 1; ++dx)
  {
    for (int dy = -1; dy <= 1; ++dy)
    {
      for (int dz = -1; dz <= 1; ++dz)
      {
        if (dx == 0 && dy == 0 && dz == 0) continue;
        if (cellState({voxel.x + dx, voxel.y + dy, voxel.z + dz}) == 1)
          return false;
      }
    }
  }
  return true;
}

void ExplorerCore::markFrontierDirty(const VoxelKey &voxel)
{
  for (int dx = -1; dx <= 1; ++dx)
  {
    for (int dy = -1; dy <= 1; ++dy)
    {
      for (int dz = -1; dz <= 1; ++dz)
        dirty_frontiers_.insert({voxel.x + dx, voxel.y + dy, voxel.z + dz});
    }
  }
}

void ExplorerCore::updateCell(const VoxelKey &voxel, int delta)
{
  const int old_state = cellState(voxel);
  const bool is_new = map_.find(voxel) == map_.end();
  Cell &cell = map_[voxel];
  if (is_new) map_queue_.push_back(voxel);
  cell.log_odds = static_cast<int16_t>(
      std::max(-5, std::min(5, static_cast<int>(cell.log_odds) + delta)));
  cell.last_update = update_id_;
  const int new_state = cellState(voxel);
  if (old_state == new_state) return;
  if (old_state == 0 && stats_.free_cells > 0) --stats_.free_cells;
  if (old_state == 1 && stats_.occupied_cells > 0) --stats_.occupied_cells;
  if (new_state == 0) ++stats_.free_cells;
  if (new_state == 1) ++stats_.occupied_cells;
  markFrontierDirty(voxel);
}

void ExplorerCore::setHealth(bool degenerate, double degeneracy_score,
                             double lio_runtime_ms)
{
  degenerate_ = degenerate;
  degeneracy_score_ = std::isfinite(degeneracy_score) ? degeneracy_score : 0.0;
  if (std::isfinite(lio_runtime_ms) && lio_runtime_ms >= 0.0)
  {
    if (smoothed_lio_ms_ < 0.0) smoothed_lio_ms_ = lio_runtime_ms;
    else
    {
      const double alpha = config_.lio_time_ema_alpha;
      smoothed_lio_ms_ =
          alpha * lio_runtime_ms + (1.0 - alpha) * smoothed_lio_ms_;
    }
  }

  stats_.smoothed_lio_time_ms = std::max(0.0, smoothed_lio_ms_);
  stats_.budget_scale = 1.0;
  if (config_.dynamic_budget_enabled && smoothed_lio_ms_ >= 0.0)
  {
    if (smoothed_lio_ms_ >= config_.lio_overload_threshold_ms)
      stats_.budget_scale = config_.overload_budget_scale;
    else if (smoothed_lio_ms_ >= config_.lio_busy_threshold_ms)
      stats_.budget_scale = config_.busy_budget_scale;
  }
}

void ExplorerCore::integrateCloud(const Vec3 &origin,
                                  const std::vector<Vec3> &points)
{
  // The vehicle physically occupies this voxel, so it is direct free-space
  // evidence rather than one weak ray miss. Clear stale endpoint hits
  // immediately; otherwise a previously occupied voxel can remain occupied
  // for several map cycles after the vehicle has entered it and EGO will
  // reject every trajectory as starting inside an obstacle.
  updateCell(key(origin, config_.planning_voxel_size_m), -10);
  std::unordered_map<VoxelKey, uint8_t, VoxelKeyHash> memory_frame;
  const int ray_budget = std::max(
      1, static_cast<int>(std::lround(
             config_.max_raycasts_per_update * stats_.budget_scale)));
  stats_.effective_raycasts = ray_budget;
  if (points.empty())
    return;

  const std::size_t max_rays = static_cast<std::size_t>(ray_budget);
  const std::size_t stride =
      std::max<std::size_t>(1, (points.size() + max_rays - 1) / max_rays);
  std::size_t sampled = 0;
  for (std::size_t index = 0;
       index < points.size() && sampled < max_rays;
       index += stride, ++sampled)
  {
    const Vec3 endpoint = points[index];
    const Vec3 ray = subtract(endpoint, origin);
    const double range = distance(endpoint, origin);
    if (!std::isfinite(range) ||
        range < config_.planning_voxel_size_m ||
        range > config_.planning_sensor_range_m)
      continue;

    if (config_.exploration_memory_enabled)
      memory_frame[key(origin, config_.exploration_memory_voxel_size_m)] |= 1U;

    const int steps = std::min(
        config_.max_ray_steps,
        std::max(1, static_cast<int>(
                        std::ceil(range / config_.planning_voxel_size_m))));
    VoxelKey previous = key(origin, config_.planning_voxel_size_m);
    for (int step = 1; step < steps; ++step)
    {
      const VoxelKey free_key =
          key(addScaled(origin, ray, static_cast<double>(step) / steps),
              config_.planning_voxel_size_m);
      if (!(free_key == previous))
      {
        updateCell(free_key, -1);
        previous = free_key;
      }
    }
    updateCell(key(endpoint, config_.planning_voxel_size_m), 2);

    if (config_.exploration_memory_enabled)
    {
      const double memory_range =
          std::min(range, config_.exploration_memory_max_range_m);
      const int memory_steps = std::max(
          1, static_cast<int>(std::ceil(
                 memory_range / config_.exploration_memory_voxel_size_m)));
      VoxelKey previous_memory =
          key(origin, config_.exploration_memory_voxel_size_m);
      const bool endpoint_in_memory =
          range <= config_.exploration_memory_max_range_m;
      const int last_free_step =
          endpoint_in_memory ? memory_steps - 1 : memory_steps;
      for (int step = 1; step <= last_free_step; ++step)
      {
        const VoxelKey free_key = key(
            addScaled(origin, ray,
                      memory_range / range *
                          static_cast<double>(step) / memory_steps),
            config_.exploration_memory_voxel_size_m);
        if (!(free_key == previous_memory))
        {
          memory_frame[free_key] |= 1U;
          previous_memory = free_key;
        }
      }
      if (endpoint_in_memory)
        memory_frame[key(endpoint,
                         config_.exploration_memory_voxel_size_m)] |= 2U;
    }
  }
  updateExplorationMemory(memory_frame);
}

void ExplorerCore::updateExplorationMemory(
    const std::unordered_map<VoxelKey, uint8_t, VoxelKeyHash> &frame_evidence)
{
  if (!config_.exploration_memory_enabled) return;
  for (const auto &entry : frame_evidence)
  {
    ExplorationMemoryCell &cell = exploration_memory_[entry.first];
    const bool was_stable =
        cell.observations >= config_.exploration_memory_min_observations;
    if (cell.observations < std::numeric_limits<uint16_t>::max())
      ++cell.observations;
    cell.evidence |= entry.second;
    if (!was_stable &&
        cell.observations >= config_.exploration_memory_min_observations)
      ++stats_.stable_exploration_memory_cells;
  }
  stats_.exploration_memory_cells = exploration_memory_.size();
}

bool ExplorerCore::explorationMemoryObserved(const VoxelKey &voxel) const
{
  const auto iter = exploration_memory_.find(voxel);
  return iter != exploration_memory_.end() &&
         iter->second.observations >=
             config_.exploration_memory_min_observations;
}

double ExplorerCore::clusterHistoricalObservedRatio(
    const std::vector<VoxelKey> &cluster,
    std::size_t *probe_cells) const
{
  std::unordered_set<VoxelKey, VoxelKeyHash> probes;
  if (!config_.exploration_memory_enabled || cluster.empty())
  {
    if (probe_cells) *probe_cells = 0;
    return 0.0;
  }

  for (const VoxelKey &frontier : cluster)
  {
    Vec3 unknown_direction;
    for (const auto &offset : kFaceNeighbors)
    {
      if (cellState({frontier.x + offset[0], frontier.y + offset[1],
                     frontier.z + offset[2]}) < 0)
      {
        unknown_direction.x += offset[0];
        unknown_direction.y += offset[1];
        unknown_direction.z += offset[2];
      }
    }
    const double norm = std::sqrt(
        unknown_direction.x * unknown_direction.x +
        unknown_direction.y * unknown_direction.y +
        unknown_direction.z * unknown_direction.z);
    if (norm < 1e-6) continue;
    unknown_direction.x /= norm;
    unknown_direction.y /= norm;
    unknown_direction.z /= norm;
    const Vec3 origin = center(frontier, config_.planning_voxel_size_m);
    for (double distance_m = config_.frontier_history_probe_step_m;
         distance_m <= config_.frontier_history_probe_distance_m + 1e-9;
         distance_m += config_.frontier_history_probe_step_m)
    {
      probes.insert(key(addScaled(origin, unknown_direction, distance_m),
                        config_.exploration_memory_voxel_size_m));
    }
  }

  if (probe_cells) *probe_cells = probes.size();
  if (probes.empty()) return 0.0;
  std::size_t observed = 0;
  for (const VoxelKey &probe : probes)
  {
    if (explorationMemoryObserved(probe)) ++observed;
  }
  return static_cast<double>(observed) / probes.size();
}

void ExplorerCore::prune(const Vec3 &position)
{
  int processed = 0;
  while (!map_queue_.empty() && processed < config_.planning_prune_budget)
  {
    const VoxelKey voxel = map_queue_.front();
    map_queue_.pop_front();
    auto iter = map_.find(voxel);
    if (iter == map_.end())
    {
      ++processed;
      continue;
    }
    if (distance(center(voxel, config_.planning_voxel_size_m), position) >
        config_.planning_map_radius_m)
    {
      const int old_state = cellState(voxel);
      if (old_state == 0 && stats_.free_cells > 0) --stats_.free_cells;
      if (old_state == 1 && stats_.occupied_cells > 0) --stats_.occupied_cells;
      frontiers_.erase(voxel);
      dirty_frontiers_.erase(voxel);
      map_.erase(iter);
      markFrontierDirty(voxel);
    }
    else
      map_queue_.push_back(voxel);
    ++processed;
  }
}

void ExplorerCore::updateFrontiers()
{
  const int budget = std::max(
      1, static_cast<int>(std::lround(
             config_.frontier_update_budget * stats_.budget_scale)));
  stats_.effective_frontier_updates = budget;
  int processed = 0;
  while (!dirty_frontiers_.empty() && processed < budget)
  {
    const auto dirty_iter = dirty_frontiers_.begin();
    const VoxelKey voxel = *dirty_iter;
    dirty_frontiers_.erase(dirty_iter);
    if (isFrontierVoxel(voxel)) frontiers_.insert(voxel);
    else frontiers_.erase(voxel);
    ++processed;
  }
  stats_.frontier_cells = frontiers_.size();
}

bool ExplorerCore::segmentBlocked(const Vec3 &start, const Vec3 &end,
                                  double *known_free_ratio) const
{
  const Vec3 segment = subtract(end, start);
  const double length = distance(start, end);
  if (!std::isfinite(length) || length <= 0.0)
  {
    if (known_free_ratio) *known_free_ratio = 1.0;
    return false;
  }
  const int steps = std::max(
      1, static_cast<int>(std::ceil(length / config_.planning_voxel_size_m)));
  int known_free = 0;
  for (int step = 1; step <= steps; ++step)
  {
    const int state =
        cellState(key(addScaled(start, segment,
                                static_cast<double>(step) / steps),
                      config_.planning_voxel_size_m));
    if (state == 1)
    {
      if (known_free_ratio)
        *known_free_ratio = static_cast<double>(known_free) / step;
      return true;
    }
    if (state == 0) ++known_free;
  }
  if (known_free_ratio)
    *known_free_ratio = static_cast<double>(known_free) / steps;
  return false;
}

bool ExplorerCore::pathReachable(const Vec3 &start, const Vec3 &end,
                                 int max_expansions,
                                 bool *budget_exhausted) const
{
  if (budget_exhausted) *budget_exhausted = false;
  if (!config_.reachability_enabled || !segmentBlocked(start, end))
    return true;

  const VoxelKey start_key = key(start, config_.planning_voxel_size_m);
  const VoxelKey goal_key = key(end, config_.planning_voxel_size_m);
  if (cellState(goal_key) != 0) return false;
  struct Node
  {
    VoxelKey key;
    int g = 0;
    int f = 0;
  };
  struct Greater
  {
    bool operator()(const Node &left, const Node &right) const
    {
      return left.f > right.f;
    }
  };
  const auto heuristic = [&goal_key](const VoxelKey &voxel)
  {
    return static_cast<int>(std::llabs(voxel.x - goal_key.x) +
                            std::llabs(voxel.y - goal_key.y) +
                            std::llabs(voxel.z - goal_key.z));
  };
  std::priority_queue<Node, std::vector<Node>, Greater> open;
  std::unordered_map<VoxelKey, int, VoxelKeyHash> best_g;
  open.push({start_key, 0, heuristic(start_key)});
  best_g[start_key] = 0;
  int expansions = 0;
  while (!open.empty() && expansions < max_expansions)
  {
    const Node current = open.top();
    open.pop();
    const auto best_iter = best_g.find(current.key);
    if (best_iter == best_g.end() || current.g != best_iter->second) continue;
    if (current.key == goal_key) return true;
    ++expansions;
    for (const auto &offset : kFaceNeighbors)
    {
      const VoxelKey next{current.key.x + offset[0],
                          current.key.y + offset[1],
                          current.key.z + offset[2]};
      if (!(next == goal_key) && cellState(next) != 0) continue;
      const int next_g = current.g + 1;
      const auto next_iter = best_g.find(next);
      if (next_iter != best_g.end() && next_iter->second <= next_g) continue;
      best_g[next] = next_g;
      open.push({next, next_g, next_g + heuristic(next)});
    }
  }
  if (budget_exhausted && !open.empty()) *budget_exhausted = true;
  return false;
}

bool ExplorerCore::hasWallClearance(const VoxelKey &voxel) const
{
  const int radius = static_cast<int>(std::ceil(
      config_.min_wall_clearance_m / config_.planning_voxel_size_m));
  const double max_distance_sq =
      config_.min_wall_clearance_m * config_.min_wall_clearance_m;
  for (int dx = -radius; dx <= radius; ++dx)
  {
    for (int dy = -radius; dy <= radius; ++dy)
    {
      for (int dz = -radius; dz <= radius; ++dz)
      {
        const double metric_sq = config_.planning_voxel_size_m *
            config_.planning_voxel_size_m * (dx * dx + dy * dy + dz * dz);
        if (metric_sq > max_distance_sq) continue;
        if (cellState({voxel.x + dx, voxel.y + dy, voxel.z + dz}) == 1)
          return false;
      }
    }
  }
  return true;
}

std::vector<std::vector<VoxelKey>> ExplorerCore::frontierClusters()
{
  const auto start = std::chrono::steady_clock::now();
  std::vector<std::vector<VoxelKey>> clusters;
  std::unordered_set<VoxelKey, VoxelKeyHash> unvisited;
  std::vector<VoxelKey> stale;
  unvisited.reserve(frontiers_.size());
  stale.reserve(frontiers_.size());
  stats_.rejected_stale_frontiers = 0;
  for (const VoxelKey &voxel : frontiers_)
  {
    if (isFrontierVoxel(voxel))
      unvisited.insert(voxel);
    else
    {
      stale.push_back(voxel);
      ++stats_.rejected_stale_frontiers;
    }
  }
  for (const VoxelKey &voxel : stale) frontiers_.erase(voxel);
  stats_.frontier_cells = frontiers_.size();
  stats_.valid_frontier_cells = unvisited.size();
  stats_.frontier_components = 0;
  stats_.rejected_small_clusters = 0;
  clusters.reserve(unvisited.size());

  std::queue<VoxelKey> open;
  while (!unvisited.empty())
  {
    std::vector<VoxelKey> component;
    const VoxelKey seed = *unvisited.begin();
    unvisited.erase(seed);
    open.push(seed);
    while (!open.empty())
    {
      const VoxelKey current = open.front();
      open.pop();
      component.push_back(current);
      for (const auto &offset : kClusterNeighbors)
      {
        const VoxelKey neighbor{current.x + offset[0],
                                current.y + offset[1],
                                current.z + offset[2]};
        const auto neighbor_iter = unvisited.find(neighbor);
        if (neighbor_iter == unvisited.end()) continue;
        open.push(*neighbor_iter);
        unvisited.erase(neighbor_iter);
      }
    }
    ++stats_.frontier_components;
    if (component.size() <
        static_cast<std::size_t>(config_.min_frontier_cluster_cells))
    {
      ++stats_.rejected_small_clusters;
      continue;
    }
    clusters.push_back(std::move(component));
  }
  stats_.last_cluster_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  return clusters;
}

std::vector<ExplorerCore::SafeViewpoint> ExplorerCore::makeSafeViewpoints(
    const std::vector<VoxelKey> &cluster,
    const Vec3 &position,
    int limit) const
{
  std::vector<SafeViewpoint> viewpoints;
  if (cluster.empty() || limit <= 0) return viewpoints;

  std::vector<VoxelKey> anchor_pool = cluster;
  std::sort(
      anchor_pool.begin(), anchor_pool.end(),
      [this, &position](const VoxelKey &left, const VoxelKey &right)
      {
        const double left_distance =
            distance(center(left, config_.planning_voxel_size_m), position);
        const double right_distance =
            distance(center(right, config_.planning_voxel_size_m), position);
        if (left_distance != right_distance)
          return left_distance < right_distance;
        if (left.x != right.x) return left.x < right.x;
        if (left.y != right.y) return left.y < right.y;
        return left.z < right.z;
      });

  std::vector<VoxelKey> anchors;
  anchors.reserve(std::min<std::size_t>(
      anchor_pool.size(), static_cast<std::size_t>(limit)));
  anchors.push_back(anchor_pool.front());
  while (anchors.size() < anchor_pool.size() &&
         static_cast<int>(anchors.size()) < limit)
  {
    const VoxelKey *best = nullptr;
    double best_separation = -1.0;
    for (const VoxelKey &voxel : anchor_pool)
    {
      if (std::find(anchors.begin(), anchors.end(), voxel) != anchors.end())
        continue;
      double nearest_anchor = std::numeric_limits<double>::infinity();
      const Vec3 point = center(voxel, config_.planning_voxel_size_m);
      for (const VoxelKey &anchor : anchors)
        nearest_anchor = std::min(
            nearest_anchor,
            distance(point, center(anchor, config_.planning_voxel_size_m)));
      if (nearest_anchor > best_separation)
      {
        best_separation = nearest_anchor;
        best = &voxel;
      }
    }
    if (!best) break;
    anchors.push_back(*best);
  }

  std::unordered_set<VoxelKey, VoxelKeyHash> used_viewpoints;
  const int search_radius = static_cast<int>(std::ceil(
      config_.viewpoint_search_radius_m / config_.planning_voxel_size_m));
  for (const VoxelKey &anchor : anchors)
  {
    const Vec3 target = center(anchor, config_.planning_voxel_size_m);
    Vec3 unknown_direction;
    for (const auto &offset : kFaceNeighbors)
    {
      if (cellState({anchor.x + offset[0], anchor.y + offset[1],
                     anchor.z + offset[2]}) < 0)
      {
        unknown_direction.x += offset[0];
        unknown_direction.y += offset[1];
        unknown_direction.z += offset[2];
      }
    }
    const double direction_norm = std::sqrt(
        unknown_direction.x * unknown_direction.x +
        unknown_direction.y * unknown_direction.y +
        unknown_direction.z * unknown_direction.z);
    Vec3 observation_target = target;
    Vec3 desired = target;
    if (direction_norm >= 1e-6)
    {
      observation_target.x += config_.planning_voxel_size_m *
                              unknown_direction.x / direction_norm;
      observation_target.y += config_.planning_voxel_size_m *
                              unknown_direction.y / direction_norm;
      observation_target.z += config_.planning_voxel_size_m *
                              unknown_direction.z / direction_norm;
      desired.x -= config_.viewpoint_standoff_m *
                   unknown_direction.x / direction_norm;
      desired.y -= config_.viewpoint_standoff_m *
                   unknown_direction.y / direction_norm;
      desired.z -= config_.viewpoint_standoff_m *
                   unknown_direction.z / direction_norm;
    }

    const VoxelKey desired_key = key(desired, config_.planning_voxel_size_m);
    bool found = false;
    VoxelKey best_key;
    int best_height_tier = 2;
    double best_vertical = std::numeric_limits<double>::infinity();
    double best_offset = std::numeric_limits<double>::infinity();
    for (int dx = -search_radius; dx <= search_radius; ++dx)
    {
      for (int dy = -search_radius; dy <= search_radius; ++dy)
      {
        for (int dz = -search_radius; dz <= search_radius; ++dz)
        {
          const VoxelKey candidate_key{desired_key.x + dx,
                                       desired_key.y + dy,
                                       desired_key.z + dz};
          if (used_viewpoints.find(candidate_key) != used_viewpoints.end() ||
              cellState(candidate_key) != 0 ||
              !hasWallClearance(candidate_key))
            continue;
          const Vec3 candidate =
              center(candidate_key, config_.planning_voxel_size_m);
          if (segmentBlocked(candidate, target)) continue;
          const double vertical = std::fabs(candidate.z - position.z);
          const int height_tier =
              vertical <= config_.viewpoint_same_height_tolerance_m ? 0 : 1;
          const double offset = distance(candidate, desired);
          if (!found || height_tier < best_height_tier ||
              (height_tier == best_height_tier && vertical < best_vertical) ||
              (height_tier == best_height_tier && vertical == best_vertical &&
               offset < best_offset))
          {
            found = true;
            best_key = candidate_key;
            best_height_tier = height_tier;
            best_vertical = vertical;
            best_offset = offset;
          }
        }
      }
    }
    if (!found)
    {
      // Sparse ray maps may not contain the ideal standoff voxel. Preserve
      // the old safe fallback to a known-free frontier voxel, while still
      // keeping viewpoints unique across anchors.
      std::vector<std::pair<double, VoxelKey>> fallback;
      fallback.reserve(cluster.size());
      for (const VoxelKey &voxel : cluster)
      {
        if (used_viewpoints.find(voxel) != used_viewpoints.end() ||
            cellState(voxel) != 0 || !hasWallClearance(voxel))
          continue;
        fallback.emplace_back(
            distance(center(voxel, config_.planning_voxel_size_m), desired),
            voxel);
      }
      std::sort(
          fallback.begin(), fallback.end(),
          [](const std::pair<double, VoxelKey> &left,
             const std::pair<double, VoxelKey> &right)
          {
            if (left.first != right.first) return left.first < right.first;
            if (left.second.x != right.second.x)
              return left.second.x < right.second.x;
            if (left.second.y != right.second.y)
              return left.second.y < right.second.y;
            return left.second.z < right.second.z;
          });
      if (fallback.empty()) continue;
      best_key = fallback.front().second;
    }
    used_viewpoints.insert(best_key);
    viewpoints.push_back(
        {center(best_key, config_.planning_voxel_size_m),
         observation_target,
         anchor});
  }
  return viewpoints;
}

double ExplorerCore::currentYaw() const
{
  const double sin_yaw = 2.0 *
      (current_orientation_.w * current_orientation_.z +
       current_orientation_.x * current_orientation_.y);
  const double cos_yaw = 1.0 - 2.0 *
      (current_orientation_.y * current_orientation_.y +
       current_orientation_.z * current_orientation_.z);
  return std::atan2(sin_yaw, cos_yaw);
}

double ExplorerCore::headingChange(const Vec3 &position,
                                   const Vec3 &goal) const
{
  constexpr double kPi = 3.14159265358979323846;
  const double desired = std::atan2(goal.y - position.y,
                                    goal.x - position.x);
  return std::fabs(std::remainder(desired - currentYaw(), 2.0 * kPi)) *
         180.0 / kPi;
}

bool ExplorerCore::nearFailedGoal(const Vec3 &point) const
{
  for (const FailedGoal &failed : failed_goals_)
  {
    if (distance(point, failed.position) <=
        config_.failed_goal_exclusion_radius_m)
      return true;
  }
  return false;
}

void ExplorerCore::pruneFailedGoals(double timestamp)
{
  failed_goals_.erase(
      std::remove_if(failed_goals_.begin(), failed_goals_.end(),
                     [timestamp](const FailedGoal &failed)
                     {
                       return failed.expires_at <= timestamp;
                     }),
      failed_goals_.end());
  stats_.failed_goals_in_cooldown = failed_goals_.size();
}

void ExplorerCore::recordFailedGoal(double timestamp)
{
  if (!decision_.valid || decision_.generation == last_failed_generation_)
    return;
  failed_goals_.push_back(
      {decision_.position, timestamp + config_.failed_goal_cooldown_s});
  last_failed_generation_ = decision_.generation;
  stats_.failed_goals_in_cooldown = failed_goals_.size();
}

void ExplorerCore::resetGoalProgress(const Vec3 &position, double timestamp)
{
  if (!decision_.valid)
  {
    best_goal_distance_m_ = std::numeric_limits<double>::infinity();
    last_goal_progress_time_ = -1.0;
    goal_stalled_ = false;
    return;
  }
  best_goal_distance_m_ = distance(decision_.position, position);
  last_goal_progress_time_ = timestamp;
  goal_stalled_ = false;
}

double ExplorerCore::frontierScore(const VoxelKey &voxel) const
{
  int unknown_neighbors = 0;
  int occupied_neighbors = 0;
  for (const auto &offset : kFaceNeighbors)
  {
    const int state = cellState(
        {voxel.x + offset[0], voxel.y + offset[1], voxel.z + offset[2]});
    if (state < 0) ++unknown_neighbors;
    else if (state > 0) ++occupied_neighbors;
  }
  const Vec3 point = center(voxel, config_.planning_voxel_size_m);
  const auto visit_iter = visits_.find(key(point, config_.coverage_voxel_size_m));
  const uint32_t visits =
      visit_iter == visits_.end() ? 0U : visit_iter->second;
  const double novelty = 1.0 / (1.0 + visits);
  const double weakness =
      degenerate_
          ? 1.0
          : clamp(1.0 - degeneracy_score_ / 0.08, 0.0, 1.0);
  return 1.5 * unknown_neighbors + 2.0 * novelty +
         0.35 * weakness * occupied_neighbors;
}

double ExplorerCore::pvbsmScoreAdjustment(
    const PvbsmExplorationHint &hint) const
{
  if (!config_.pvbsm_scoring_enabled) return 0.0;
  double adjustment = 0.0;
  if (!hint.submap_observed)
    adjustment += config_.pvbsm_unseen_submap_bonus;
  else
    adjustment -= config_.pvbsm_submap_coverage_penalty *
                  clamp(hint.submap_coverage, 0.0, 1.0);
  if (hint.root_observed)
    adjustment -= config_.pvbsm_observed_root_penalty;
  if (degenerate_)
    adjustment += config_.pvbsm_degenerate_structure_bonus *
                  clamp(hint.structural_support, 0.0, 1.0);
  return adjustment;
}

void ExplorerCore::updateVisitMemory(const Vec3 &position)
{
  const VoxelKey coverage_key = key(position, config_.coverage_voxel_size_m);
  ++visits_[coverage_key];
  stats_.visited_cells = visits_.size();
}

void ExplorerCore::updateDecision(const Vec3 &position, double timestamp)
{
  std::vector<std::vector<VoxelKey>> clusters = frontierClusters();
  valid_cluster_frontiers_.clear();
  for (const std::vector<VoxelKey> &cluster : clusters)
    valid_cluster_frontiers_.insert(
        valid_cluster_frontiers_.end(), cluster.begin(), cluster.end());

  stats_.historical_clusters_checked = 0;
  stats_.historical_clusters_observed = 0;
  stats_.rejected_historical_clusters = 0;
  stats_.historical_probe_cells = 0;
  if (config_.exploration_memory_enabled)
  {
    clusters.erase(
        std::remove_if(
            clusters.begin(), clusters.end(),
            [this](const std::vector<VoxelKey> &cluster)
            {
              std::size_t probe_cells = 0;
              const double observed_ratio =
                  clusterHistoricalObservedRatio(cluster, &probe_cells);
              ++stats_.historical_clusters_checked;
              stats_.historical_probe_cells += probe_cells;
              const bool historically_observed =
                  probe_cells > 0 &&
                  observed_ratio + 1e-9 >=
                      config_.frontier_history_observed_ratio;
              if (historically_observed)
                ++stats_.historical_clusters_observed;
              if (!config_.exploration_memory_filter_enabled ||
                  !historically_observed)
                return false;
              ++stats_.rejected_historical_clusters;
              return true;
            }),
        clusters.end());
  }

  pruneFailedGoals(timestamp);
  const bool had_goal = decision_.valid;
  const bool failed_active_goal =
      had_goal && (goal_blocked_ || goal_stalled_ || goal_timeout_);
  if (failed_active_goal) recordFailedGoal(timestamp);

  const bool hold =
      had_goal && goal_set_time_ >= 0.0 &&
      timestamp - goal_set_time_ < config_.goal_min_hold_time_s;
  if (hold && !goal_reached_ && !failed_active_goal) return;
  if (had_goal && !goal_reached_ && !failed_active_goal &&
      !config_.allow_periodic_goal_switch)
    return;
  const bool periodic =
      last_plan_time_ < 0.0 ||
      timestamp - last_plan_time_ >= config_.replan_interval_s;
  if (had_goal && !goal_reached_ && !failed_active_goal && !periodic)
    return;

  const auto start = std::chrono::steady_clock::now();
  const int budget = std::max(
      1, static_cast<int>(std::lround(
             config_.frontier_evaluation_budget * stats_.budget_scale)));
  stats_.effective_frontier_evaluations = budget;
  const double max_distance =
      degenerate_ ? config_.degenerate_max_goal_distance_m
                  : config_.max_goal_distance_m;
  stats_.rejected_no_viewpoint = 0;
  stats_.rejected_distance = 0;
  stats_.rejected_vertical_distance = 0;
  stats_.rejected_heading = 0;
  stats_.rejected_known_free_path = 0;
  stats_.rejected_failed_goal = 0;
  stats_.candidates_scored = 0;
  struct Candidate
  {
    VoxelKey voxel;
    Vec3 point;
    const std::vector<VoxelKey> *cluster = nullptr;
    double known_free = 0.0;
    double distance = 0.0;
    double heading_change_deg = 0.0;
    double observation_yaw = 0.0;
    double rotation_cost_deg = 0.0;
    int tier = 0;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(static_cast<std::size_t>(budget));
  stats_.frontier_clusters = clusters.size();
  struct ClusterOrder
  {
    const std::vector<VoxelKey> *cluster = nullptr;
    int heading_tier = 0;
    VoxelKey stable_key;
  };
  std::vector<ClusterOrder> ordered_clusters;
  ordered_clusters.reserve(clusters.size());
  for (const std::vector<VoxelKey> &cluster : clusters)
  {
    Vec3 centroid;
    VoxelKey stable_key = cluster.front();
    for (const VoxelKey &voxel : cluster)
    {
      const Vec3 point = center(voxel, config_.planning_voxel_size_m);
      centroid.x += point.x;
      centroid.y += point.y;
      centroid.z += point.z;
      if (voxel.x < stable_key.x ||
          (voxel.x == stable_key.x && voxel.y < stable_key.y) ||
          (voxel.x == stable_key.x && voxel.y == stable_key.y &&
           voxel.z < stable_key.z))
        stable_key = voxel;
    }
    centroid.x /= cluster.size();
    centroid.y /= cluster.size();
    centroid.z /= cluster.size();
    const double approximate_heading = headingChange(position, centroid);
    const int approximate_tier =
        approximate_heading <= config_.preferred_heading_change_deg
            ? 0
            : approximate_heading <= config_.max_heading_change_deg
                  ? 1
                  : 2;
    ordered_clusters.push_back({&cluster, approximate_tier, stable_key});
  }
  std::sort(
      ordered_clusters.begin(), ordered_clusters.end(),
      [](const ClusterOrder &left, const ClusterOrder &right)
      {
        if (left.heading_tier != right.heading_tier)
          return left.heading_tier < right.heading_tier;
        if (left.cluster->size() != right.cluster->size())
          return left.cluster->size() > right.cluster->size();
        if (left.stable_key.x != right.stable_key.x)
          return left.stable_key.x < right.stable_key.x;
        if (left.stable_key.y != right.stable_key.y)
          return left.stable_key.y < right.stable_key.y;
        return left.stable_key.z < right.stable_key.z;
      });
  int evaluated = 0;
  const int viewpoint_budget = std::max(
      1, static_cast<int>(std::lround(
             config_.max_safe_viewpoint_candidates * stats_.budget_scale)));
  for (const ClusterOrder &ordered_cluster : ordered_clusters)
  {
    if (evaluated++ >= budget) break;
    if (static_cast<int>(candidates.size()) >= viewpoint_budget) break;
    const std::vector<VoxelKey> &frontier_cluster = *ordered_cluster.cluster;
    const std::vector<SafeViewpoint> safe_viewpoints = makeSafeViewpoints(
        frontier_cluster, position, config_.max_viewpoints_per_cluster);
    if (safe_viewpoints.empty())
    {
      ++stats_.rejected_no_viewpoint;
      continue;
    }
    for (const SafeViewpoint &safe_viewpoint : safe_viewpoints)
    {
      if (static_cast<int>(candidates.size()) >= viewpoint_budget) break;
      const Vec3 &candidate = safe_viewpoint.point;
      const double candidate_distance = distance(candidate, position);
      if (candidate_distance < config_.min_goal_distance_m ||
          candidate_distance > max_distance)
      {
        ++stats_.rejected_distance;
        continue;
      }
      const double vertical_distance = std::fabs(candidate.z - position.z);
      if (vertical_distance > config_.max_goal_vertical_distance_m)
      {
        ++stats_.rejected_vertical_distance;
        continue;
      }
      if (nearFailedGoal(candidate))
      {
        ++stats_.rejected_failed_goal;
        continue;
      }
      double known_free = 0.0;
      segmentBlocked(position, candidate, &known_free);
      if (known_free + 1e-9 < config_.min_known_free_path_ratio)
      {
        ++stats_.rejected_known_free_path;
        continue;
      }
      const double travel_yaw = yawBetween(position, candidate);
      const double heading = yawDifferenceDeg(currentYaw(), travel_yaw);
      if (heading > config_.max_heading_change_deg)
      {
        ++stats_.rejected_heading;
        continue;
      }
      const double observation_yaw =
          yawBetween(candidate, safe_viewpoint.observation_target);
      const double rotation_cost =
          heading + config_.arrival_yaw_cost_weight *
                        yawDifferenceDeg(travel_yaw, observation_yaw);
      const bool same_height =
          vertical_distance <= config_.viewpoint_same_height_tolerance_m;
      const int tier = same_height
                           ? (heading <= config_.preferred_heading_change_deg
                                  ? 0
                                  : 1)
                           : 2;
      candidates.push_back(
          {safe_viewpoint.representative,
           candidate,
           &frontier_cluster,
           known_free,
           candidate_distance,
           heading,
           observation_yaw,
           rotation_cost,
           tier});
    }
  }
  const bool has_same_height_candidate = std::any_of(
      candidates.begin(), candidates.end(),
      [](const Candidate &candidate) { return candidate.tier < 2; });
  if (has_same_height_candidate)
  {
    candidates.erase(
        std::remove_if(
            candidates.begin(), candidates.end(),
            [](const Candidate &candidate) { return candidate.tier == 2; }),
        candidates.end());
  }
  stats_.safe_viewpoint_candidates = candidates.size();

  std::vector<PvbsmExplorationHint> pvbsm_hints;
  std::size_t current_goal_hint_index =
      std::numeric_limits<std::size_t>::max();
  if (config_.pvbsm_scoring_enabled && pvbsm_batch_query_ &&
      !candidates.empty())
  {
    std::vector<PvbsmQueryPoint> query_points;
    query_points.reserve(candidates.size() + (had_goal ? 1U : 0U));
    for (const Candidate &candidate : candidates)
      query_points.push_back(
          {candidate.point.x, candidate.point.y, candidate.point.z});
    if (had_goal)
    {
      current_goal_hint_index = query_points.size();
      query_points.push_back(
          {decision_.position.x,
           decision_.position.y,
           decision_.position.z});
    }
    pvbsm_hints = pvbsm_batch_query_(query_points);
    if (pvbsm_hints.size() != query_points.size())
    {
      pvbsm_hints.clear();
      current_goal_hint_index = std::numeric_limits<std::size_t>::max();
    }
  }

  stats_.pvbsm_scored_candidates =
      pvbsm_hints.empty() ? 0U : candidates.size();
  stats_.pvbsm_unseen_candidates = 0;
  stats_.pvbsm_best_adjustment = 0.0;
  double best_score = -std::numeric_limits<double>::infinity();
  Vec3 best;
  double best_heading_change = 0.0;
  double best_observation_yaw = 0.0;
  bool found = false;
  const std::vector<VoxelKey> *best_cluster = nullptr;
  int selected_tier = -1;
  for (std::size_t index = 0; index < candidates.size(); ++index)
  {
    const Candidate &candidate = candidates[index];
    double pvbsm_adjustment = 0.0;
    if (!pvbsm_hints.empty())
    {
      pvbsm_adjustment = pvbsmScoreAdjustment(pvbsm_hints[index]);
      if (!pvbsm_hints[index].submap_observed)
        ++stats_.pvbsm_unseen_candidates;
    }
    const double score =
        frontierScore(candidate.voxel) +
        (degenerate_
             ? config_.degenerate_safe_path_weight * candidate.known_free
             : 0.0) +
        pvbsm_adjustment -
        config_.distance_cost_weight * candidate.distance -
        config_.heading_cost_weight * candidate.rotation_cost_deg / 180.0;
    ++stats_.candidates_scored;
    if (score > best_score)
    {
      best_score = score;
      best = candidate.point;
      best_heading_change = candidate.heading_change_deg;
      best_observation_yaw = candidate.observation_yaw;
      selected_tier = candidate.tier;
      best_cluster = candidate.cluster;
      found = true;
      stats_.pvbsm_best_adjustment = pvbsm_adjustment;
    }
  }
  const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start);
  stats_.last_plan_ms = elapsed.count();
  last_plan_time_ = timestamp;

  if (!found)
  {
    if (had_goal && !goal_reached_ && !failed_active_goal)
    {
      decision_.planning_time_ms = elapsed.count();
      return;
    }
    const bool changed = decision_.valid || decision_.state != "WAIT_FOR_FRONTIER";
    decision_.valid = false;
    selected_frontier_cluster_.clear();
    selected_cluster_generation_ = 0;
    decision_.updated = changed;
    decision_.state = "WAIT_FOR_FRONTIER";
    decision_.reason =
        goal_reached_
            ? "goal_reached_no_next_frontier"
            : goal_blocked_ ? "goal_blocked_no_safe_frontier"
            : goal_stalled_ ? "goal_stalled_no_safe_frontier"
            : goal_timeout_ ? "goal_timeout_no_safe_frontier"
                            : "no_safe_frontier";
    decision_.planning_time_ms = elapsed.count();
    blocked_streak_ = 0;
    goal_set_time_ = -1.0;
    resetGoalProgress(position, timestamp);
    return;
  }

  if (had_goal && !goal_reached_ && !failed_active_goal)
  {
    double current_known_free = 0.0;
    segmentBlocked(position, decision_.position, &current_known_free);
    const double current_distance = distance(position, decision_.position);
    const double current_heading = headingChange(position, decision_.position);
    const double current_travel_yaw = yawBetween(position, decision_.position);
    const double current_rotation_cost =
        current_heading + config_.arrival_yaw_cost_weight *
                              yawDifferenceDeg(current_travel_yaw,
                                               decision_.yaw);
    const double current_score =
        frontierScore(key(decision_.position, config_.planning_voxel_size_m)) +
        (degenerate_
             ? config_.degenerate_safe_path_weight * current_known_free
             : 0.0) +
        (current_goal_hint_index < pvbsm_hints.size()
             ? pvbsmScoreAdjustment(
                   pvbsm_hints[current_goal_hint_index])
             : 0.0) -
        config_.distance_cost_weight * current_distance -
        config_.heading_cost_weight * current_rotation_cost / 180.0;
    const double margin =
        degenerate_ ? config_.degenerate_goal_switch_margin
                    : config_.goal_switch_margin;
    if (best_score <= current_score +
                          margin * std::max(1.0, std::fabs(current_score)))
    {
      decision_.planning_time_ms = elapsed.count();
      return;
    }
  }

  decision_.valid = true;
  decision_.updated = true;
  decision_.position = best;
  decision_.yaw = best_observation_yaw;
  decision_.score = best_score;
  decision_.planning_time_ms = elapsed.count();
  decision_.constraint_tier = selected_tier;
  decision_.heading_change_deg = best_heading_change;
  decision_.state = degenerate_ ? "DEGRADED_EXPLORE" : "EXPLORE";
  if (!had_goal) decision_.reason = "initial_frontier";
  else if (goal_reached_) decision_.reason = "goal_reached";
  else if (goal_blocked_) decision_.reason = "new_obstacle";
  else if (goal_stalled_) decision_.reason = "goal_stalled";
  else if (goal_timeout_) decision_.reason = "goal_timeout";
  else decision_.reason = "better_frontier";
  goal_set_time_ = timestamp;
  blocked_streak_ = 0;
  ++decision_.generation;
  if (best_cluster)
    selected_frontier_cluster_ = *best_cluster;
  else
    selected_frontier_cluster_.clear();
  selected_cluster_generation_ = decision_.generation;
  last_goal_reachability_check_time_ = -1.0;
  cached_goal_reachable_ = true;
  resetGoalProgress(position, timestamp);
}

void ExplorerCore::updateGoalStatus(const Vec3 &position, double timestamp)
{
  ++stats_.goal_status_checks;
  const bool had_goal = decision_.valid;
  const double current_goal_distance =
      had_goal ? distance(decision_.position, position)
               : std::numeric_limits<double>::infinity();
  goal_reached_ =
      had_goal &&
      current_goal_distance <= config_.goal_reached_distance_m;

  if (!had_goal)
  {
    resetGoalProgress(position, timestamp);
  }
  else if (last_goal_progress_time_ < 0.0 ||
           timestamp < last_goal_progress_time_)
  {
    best_goal_distance_m_ = current_goal_distance;
    last_goal_progress_time_ = timestamp;
    goal_stalled_ = false;
  }
  else if (current_goal_distance <=
           best_goal_distance_m_ - config_.goal_progress_epsilon_m)
  {
    best_goal_distance_m_ = current_goal_distance;
    last_goal_progress_time_ = timestamp;
    goal_stalled_ = false;
  }

  const bool was_stalled = goal_stalled_;
  goal_stalled_ =
      had_goal && !goal_reached_ && config_.goal_timeout_s <= 0.0 &&
      last_goal_progress_time_ >= 0.0 &&
      timestamp - last_goal_progress_time_ >= config_.goal_stall_timeout_s;
  if (goal_stalled_ && !was_stalled) ++stats_.stalled_goals;

  const bool line_blocked =
      had_goal && segmentBlocked(position, decision_.position);
  if (!line_blocked)
  {
    cached_goal_reachable_ = true;
  }
  else if (last_goal_reachability_check_time_ < 0.0 ||
           timestamp < last_goal_reachability_check_time_ ||
           timestamp - last_goal_reachability_check_time_ + 1e-9 >=
               1.0 / config_.goal_reachability_check_rate_hz)
  {
    last_goal_reachability_check_time_ = timestamp;
    bool exhausted = false;
    ++stats_.reachability_checks;
    const int expansion_budget = std::max(
        32, static_cast<int>(std::lround(
                config_.reachability_max_expansions * stats_.budget_scale)));
    cached_goal_reachable_ = pathReachable(
        position, decision_.position, expansion_budget, &exhausted);
    if (exhausted) ++stats_.reachability_budget_exhaustions;
  }
  const bool raw_blocked = line_blocked && !cached_goal_reachable_;
  if (!had_goal || goal_reached_)
    blocked_streak_ = 0;
  else if (raw_blocked)
    blocked_streak_ =
        std::min(config_.goal_blocked_confirm_updates, blocked_streak_ + 1);
  else
    blocked_streak_ = 0;
  goal_blocked_ =
      raw_blocked &&
      blocked_streak_ >= config_.goal_blocked_confirm_updates;
  goal_timeout_ =
      had_goal && config_.goal_timeout_s > 0.0 && goal_set_time_ >= 0.0 &&
      timestamp - goal_set_time_ >= config_.goal_timeout_s;
}

bool ExplorerCore::isDue(double timestamp, double rate_hz, double &last_time)
{
  const double period = 1.0 / rate_hz;
  if (last_time < 0.0 || timestamp < last_time ||
      timestamp - last_time + 1e-9 >= period)
  {
    last_time = timestamp;
    return true;
  }
  return false;
}

void ExplorerCore::update(const Vec3 &position,
                          const Quaternion &orientation,
                          const std::vector<Vec3> &points, double timestamp)
{
  current_orientation_ = orientation;
  const auto start = std::chrono::steady_clock::now();
  ++update_id_;
  ++stats_.map_updates;
  integrateCloud(position, points);
  prune(position);
  updateGoalStatus(position, timestamp);

  if (isDue(timestamp, config_.frontier_update_rate_hz,
            last_frontier_update_time_))
  {
    updateFrontiers();
    ++stats_.frontier_update_cycles;
  }
  if (isDue(timestamp, config_.long_term_update_rate_hz,
            last_long_term_update_time_))
  {
    updateVisitMemory(position);
    ++stats_.long_term_update_cycles;
  }
  if (isDue(timestamp, config_.goal_evaluation_rate_hz,
            last_goal_evaluation_time_))
  {
    updateDecision(position, timestamp);
    ++stats_.goal_evaluation_cycles;
  }
  stats_.last_update_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start)
          .count();
}

bool ExplorerCore::consumeDecision(GoalDecision &decision)
{
  if (!decision_.updated) return false;
  decision = decision_;
  decision_.updated = false;
  return true;
}

std::vector<Vec3> ExplorerCore::frontierPoints(std::size_t limit) const
{
  std::vector<Vec3> result;
  result.reserve(std::min(limit, frontiers_.size()));
  for (const VoxelKey &voxel : frontiers_)
  {
    if (result.size() >= limit) break;
    result.push_back(center(voxel, config_.planning_voxel_size_m));
  }
  return result;
}

std::vector<Vec3> ExplorerCore::selectedFrontierPoints() const
{
  std::vector<Vec3> result;
  result.reserve(selected_frontier_cluster_.size());
  for (const VoxelKey &voxel : selected_frontier_cluster_)
    result.push_back(center(voxel, config_.planning_voxel_size_m));
  return result;
}

std::vector<Vec3> ExplorerCore::validClusterFrontierPoints() const
{
  std::vector<Vec3> result;
  result.reserve(valid_cluster_frontiers_.size());
  for (const VoxelKey &voxel : valid_cluster_frontiers_)
    result.push_back(center(voxel, config_.planning_voxel_size_m));
  return result;
}

std::vector<Vec3> ExplorerCore::occupiedPoints(
    const Vec3 &position, double radius, std::size_t limit) const
{
  std::vector<Vec3> result;
  result.reserve(std::min(limit, stats_.occupied_cells));
  for (const auto &entry : map_)
  {
    if (entry.second.log_odds >= 2)
    {
      const Vec3 point = center(entry.first, config_.planning_voxel_size_m);
      if (distance(point, position) <= radius)
        result.push_back(point);
    }
  }
  if (result.size() > limit)
  {
    const auto squared_distance = [&position](const Vec3 &point)
    {
      const double dx = point.x - position.x;
      const double dy = point.y - position.y;
      const double dz = point.z - position.z;
      return dx * dx + dy * dy + dz * dz;
    };
    std::nth_element(
        result.begin(), result.begin() + limit, result.end(),
        [&squared_distance](const Vec3 &left, const Vec3 &right)
        {
          return squared_distance(left) < squared_distance(right);
        });
    result.resize(limit);
  }
  return result;
}

}  // namespace daib_explorer
