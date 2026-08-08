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
constexpr int kNeighbors[6][3] = {
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
    {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

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
  config_.replan_interval_s = std::max(0.1, config_.replan_interval_s);
  config_.goal_timeout_s =
      std::max(config_.replan_interval_s, config_.goal_timeout_s);
  config_.goal_min_hold_time_s =
      clamp(config_.goal_min_hold_time_s, 0.0, config_.goal_timeout_s);
  config_.goal_blocked_confirm_updates =
      std::max(1, config_.goal_blocked_confirm_updates);
  config_.same_goal_tolerance_m = std::max(0.0, config_.same_goal_tolerance_m);
  config_.goal_reached_distance_m =
      std::max(0.1, config_.goal_reached_distance_m);
  config_.min_goal_distance_m =
      std::max(config_.goal_reached_distance_m, config_.min_goal_distance_m);
  config_.max_goal_distance_m =
      std::max(config_.min_goal_distance_m, config_.max_goal_distance_m);
  config_.max_goal_vertical_distance_m =
      std::max(config_.planning_voxel_size_m,
               config_.max_goal_vertical_distance_m);
  config_.goal_switch_margin = clamp(config_.goal_switch_margin, 0.0, 1.0);
  if (config_.scene_mode != "indoor" && config_.scene_mode != "outdoor")
    config_.scene_mode = "indoor";
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
  config_.min_wall_clearance_m =
      std::max(0.0, config_.min_wall_clearance_m);
  config_.max_safe_viewpoint_candidates =
      std::max(1, config_.max_safe_viewpoint_candidates);
  config_.preferred_min_goal_distance_m =
      clamp(config_.preferred_min_goal_distance_m,
            config_.min_goal_distance_m, config_.max_goal_distance_m);
  config_.preferred_heading_change_deg =
      clamp(config_.preferred_heading_change_deg, 0.0, 180.0);
  config_.fallback_heading_change_deg =
      clamp(config_.fallback_heading_change_deg,
            config_.preferred_heading_change_deg, 180.0);
  config_.indoor_max_vertical_distance_m =
      std::max(config_.planning_voxel_size_m,
               config_.indoor_max_vertical_distance_m);
  config_.outdoor_max_vertical_distance_m =
      std::max(config_.planning_voxel_size_m,
               config_.outdoor_max_vertical_distance_m);
  config_.indoor_max_climb_angle_deg =
      clamp(config_.indoor_max_climb_angle_deg, 0.0, 89.0);
  config_.outdoor_max_climb_angle_deg =
      clamp(config_.outdoor_max_climb_angle_deg, 0.0, 89.0);
  config_.reachability_max_expansions =
      std::max(32, config_.reachability_max_expansions);
  config_.max_reachability_checks_per_cycle =
      std::max(1, config_.max_reachability_checks_per_cycle);
  config_.goal_reachability_check_rate_hz =
      std::max(0.1, config_.goal_reachability_check_rate_hz);
  config_.loop_history_window_s =
      std::max(1.0, config_.loop_history_window_s);
  config_.loop_repeat_threshold = std::max(2, config_.loop_repeat_threshold);
  config_.loop_cluster_radius_m =
      std::max(config_.planning_voxel_size_m, config_.loop_cluster_radius_m);
  config_.loop_max_displacement_m =
      std::max(config_.planning_voxel_size_m,
               config_.loop_max_displacement_m);
  config_.loop_escape_duration_s =
      std::max(1.0, config_.loop_escape_duration_s);
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

void ExplorerCore::markFrontierDirty(const VoxelKey &voxel)
{
  dirty_frontiers_.insert(voxel);
  for (const auto &offset : kNeighbors)
  {
    dirty_frontiers_.insert(
        {voxel.x + offset[0], voxel.y + offset[1], voxel.z + offset[2]});
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
  const int ray_budget = std::max(
      1, static_cast<int>(std::lround(
             config_.max_raycasts_per_update * stats_.budget_scale)));
  stats_.effective_raycasts = ray_budget;
  if (points.empty()) return;

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
  }
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
    bool frontier = cellState(voxel) == 0;
    if (frontier)
    {
      frontier = false;
      for (const auto &offset : kNeighbors)
      {
        const VoxelKey neighbor{
            voxel.x + offset[0], voxel.y + offset[1], voxel.z + offset[2]};
        if (cellState(neighbor) < 0)
        {
          frontier = true;
          break;
        }
      }
    }
    if (frontier) frontiers_.insert(voxel);
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
    for (const auto &offset : kNeighbors)
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

std::vector<std::vector<VoxelKey>> ExplorerCore::frontierClusters() const
{
  std::vector<std::vector<VoxelKey>> clusters;
  std::unordered_map<VoxelKey, std::vector<VoxelKey>, VoxelKeyHash> buckets;
  for (const VoxelKey &voxel : frontiers_)
  {
    const Vec3 point = center(voxel, config_.planning_voxel_size_m);
    buckets[key(point, config_.frontier_cluster_size_m)].push_back(voxel);
  }
  clusters.reserve(buckets.size());
  for (auto &entry : buckets)
  {
    if (entry.second.size() >=
        static_cast<std::size_t>(config_.min_frontier_cluster_cells))
      clusters.push_back(std::move(entry.second));
  }
  return clusters;
}

bool ExplorerCore::makeSafeViewpoint(const std::vector<VoxelKey> &cluster,
                                     Vec3 &viewpoint,
                                     VoxelKey &representative) const
{
  if (cluster.empty()) return false;
  Vec3 centroid;
  Vec3 unknown_direction;
  for (const VoxelKey &voxel : cluster)
  {
    const Vec3 point = center(voxel, config_.planning_voxel_size_m);
    centroid.x += point.x;
    centroid.y += point.y;
    centroid.z += point.z;
    for (const auto &offset : kNeighbors)
    {
      if (cellState({voxel.x + offset[0], voxel.y + offset[1],
                     voxel.z + offset[2]}) < 0)
      {
        unknown_direction.x += offset[0];
        unknown_direction.y += offset[1];
        unknown_direction.z += offset[2];
      }
    }
  }
  const double inverse_size = 1.0 / cluster.size();
  centroid.x *= inverse_size;
  centroid.y *= inverse_size;
  centroid.z *= inverse_size;
  representative = cluster.front();
  double representative_distance = std::numeric_limits<double>::infinity();
  for (const VoxelKey &voxel : cluster)
  {
    const double candidate_distance =
        distance(center(voxel, config_.planning_voxel_size_m), centroid);
    if (candidate_distance < representative_distance)
    {
      representative_distance = candidate_distance;
      representative = voxel;
    }
  }

  const double direction_norm = std::sqrt(
      unknown_direction.x * unknown_direction.x +
      unknown_direction.y * unknown_direction.y +
      unknown_direction.z * unknown_direction.z);
  Vec3 desired = centroid;
  if (direction_norm >= 1e-6)
  {
    desired.x -= config_.viewpoint_standoff_m *
                 unknown_direction.x / direction_norm;
    desired.y -= config_.viewpoint_standoff_m *
                 unknown_direction.y / direction_norm;
    desired.z -= config_.viewpoint_standoff_m *
                 unknown_direction.z / direction_norm;
  }
  const VoxelKey desired_key = key(desired, config_.planning_voxel_size_m);
  const int search_radius = static_cast<int>(std::ceil(
      config_.viewpoint_search_radius_m / config_.planning_voxel_size_m));
  for (int shell = 0; shell <= search_radius; ++shell)
  {
    for (int dx = -shell; dx <= shell; ++dx)
    {
      for (int dy = -shell; dy <= shell; ++dy)
      {
        for (int dz = -shell; dz <= shell; ++dz)
        {
          if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != shell)
            continue;
          const VoxelKey candidate_key{desired_key.x + dx,
                                       desired_key.y + dy,
                                       desired_key.z + dz};
          if (cellState(candidate_key) != 0 ||
              !hasWallClearance(candidate_key))
            continue;
          const Vec3 candidate =
              center(candidate_key, config_.planning_voxel_size_m);
          if (segmentBlocked(candidate, centroid)) continue;
          viewpoint = candidate;
          return true;
        }
      }
    }
  }
  return false;
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

bool ExplorerCore::loopDetected(const Vec3 &position, double timestamp)
{
  while (!goal_history_.empty() &&
         timestamp - goal_history_.front().timestamp >
             config_.loop_history_window_s)
    goal_history_.pop_front();
  if (!config_.loop_escape_enabled ||
      goal_history_.size() <
          static_cast<std::size_t>(config_.loop_repeat_threshold))
    return false;
  struct ClusterHistory
  {
    int count = 0;
    Vec3 first_vehicle_position;
  };
  std::unordered_map<VoxelKey, ClusterHistory, VoxelKeyHash> counts;
  for (const GoalHistoryEntry &entry : goal_history_)
  {
    ClusterHistory &history = counts[entry.cluster];
    if (history.count == 0)
      history.first_vehicle_position = entry.vehicle_position;
    ++history.count;
  }
  for (const auto &entry : counts)
  {
    if (entry.second.count >= config_.loop_repeat_threshold &&
        distance(position, entry.second.first_vehicle_position) <=
            config_.loop_max_displacement_m)
      return true;
  }
  return false;
}

bool ExplorerCore::recentlySelectedCluster(const VoxelKey &cluster) const
{
  for (const GoalHistoryEntry &entry : goal_history_)
  {
    if (entry.cluster == cluster) return true;
  }
  return false;
}

void ExplorerCore::rememberSelectedCluster(const VoxelKey &cluster,
                                           const Vec3 &position,
                                           double timestamp)
{
  goal_history_.push_back({cluster, position, timestamp});
}

double ExplorerCore::frontierScore(const VoxelKey &voxel,
                                   const Vec3 &position) const
{
  int unknown_neighbors = 0;
  int occupied_neighbors = 0;
  for (const auto &offset : kNeighbors)
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
         0.35 * weakness * occupied_neighbors -
         0.25 * distance(point, position);
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
  const bool had_goal = decision_.valid;
  const bool hold =
      had_goal && goal_set_time_ >= 0.0 &&
      timestamp - goal_set_time_ < config_.goal_min_hold_time_s;
  if (hold && !goal_reached_ && !goal_blocked_) return;
  const bool periodic =
      last_plan_time_ < 0.0 ||
      timestamp - last_plan_time_ >= config_.replan_interval_s;
  if (had_goal && !goal_reached_ && !goal_blocked_ && !goal_timeout_ &&
      !periodic)
    return;

  const auto start = std::chrono::steady_clock::now();
  const int budget = std::max(
      1, static_cast<int>(std::lround(
             config_.frontier_evaluation_budget * stats_.budget_scale)));
  stats_.effective_frontier_evaluations = budget;
  const double max_distance =
      degenerate_ ? config_.degenerate_max_goal_distance_m
                  : config_.max_goal_distance_m;
  const double max_vertical = std::min(
      config_.max_goal_vertical_distance_m,
      config_.scene_mode == "outdoor"
          ? config_.outdoor_max_vertical_distance_m
          : config_.indoor_max_vertical_distance_m);
  const double max_climb_angle =
      config_.scene_mode == "outdoor"
          ? config_.outdoor_max_climb_angle_deg
          : config_.indoor_max_climb_angle_deg;
  const bool detected_loop = loopDetected(position, timestamp);
  if (detected_loop && timestamp >= loop_escape_until_)
  {
    loop_escape_until_ = timestamp + config_.loop_escape_duration_s;
    ++stats_.loop_escape_activations;
  }
  const bool escape_active = timestamp < loop_escape_until_;
  stats_.loop_escape_active = escape_active;
  struct Candidate
  {
    VoxelKey voxel;
    VoxelKey cluster;
    Vec3 point;
    double known_free = 0.0;
    double distance = 0.0;
    double heading_change_deg = 0.0;
    int tier = 0;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(static_cast<std::size_t>(budget));
  const std::vector<std::vector<VoxelKey>> clusters = frontierClusters();
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
            : approximate_heading <= config_.fallback_heading_change_deg
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
  int reachability_checks = 0;
  int evaluated = 0;
  const int viewpoint_budget = std::max(
      1, static_cast<int>(std::lround(
             config_.max_safe_viewpoint_candidates * stats_.budget_scale)));
  for (const ClusterOrder &ordered_cluster : ordered_clusters)
  {
    if (evaluated++ >= budget) break;
    if (static_cast<int>(candidates.size()) >= viewpoint_budget) break;
    const std::vector<VoxelKey> &frontier_cluster = *ordered_cluster.cluster;
    Vec3 candidate;
    VoxelKey representative;
    if (!makeSafeViewpoint(frontier_cluster, candidate, representative))
      continue;
    const double candidate_distance = distance(candidate, position);
    const double horizontal_distance = std::hypot(
        candidate.x - position.x, candidate.y - position.y);
    const double climb_angle = std::atan2(
        std::fabs(candidate.z - position.z),
        std::max(config_.planning_voxel_size_m, horizontal_distance)) *
        180.0 / 3.14159265358979323846;
    if (candidate_distance < config_.min_goal_distance_m ||
        candidate_distance > max_distance ||
        std::fabs(candidate.z - position.z) > max_vertical ||
        climb_angle > max_climb_angle)
      continue;
    double known_free = 0.0;
    const bool direct_blocked =
        segmentBlocked(position, candidate, &known_free);
    if (direct_blocked)
    {
      if (!config_.reachability_enabled ||
          reachability_checks >= config_.max_reachability_checks_per_cycle)
        continue;
      bool exhausted = false;
      ++reachability_checks;
      ++stats_.reachability_checks;
      const int expansion_budget = std::max(
          32, static_cast<int>(std::lround(
                  config_.reachability_max_expansions * stats_.budget_scale)));
      if (!pathReachable(position, candidate, expansion_budget, &exhausted))
      {
        if (exhausted) ++stats_.reachability_budget_exhaustions;
        continue;
      }
    }
    const double heading = headingChange(position, candidate);
    int tier = 3;
    if (candidate_distance >= config_.preferred_min_goal_distance_m &&
        heading <= config_.preferred_heading_change_deg)
      tier = 0;
    else if (heading <= config_.preferred_heading_change_deg)
      tier = 1;
    else if (heading <= config_.fallback_heading_change_deg)
      tier = 2;
    else if (!escape_active)
      continue;
    const Vec3 representative_point =
        center(representative, config_.planning_voxel_size_m);
    candidates.push_back(
        {representative,
         key(representative_point, config_.loop_cluster_radius_m),
         candidate,
         known_free,
         candidate_distance,
         heading,
         tier});
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
  VoxelKey best_cluster;
  double best_heading_change = 0.0;
  bool found = false;
  int selected_tier = 4;
  bool have_fresh_cluster = false;
  if (escape_active)
  {
    for (const Candidate &candidate : candidates)
    {
      if (!recentlySelectedCluster(candidate.cluster))
      {
        have_fresh_cluster = true;
        break;
      }
    }
  }
  for (const Candidate &candidate : candidates)
  {
    if (escape_active && have_fresh_cluster &&
        recentlySelectedCluster(candidate.cluster))
      continue;
    selected_tier = std::min(selected_tier, candidate.tier);
  }
  for (std::size_t index = 0; index < candidates.size(); ++index)
  {
    const Candidate &candidate = candidates[index];
    if (candidate.tier != selected_tier) continue;
    if (escape_active && have_fresh_cluster &&
        recentlySelectedCluster(candidate.cluster))
      continue;
    double pvbsm_adjustment = 0.0;
    if (!pvbsm_hints.empty())
    {
      pvbsm_adjustment = pvbsmScoreAdjustment(pvbsm_hints[index]);
      if (!pvbsm_hints[index].submap_observed)
        ++stats_.pvbsm_unseen_candidates;
    }
    const double score =
        frontierScore(candidate.voxel, position) +
        (degenerate_
             ? config_.degenerate_safe_path_weight * candidate.known_free
             : 0.0) +
        pvbsm_adjustment;
    if (score > best_score)
    {
      best_score = score;
      best = candidate.point;
      best_cluster = candidate.cluster;
      best_heading_change = candidate.heading_change_deg;
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
    if (had_goal && !goal_reached_ && !goal_blocked_ && !goal_timeout_)
    {
      decision_.planning_time_ms = elapsed.count();
      return;
    }
    const bool changed = decision_.valid || decision_.state != "WAIT_FOR_FRONTIER";
    decision_.valid = false;
    decision_.updated = changed;
    decision_.state = "WAIT_FOR_FRONTIER";
    decision_.reason =
        goal_reached_
            ? "goal_reached_no_next_frontier"
            : goal_blocked_ ? "goal_blocked_no_safe_frontier"
                            : "no_safe_frontier";
    decision_.planning_time_ms = elapsed.count();
    blocked_streak_ = 0;
    return;
  }

  if (had_goal && goal_timeout_ && !goal_reached_ && !goal_blocked_ &&
      distance(best, decision_.position) <= config_.same_goal_tolerance_m)
  {
    goal_set_time_ = timestamp;
    decision_.score = best_score;
    decision_.planning_time_ms = elapsed.count();
    decision_.reason = "goal_timeout_same_goal";
    ++stats_.suppressed_goal_republishes;
    // A suppressed publication is still a repeated planning choice. Keep it
    // in the loop detector so a vehicle stuck on one unchanged frontier can
    // enter escape mode instead of refreshing that goal forever.
    rememberSelectedCluster(best_cluster, position, timestamp);
    return;
  }

  if (had_goal && !goal_reached_ && !goal_blocked_ && !goal_timeout_)
  {
    double current_known_free = 0.0;
    segmentBlocked(position, decision_.position, &current_known_free);
    const double current_score =
        frontierScore(key(decision_.position, config_.planning_voxel_size_m),
                      position) +
        (degenerate_
             ? config_.degenerate_safe_path_weight * current_known_free
             : 0.0) +
        (current_goal_hint_index < pvbsm_hints.size()
             ? pvbsmScoreAdjustment(
                   pvbsm_hints[current_goal_hint_index])
             : 0.0);
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
  decision_.yaw = std::atan2(best.y - position.y, best.x - position.x);
  decision_.score = best_score;
  decision_.planning_time_ms = elapsed.count();
  decision_.constraint_tier = selected_tier;
  decision_.heading_change_deg = best_heading_change;
  decision_.state = degenerate_ ? "DEGRADED_EXPLORE" : "EXPLORE";
  if (escape_active) decision_.reason = "loop_escape";
  else if (!had_goal) decision_.reason = "initial_frontier";
  else if (goal_reached_) decision_.reason = "goal_reached";
  else if (goal_blocked_) decision_.reason = "new_obstacle";
  else if (goal_timeout_) decision_.reason = "goal_timeout";
  else decision_.reason = "better_frontier";
  goal_set_time_ = timestamp;
  blocked_streak_ = 0;
  ++decision_.generation;
  rememberSelectedCluster(best_cluster, position, timestamp);
}

void ExplorerCore::updateGoalStatus(const Vec3 &position, double timestamp)
{
  ++stats_.goal_status_checks;
  const bool had_goal = decision_.valid;
  goal_reached_ =
      had_goal &&
      distance(decision_.position, position) <=
          config_.goal_reached_distance_m;
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
      had_goal && goal_set_time_ >= 0.0 &&
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
