#include "daib_explorer/explorer_core.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace daib_explorer
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
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
  config_.max_coverage_points_per_update =
      std::max(1, config_.max_coverage_points_per_update);
  config_.submap_translation_threshold_m =
      std::max(1.0, config_.submap_translation_threshold_m);
  config_.submap_rotation_threshold_deg =
      std::max(1.0, config_.submap_rotation_threshold_deg);
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
  updateCell(key(origin, config_.planning_voxel_size_m), -1);
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

void ExplorerCore::updateSubmap(const Vec3 &position,
                                const Quaternion &orientation,
                                const std::vector<Vec3> &points)
{
  const VoxelKey coverage_key = key(position, config_.coverage_voxel_size_m);
  ++visits_[coverage_key];
  stats_.visited_cells = visits_.size();

  const double quaternion_norm =
      std::sqrt(orientation.x * orientation.x +
                orientation.y * orientation.y +
                orientation.z * orientation.z +
                orientation.w * orientation.w);
  const Quaternion normalized =
      quaternion_norm > 1e-12
          ? Quaternion{orientation.x / quaternion_norm,
                       orientation.y / quaternion_norm,
                       orientation.z / quaternion_norm,
                       orientation.w / quaternion_norm}
          : Quaternion{};
  bool create = submaps_.empty();
  if (!create)
  {
    const SubmapSummary &active = submaps_.back();
    const Quaternion &anchor = active.anchor_orientation;
    const double dot = std::fabs(
        normalized.x * anchor.x + normalized.y * anchor.y +
        normalized.z * anchor.z + normalized.w * anchor.w);
    const double rotation_delta =
        2.0 * std::acos(clamp(dot, 0.0, 1.0)) * 180.0 / kPi;
    create =
        distance(position, active.anchor) >=
            config_.submap_translation_threshold_m ||
        rotation_delta >= config_.submap_rotation_threshold_deg;
  }
  if (create)
  {
    SubmapSummary summary;
    summary.id = (static_cast<uint64_t>(config_.robot_id) << 48U) |
                 static_cast<uint64_t>(submaps_.size() + 1);
    summary.start_update = update_id_;
    summary.end_update = update_id_;
    summary.anchor = position;
    summary.anchor_orientation = normalized;
    summary.min_bound = position;
    summary.max_bound = position;
    submaps_.push_back(summary);
    active_submap_cells_.clear();
  }

  SubmapSummary &active = submaps_.back();
  active.end_update = update_id_;
  active.min_bound.x = std::min(active.min_bound.x, position.x);
  active.min_bound.y = std::min(active.min_bound.y, position.y);
  active.min_bound.z = std::min(active.min_bound.z, position.z);
  active.max_bound.x = std::max(active.max_bound.x, position.x);
  active.max_bound.y = std::max(active.max_bound.y, position.y);
  active.max_bound.z = std::max(active.max_bound.z, position.z);
  bool coverage_changed = active_submap_cells_.insert(coverage_key).second;
  if (!points.empty())
  {
    const std::size_t sample_budget =
        static_cast<std::size_t>(config_.max_coverage_points_per_update);
    const std::size_t stride = std::max<std::size_t>(
        1, (points.size() + sample_budget - 1) / sample_budget);
    std::size_t sampled = 0;
    for (std::size_t index = 0;
         index < points.size() && sampled < sample_budget;
         index += stride, ++sampled)
    {
      const VoxelKey observed_key =
          key(points[index], config_.coverage_voxel_size_m);
      ++observations_[observed_key];
      coverage_changed =
          active_submap_cells_.insert(observed_key).second || coverage_changed;
    }
  }
  stats_.observed_cells = observations_.size();
  if (coverage_changed)
  {
    active.covered_cells = active_submap_cells_.size();
    ++active.version;
  }
  stats_.submap_count = submaps_.size();
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
  double best_score = -std::numeric_limits<double>::infinity();
  Vec3 best;
  bool found = false;
  int evaluated = 0;
  for (const VoxelKey &voxel : frontiers_)
  {
    if (evaluated++ >= budget) break;
    if (cellState(voxel) != 0) continue;
    const Vec3 candidate = center(voxel, config_.planning_voxel_size_m);
    const double candidate_distance = distance(candidate, position);
    if (candidate_distance < config_.min_goal_distance_m ||
        candidate_distance > max_distance ||
        std::fabs(candidate.z - position.z) >
            config_.max_goal_vertical_distance_m)
      continue;
    double known_free = 0.0;
    if (segmentBlocked(position, candidate, &known_free)) continue;
    const double score =
        frontierScore(voxel, position) +
        (degenerate_ ? config_.degenerate_safe_path_weight * known_free : 0.0);
    if (score > best_score)
    {
      best_score = score;
      best = candidate;
      found = true;
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
  decision_.state = degenerate_ ? "DEGRADED_EXPLORE" : "EXPLORE";
  if (!had_goal) decision_.reason = "initial_frontier";
  else if (goal_reached_) decision_.reason = "goal_reached";
  else if (goal_blocked_) decision_.reason = "new_obstacle";
  else if (goal_timeout_) decision_.reason = "goal_timeout";
  else decision_.reason = "better_frontier";
  goal_set_time_ = timestamp;
  blocked_streak_ = 0;
  ++decision_.generation;
}

void ExplorerCore::updateGoalStatus(const Vec3 &position, double timestamp)
{
  ++stats_.goal_status_checks;
  const bool had_goal = decision_.valid;
  goal_reached_ =
      had_goal &&
      distance(decision_.position, position) <=
          config_.goal_reached_distance_m;
  const bool raw_blocked =
      had_goal && segmentBlocked(position, decision_.position);
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
    updateSubmap(position, orientation, points);
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
