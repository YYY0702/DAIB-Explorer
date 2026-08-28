#include "daib_explorer/explorer_core.h"
#include "daib_explorer/pvbsm_memory.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <daib_explorer/CoExploreTask.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt64MultiArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <daib_decision/DaibActionAck.h>
#include <daib_decision/DaibDecisionCommand.h>
#include <daib_decision/DaibEvent.h>
#include <daib_decision/DaibModuleStatus.h>

namespace daib_explorer
{

class ExplorerNode
{
public:
  ExplorerNode() : private_nh_("~")
  {
    ExplorerConfig config;
    readParameters(config);
    core_ = std::make_unique<ExplorerCore>(config);
    robot_id_ = static_cast<uint16_t>(config.robot_id);
    if (pvbsm_memory_enabled_)
      pvbsm_memory_ = std::make_unique<PvbsmMemory>(
          static_cast<std::size_t>(pvbsm_memory_max_records_));
    if (pvbsm_memory_)
    {
      pvbsm_query_source_id_ = static_cast<uint16_t>(
          std::max(0, std::min(65535, config.robot_id)));
      const double root_voxel_size =
          std::max(0.1, config.pvbsm_root_voxel_size_m);
      pvbsm_expected_submap_edge_roots_ = static_cast<uint8_t>(
          std::max(1, std::min(255, config.pvbsm_submap_edge_roots)));
      const std::size_t covered_root_target = static_cast<std::size_t>(
          std::max(1, config.pvbsm_covered_root_target));
      core_->setPvbsmBatchQuery(
          [this, root_voxel_size,
           covered_root_target](
              const std::vector<PvbsmQueryPoint> &points)
          {
            return queryFusedPvbsmHints(
                points, root_voxel_size, covered_root_target);
          });
    }

    odom_sub_ = nh_.subscribe(odom_topic_, 5, &ExplorerNode::odomCallback, this);
    cloud_sub_ =
        nh_.subscribe(cloud_topic_, 1, &ExplorerNode::cloudCallback, this);
    degenerate_sub_ = nh_.subscribe(
        degenerate_topic_, 1, &ExplorerNode::degenerateCallback, this);
    score_sub_ =
        nh_.subscribe(score_topic_, 1, &ExplorerNode::scoreCallback, this);
    runtime_sub_ =
        nh_.subscribe(runtime_topic_, 1, &ExplorerNode::runtimeCallback, this);
    if (pvbsm_memory_)
    {
      pvbsm_sub_ = nh_.subscribe(
          pvbsm_topic_, 2, &ExplorerNode::pvbsmCallback, this);
      if (cooperation_enabled_ && !peer_pvbsm_topic_.empty())
        peer_pvbsm_sub_ = nh_.subscribe(
            peer_pvbsm_topic_, 2, &ExplorerNode::pvbsmCallback, this);
    }
    decision_command_sub_ = nh_.subscribe(
        decision_command_topic_, 10,
        &ExplorerNode::decisionCommandCallback, this);
    if (cooperation_enabled_)
      coexplore_task_sub_ = nh_.subscribe(
          coexplore_task_topic_, 20,
          &ExplorerNode::coexploreTaskCallback, this);

    goal_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1, true);
    frontiers_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(frontiers_topic_, 1, true);
    valid_cluster_frontiers_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(
            valid_cluster_frontiers_topic_, 1, true);
    selected_cluster_frontiers_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(
            selected_cluster_frontiers_topic_, 1, true);
    planning_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(planning_cloud_topic_, 1);
    ready_pub_ = nh_.advertise<std_msgs::Bool>(ready_topic_, 1, true);
    state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 1, true);
    generation_pub_ =
        nh_.advertise<std_msgs::UInt64>(generation_topic_, 1, true);
    decision_status_pub_ =
        nh_.advertise<daib_decision::DaibModuleStatus>(
            decision_status_topic_, 10);
    decision_event_pub_ = nh_.advertise<daib_decision::DaibEvent>(
        decision_event_topic_, 10);
    decision_ack_pub_ = nh_.advertise<daib_decision::DaibActionAck>(
        decision_ack_topic_, 10);
    if (cooperation_enabled_)
      coexplore_task_pub_ = nh_.advertise<daib_explorer::CoExploreTask>(
          coexplore_task_topic_, 10);
    if (pvbsm_memory_)
      pvbsm_stats_pub_ = nh_.advertise<std_msgs::UInt64MultiArray>(
          pvbsm_stats_topic_, 1, true);

    map_timer_ = nh_.createTimer(
        ros::Duration(1.0 / map_update_rate_hz_),
        &ExplorerNode::mapTimerCallback, this);
    publishReady(false);
    ROS_INFO_STREAM("[ DAIB Explorer ] isolated node ready; odom="
                    << odom_topic_ << ", cloud=" << cloud_topic_
                    << ", map_update_rate=" << map_update_rate_hz_ << " Hz"
                    << ", pvbsm_memory="
                    << (pvbsm_memory_ ? "enabled" : "disabled")
                    << ", cooperation="
                    << (cooperation_enabled_ ? "enabled" : "disabled")
                    << ", legacy_submap_memory=disabled");
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber degenerate_sub_;
  ros::Subscriber score_sub_;
  ros::Subscriber runtime_sub_;
  ros::Subscriber pvbsm_sub_;
  ros::Subscriber peer_pvbsm_sub_;
  ros::Subscriber decision_command_sub_;
  ros::Subscriber coexplore_task_sub_;
  ros::Publisher goal_pub_;
  ros::Publisher frontiers_pub_;
  ros::Publisher valid_cluster_frontiers_pub_;
  ros::Publisher selected_cluster_frontiers_pub_;
  ros::Publisher planning_cloud_pub_;
  ros::Publisher ready_pub_;
  ros::Publisher state_pub_;
  ros::Publisher generation_pub_;
  ros::Publisher decision_status_pub_;
  ros::Publisher decision_event_pub_;
  ros::Publisher decision_ack_pub_;
  ros::Publisher coexplore_task_pub_;
  ros::Publisher pvbsm_stats_pub_;
  ros::Timer map_timer_;
  std::unique_ptr<ExplorerCore> core_;
  std::unique_ptr<PvbsmMemory> pvbsm_memory_;

  std::mutex input_mutex_;
  std::mutex update_mutex_;
  std::mutex pvbsm_mutex_;
  std::mutex peer_mutex_;
  tf::TransformListener tf_listener_;
  nav_msgs::OdometryConstPtr latest_odom_;
  sensor_msgs::PointCloud2ConstPtr latest_cloud_;
  ros::WallTime last_odom_receive_;
  ros::WallTime last_cloud_receive_;
  ros::WallTime last_ready_publish_;
  uint64_t cloud_sequence_ = 0;
  uint64_t processed_cloud_sequence_ = 0;
  bool degenerate_ = true;
  double degeneracy_score_ = 0.0;
  double lio_runtime_ms_ = -1.0;

  std::string odom_topic_ = "/daib_slam/odom";
  std::string cloud_topic_ = "/daib_slam/planning_cloud";
  std::string degenerate_topic_ = "/daib_slam/degenerate";
  std::string score_topic_ = "/daib_slam/degeneracy_score";
  std::string runtime_topic_ = "/daib_slam/lio_runtime_ms";
  std::string pvbsm_topic_ = "/daib_slam/pvbsm_delta";
  std::string goal_topic_ = "/daib_explorer/goal";
  std::string frontiers_topic_ = "/daib_explorer/frontiers";
  std::string valid_cluster_frontiers_topic_ =
      "/daib_explorer/valid_cluster_frontiers";
  std::string selected_cluster_frontiers_topic_ =
      "/daib_explorer/selected_cluster_frontiers";
  std::string planning_cloud_topic_ = "/daib_explorer/planning_cloud";
  std::string ready_topic_ = "/daib_explorer/ready";
  std::string state_topic_ = "/daib_explorer/state";
  std::string generation_topic_ = "/daib_explorer/generation";
  std::string pvbsm_stats_topic_ = "/daib_explorer/pvbsm_memory_stats";
  std::string decision_status_topic_ = "/daib_decision/module_status";
  std::string decision_event_topic_ = "/daib_decision/event";
  std::string decision_command_topic_ = "/daib_decision/command";
  std::string decision_ack_topic_ = "/daib_decision/action_ack";
  std::string coexplore_task_topic_ = "/daib_coexplore/task";
  double map_update_rate_hz_ = 10.0;
  double input_timeout_s_ = 1.0;
  double ready_heartbeat_rate_hz_ = 1.0;
  double max_input_stamp_skew_s_ = 0.2;
  int max_cloud_points_to_convert_ = 6000;
  int max_published_frontiers_ = 2000;
  double planning_output_radius_m_ = 12.0;
  int max_published_planning_points_ = 6000;
  bool pvbsm_memory_enabled_ = true;
  int pvbsm_memory_max_records_ = 200000;
  int max_pvbsm_records_per_delta_ = 2048;
  uint16_t pvbsm_query_source_id_ = 0;
  uint8_t pvbsm_expected_submap_edge_roots_ = 8;
  bool ready_ = false;
  bool ready_initialized_ = false;
  uint64_t decision_source_session_ = ros::WallTime::now().toNSec();
  uint64_t decision_status_sequence_ = 0;
  uint64_t decision_event_sequence_ = 0;
  uint64_t latest_decision_session_ = 0;
  uint64_t latest_decision_command_id_ = 0;
  uint64_t published_generation_ = 0;
  uint16_t robot_id_ = 0;
  bool cooperation_enabled_ = false;
  int expected_peer_count_ = 1;
  double peer_goal_ttl_s_ = 3.0;
  double peer_goal_exclusion_radius_m_ = 2.0;
  double peer_transform_timeout_s_ = 0.05;
  double peer_max_future_skew_s_ = 0.5;
  uint64_t coexplore_session_ = ros::WallTime::now().toNSec();
  uint64_t coexplore_sequence_ = 0;
  ros::WallTime last_coexplore_publish_;
  std::string planning_frame_;
  GoalDecision leased_goal_;
  bool leased_goal_active_ = false;
  struct PeerTaskState
  {
    PeerGoalLease lease;
    ros::WallTime last_receive;
  };
  std::unordered_map<uint16_t, PeerTaskState> peer_tasks_;
  std::unordered_map<uint16_t, std::string> pvbsm_source_frames_;
  std::string peer_pvbsm_topic_;

  void readParameters(ExplorerConfig &config)
  {
    private_nh_.param("topics/odom", odom_topic_, odom_topic_);
    private_nh_.param("topics/cloud", cloud_topic_, cloud_topic_);
    private_nh_.param(
        "topics/degenerate", degenerate_topic_, degenerate_topic_);
    private_nh_.param("topics/degeneracy_score", score_topic_, score_topic_);
    private_nh_.param("topics/lio_runtime_ms", runtime_topic_, runtime_topic_);
    private_nh_.param("topics/pvbsm_delta", pvbsm_topic_, pvbsm_topic_);
    private_nh_.param("topics/peer_pvbsm_delta",
                      peer_pvbsm_topic_, peer_pvbsm_topic_);
    private_nh_.param("topics/goal", goal_topic_, goal_topic_);
    private_nh_.param("topics/frontiers", frontiers_topic_, frontiers_topic_);
    private_nh_.param("topics/valid_cluster_frontiers",
                      valid_cluster_frontiers_topic_,
                      valid_cluster_frontiers_topic_);
    private_nh_.param("topics/selected_cluster_frontiers",
                      selected_cluster_frontiers_topic_,
                      selected_cluster_frontiers_topic_);
    private_nh_.param(
        "topics/planning_cloud", planning_cloud_topic_, planning_cloud_topic_);
    private_nh_.param("topics/ready", ready_topic_, ready_topic_);
    private_nh_.param("topics/state", state_topic_, state_topic_);
    private_nh_.param(
        "topics/generation", generation_topic_, generation_topic_);
    private_nh_.param(
        "topics/pvbsm_memory_stats",
        pvbsm_stats_topic_,
        pvbsm_stats_topic_);
    private_nh_.param("topics/decision_status",
                      decision_status_topic_, decision_status_topic_);
    private_nh_.param("topics/decision_event",
                      decision_event_topic_, decision_event_topic_);
    private_nh_.param("topics/decision_command",
                      decision_command_topic_, decision_command_topic_);
    private_nh_.param("topics/decision_ack",
                      decision_ack_topic_, decision_ack_topic_);
    private_nh_.param("topics/coexplore_task",
                      coexplore_task_topic_, coexplore_task_topic_);

    private_nh_.param("map_update_rate_hz", map_update_rate_hz_, 10.0);
    private_nh_.param("input_timeout_s", input_timeout_s_, 1.0);
    private_nh_.param("ready_heartbeat_rate_hz",
                      ready_heartbeat_rate_hz_,
                      ready_heartbeat_rate_hz_);
    private_nh_.param(
        "max_input_stamp_skew_s", max_input_stamp_skew_s_, 0.2);
    private_nh_.param(
        "max_cloud_points_to_convert", max_cloud_points_to_convert_, 6000);
    private_nh_.param(
        "max_published_frontiers", max_published_frontiers_, 2000);
    private_nh_.param("planning_output_radius_m",
                      planning_output_radius_m_,
                      planning_output_radius_m_);
    private_nh_.param("max_published_planning_points",
                      max_published_planning_points_, 6000);
    private_nh_.param(
        "pvbsm_memory_enabled",
        pvbsm_memory_enabled_,
        pvbsm_memory_enabled_);
    private_nh_.param(
        "pvbsm_memory_max_records",
        pvbsm_memory_max_records_,
        pvbsm_memory_max_records_);
    private_nh_.param(
        "max_pvbsm_records_per_delta",
        max_pvbsm_records_per_delta_,
        max_pvbsm_records_per_delta_);
    private_nh_.param("cooperation/enabled",
                      cooperation_enabled_, cooperation_enabled_);
    private_nh_.param("cooperation/expected_peer_count",
                      expected_peer_count_, expected_peer_count_);
    private_nh_.param("cooperation/goal_ttl_s",
                      peer_goal_ttl_s_, peer_goal_ttl_s_);
    private_nh_.param("cooperation/goal_exclusion_radius_m",
                      peer_goal_exclusion_radius_m_,
                      peer_goal_exclusion_radius_m_);
    private_nh_.param("cooperation/transform_timeout_s",
                      peer_transform_timeout_s_,
                      peer_transform_timeout_s_);
    private_nh_.param("cooperation/max_future_skew_s",
                      peer_max_future_skew_s_,
                      peer_max_future_skew_s_);
    map_update_rate_hz_ = std::max(0.2, map_update_rate_hz_);
    input_timeout_s_ = std::max(0.1, input_timeout_s_);
    ready_heartbeat_rate_hz_ = std::max(0.1, ready_heartbeat_rate_hz_);
    max_input_stamp_skew_s_ = std::max(0.0, max_input_stamp_skew_s_);
    max_cloud_points_to_convert_ = std::max(64, max_cloud_points_to_convert_);
    max_published_frontiers_ = std::max(1, max_published_frontiers_);
    planning_output_radius_m_ = std::max(1.0, planning_output_radius_m_);
    max_published_planning_points_ =
        std::max(1, max_published_planning_points_);
    pvbsm_memory_max_records_ = std::max(1, pvbsm_memory_max_records_);
    max_pvbsm_records_per_delta_ =
        std::max(1, max_pvbsm_records_per_delta_);
    expected_peer_count_ = std::max(0, expected_peer_count_);
    peer_goal_ttl_s_ = std::max(0.5, peer_goal_ttl_s_);
    peer_goal_exclusion_radius_m_ =
        std::max(0.5, peer_goal_exclusion_radius_m_);
    peer_transform_timeout_s_ = std::max(0.0, peer_transform_timeout_s_);
    peer_max_future_skew_s_ = std::max(0.0, peer_max_future_skew_s_);

    private_nh_.param("robot_id", config.robot_id, config.robot_id);
    config.cooperation_enabled = cooperation_enabled_;
    config.peer_goal_exclusion_radius_m = peer_goal_exclusion_radius_m_;
    private_nh_.param("cooperation/score_tie_epsilon",
                      config.peer_goal_score_epsilon,
                      config.peer_goal_score_epsilon);
    private_nh_.param("planning_voxel_size_m",
                      config.planning_voxel_size_m,
                      config.planning_voxel_size_m);
    private_nh_.param("planning_sensor_range_m",
                      config.planning_sensor_range_m,
                      config.planning_sensor_range_m);
    private_nh_.param("planning_map_radius_m",
                      config.planning_map_radius_m,
                      config.planning_map_radius_m);
    private_nh_.param("planning_prune_budget",
                      config.planning_prune_budget,
                      config.planning_prune_budget);
    private_nh_.param("max_raycasts_per_update",
                      config.max_raycasts_per_update,
                      config.max_raycasts_per_update);
    private_nh_.param(
        "max_ray_steps", config.max_ray_steps, config.max_ray_steps);
    private_nh_.param("frontier_update_budget",
                      config.frontier_update_budget,
                      config.frontier_update_budget);
    private_nh_.param("frontier_evaluation_budget",
                      config.frontier_evaluation_budget,
                      config.frontier_evaluation_budget);
    private_nh_.param("frontier_update_rate_hz",
                      config.frontier_update_rate_hz,
                      config.frontier_update_rate_hz);
    private_nh_.param("goal_evaluation_rate_hz",
                      config.goal_evaluation_rate_hz,
                      config.goal_evaluation_rate_hz);
    private_nh_.param("long_term_update_rate_hz",
                      config.long_term_update_rate_hz,
                      config.long_term_update_rate_hz);
    private_nh_.param("coverage_voxel_size_m",
                      config.coverage_voxel_size_m,
                      config.coverage_voxel_size_m);
    private_nh_.param("exploration_memory_enabled",
                      config.exploration_memory_enabled,
                      config.exploration_memory_enabled);
    private_nh_.param("exploration_memory_filter_enabled",
                      config.exploration_memory_filter_enabled,
                      config.exploration_memory_filter_enabled);
    private_nh_.param("exploration_memory_voxel_size_m",
                      config.exploration_memory_voxel_size_m,
                      config.exploration_memory_voxel_size_m);
    private_nh_.param("exploration_memory_min_observations",
                      config.exploration_memory_min_observations,
                      config.exploration_memory_min_observations);
    private_nh_.param("exploration_memory_max_range_m",
                      config.exploration_memory_max_range_m,
                      config.exploration_memory_max_range_m);
    private_nh_.param("frontier_history_probe_distance_m",
                      config.frontier_history_probe_distance_m,
                      config.frontier_history_probe_distance_m);
    private_nh_.param("frontier_history_probe_step_m",
                      config.frontier_history_probe_step_m,
                      config.frontier_history_probe_step_m);
    private_nh_.param("frontier_history_observed_ratio",
                      config.frontier_history_observed_ratio,
                      config.frontier_history_observed_ratio);
    private_nh_.param("replan_interval_s",
                      config.replan_interval_s,
                      config.replan_interval_s);
    private_nh_.param("goal_min_hold_time_s",
                      config.goal_min_hold_time_s,
                      config.goal_min_hold_time_s);
    private_nh_.param("goal_blocked_confirm_updates",
                      config.goal_blocked_confirm_updates,
                      config.goal_blocked_confirm_updates);
    private_nh_.param(
        "goal_timeout_s", config.goal_timeout_s, config.goal_timeout_s);
    private_nh_.param("goal_reached_distance_m",
                      config.goal_reached_distance_m,
                      config.goal_reached_distance_m);
    private_nh_.param("goal_progress_epsilon_m",
                      config.goal_progress_epsilon_m,
                      config.goal_progress_epsilon_m);
    private_nh_.param("goal_stall_timeout_s",
                      config.goal_stall_timeout_s,
                      config.goal_stall_timeout_s);
    private_nh_.param("failed_goal_exclusion_radius_m",
                      config.failed_goal_exclusion_radius_m,
                      config.failed_goal_exclusion_radius_m);
    private_nh_.param("failed_goal_cooldown_s",
                      config.failed_goal_cooldown_s,
                      config.failed_goal_cooldown_s);
    private_nh_.param("allow_periodic_goal_switch",
                      config.allow_periodic_goal_switch,
                      config.allow_periodic_goal_switch);
    private_nh_.param("min_goal_distance_m",
                      config.min_goal_distance_m,
                      config.min_goal_distance_m);
    private_nh_.param("max_goal_distance_m",
                      config.max_goal_distance_m,
                      config.max_goal_distance_m);
    private_nh_.param("min_known_free_path_ratio",
                      config.min_known_free_path_ratio,
                      config.min_known_free_path_ratio);
    private_nh_.param("max_goal_vertical_distance_m",
                      config.max_goal_vertical_distance_m,
                      config.max_goal_vertical_distance_m);
    private_nh_.param("goal_switch_margin",
                      config.goal_switch_margin,
                      config.goal_switch_margin);
    private_nh_.param("frontier_cluster_size_m",
                      config.frontier_cluster_size_m,
                      config.frontier_cluster_size_m);
    private_nh_.param("min_frontier_cluster_cells",
                      config.min_frontier_cluster_cells,
                      config.min_frontier_cluster_cells);
    private_nh_.param("viewpoint_standoff_m",
                      config.viewpoint_standoff_m,
                      config.viewpoint_standoff_m);
    private_nh_.param("viewpoint_search_radius_m",
                      config.viewpoint_search_radius_m,
                      config.viewpoint_search_radius_m);
    private_nh_.param("viewpoint_same_height_tolerance_m",
                      config.viewpoint_same_height_tolerance_m,
                      config.viewpoint_same_height_tolerance_m);
    private_nh_.param("min_wall_clearance_m",
                      config.min_wall_clearance_m,
                      config.min_wall_clearance_m);
    private_nh_.param("max_viewpoints_per_cluster",
                      config.max_viewpoints_per_cluster,
                      config.max_viewpoints_per_cluster);
    private_nh_.param("max_safe_viewpoint_candidates",
                      config.max_safe_viewpoint_candidates,
                      config.max_safe_viewpoint_candidates);
    private_nh_.param("preferred_heading_change_deg",
                      config.preferred_heading_change_deg,
                      config.preferred_heading_change_deg);
    private_nh_.param("max_heading_change_deg",
                      config.max_heading_change_deg,
                      config.max_heading_change_deg);
    private_nh_.param("distance_cost_weight",
                      config.distance_cost_weight,
                      config.distance_cost_weight);
    private_nh_.param("heading_cost_weight",
                      config.heading_cost_weight,
                      config.heading_cost_weight);
    private_nh_.param("arrival_yaw_cost_weight",
                      config.arrival_yaw_cost_weight,
                      config.arrival_yaw_cost_weight);
    private_nh_.param("reachability_enabled",
                      config.reachability_enabled,
                      config.reachability_enabled);
    private_nh_.param("reachability_max_expansions",
                      config.reachability_max_expansions,
                      config.reachability_max_expansions);
    private_nh_.param("goal_reachability_check_rate_hz",
                      config.goal_reachability_check_rate_hz,
                      config.goal_reachability_check_rate_hz);
    private_nh_.param("dynamic_budget_enabled",
                      config.dynamic_budget_enabled,
                      config.dynamic_budget_enabled);
    private_nh_.param("lio_busy_threshold_ms",
                      config.lio_busy_threshold_ms,
                      config.lio_busy_threshold_ms);
    private_nh_.param("lio_overload_threshold_ms",
                      config.lio_overload_threshold_ms,
                      config.lio_overload_threshold_ms);
    private_nh_.param("lio_time_ema_alpha",
                      config.lio_time_ema_alpha,
                      config.lio_time_ema_alpha);
    private_nh_.param("busy_budget_scale",
                      config.busy_budget_scale,
                      config.busy_budget_scale);
    private_nh_.param("overload_budget_scale",
                      config.overload_budget_scale,
                      config.overload_budget_scale);
    private_nh_.param("degenerate_max_goal_distance_m",
                      config.degenerate_max_goal_distance_m,
                      config.degenerate_max_goal_distance_m);
    private_nh_.param("degenerate_goal_switch_margin",
                      config.degenerate_goal_switch_margin,
                      config.degenerate_goal_switch_margin);
    private_nh_.param("degenerate_safe_path_weight",
                      config.degenerate_safe_path_weight,
                      config.degenerate_safe_path_weight);
    private_nh_.param("pvbsm_scoring_enabled",
                      config.pvbsm_scoring_enabled,
                      config.pvbsm_scoring_enabled);
    private_nh_.param("pvbsm_root_voxel_size_m",
                      config.pvbsm_root_voxel_size_m,
                      config.pvbsm_root_voxel_size_m);
    private_nh_.param("pvbsm_submap_edge_roots",
                      config.pvbsm_submap_edge_roots,
                      config.pvbsm_submap_edge_roots);
    private_nh_.param("pvbsm_covered_root_target",
                      config.pvbsm_covered_root_target,
                      config.pvbsm_covered_root_target);
    private_nh_.param("pvbsm_unseen_submap_bonus",
                      config.pvbsm_unseen_submap_bonus,
                      config.pvbsm_unseen_submap_bonus);
    private_nh_.param("pvbsm_submap_coverage_penalty",
                      config.pvbsm_submap_coverage_penalty,
                      config.pvbsm_submap_coverage_penalty);
    private_nh_.param("pvbsm_observed_root_penalty",
                      config.pvbsm_observed_root_penalty,
                      config.pvbsm_observed_root_penalty);
    private_nh_.param("pvbsm_degenerate_structure_bonus",
                      config.pvbsm_degenerate_structure_bonus,
                      config.pvbsm_degenerate_structure_bonus);
    if (config.goal_timeout_s > 0.0)
    {
      ROS_WARN_STREAM(
          "[ DAIB Explorer ] goal_timeout_s is deprecated and disables "
          "progress-based stall replacement; set it to 0 for the default "
          "goal_progress_epsilon_m/goal_stall_timeout_s policy");
    }
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &message)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_odom_ = message;
    last_odom_receive_ = ros::WallTime::now();
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &message)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_cloud_ = message;
    ++cloud_sequence_;
    last_cloud_receive_ = ros::WallTime::now();
  }

  void degenerateCallback(const std_msgs::BoolConstPtr &message)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    degenerate_ = message->data;
  }

  void scoreCallback(const std_msgs::Float64ConstPtr &message)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    degeneracy_score_ = message->data;
  }

  void runtimeCallback(const std_msgs::Float64ConstPtr &message)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    lio_runtime_ms_ = message->data;
  }

  std::vector<PvbsmExplorationHint> queryFusedPvbsmHints(
      const std::vector<PvbsmQueryPoint> &local_points,
      double root_voxel_size,
      std::size_t covered_root_target)
  {
    std::vector<PvbsmExplorationHint> fused(local_points.size());
    std::vector<uint16_t> sources;
    std::unordered_map<uint16_t, std::string> source_frames;
    {
      std::lock_guard<std::mutex> lock(pvbsm_mutex_);
      if (!pvbsm_memory_) return fused;
      sources = pvbsm_memory_->sourceIds();
      source_frames = pvbsm_source_frames_;
    }
    std::string local_frame;
    {
      std::lock_guard<std::mutex> lock(peer_mutex_);
      local_frame = planning_frame_;
    }

    for (uint16_t source_id : sources)
    {
      std::vector<PvbsmQueryPoint> source_points = local_points;
      if (source_id != pvbsm_query_source_id_)
      {
        const auto source_frame = source_frames.find(source_id);
        if (local_frame.empty() || source_frame == source_frames.end() ||
            source_frame->second.empty())
          continue;
        try
        {
          tf::StampedTransform transform;
          tf_listener_.lookupTransform(
              source_frame->second, local_frame, ros::Time(0), transform);
          for (std::size_t index = 0; index < local_points.size(); ++index)
          {
            const tf::Vector3 local(
                local_points[index].x,
                local_points[index].y,
                local_points[index].z);
            const tf::Vector3 remote = transform * local;
            source_points[index] = {remote.x(), remote.y(), remote.z()};
          }
        }
        catch (const tf::TransformException &error)
        {
          ROS_WARN_THROTTLE(
              2.0, "[ DAIB CoExplore ] remote PVBSM TF unavailable: %s",
              error.what());
          continue;
        }
      }

      std::vector<PvbsmExplorationHint> hints;
      {
        std::lock_guard<std::mutex> lock(pvbsm_mutex_);
        hints = pvbsm_memory_->queryExplorationHints(
            source_points, source_id, root_voxel_size,
            pvbsm_expected_submap_edge_roots_, covered_root_target);
      }
      if (hints.size() != fused.size()) continue;
      for (std::size_t index = 0; index < fused.size(); ++index)
      {
        fused[index].root_observed =
            fused[index].root_observed || hints[index].root_observed;
        fused[index].submap_observed =
            fused[index].submap_observed || hints[index].submap_observed;
        fused[index].submap_coverage = std::max(
            fused[index].submap_coverage, hints[index].submap_coverage);
        fused[index].structural_support = std::max(
            fused[index].structural_support, hints[index].structural_support);
      }
    }
    return fused;
  }

  void pvbsmCallback(const sensor_msgs::PointCloud2ConstPtr &message)
  {
    if (!pvbsm_memory_) return;
    std::vector<PvbsmRecord> records;
    const std::size_t count =
        static_cast<std::size_t>(message->width) * message->height;
    records.reserve(std::min<std::size_t>(
        count, static_cast<std::size_t>(max_pvbsm_records_per_delta_)));
    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      sensor_msgs::PointCloud2ConstIterator<float> normal_x(
          *message, "normal_x");
      sensor_msgs::PointCloud2ConstIterator<float> normal_y(
          *message, "normal_y");
      sensor_msgs::PointCloud2ConstIterator<float> normal_z(
          *message, "normal_z");
      sensor_msgs::PointCloud2ConstIterator<float> extent_x(
          *message, "extent_x");
      sensor_msgs::PointCloud2ConstIterator<float> extent_y(
          *message, "extent_y");
      sensor_msgs::PointCloud2ConstIterator<float> thickness(
          *message, "thickness");
      sensor_msgs::PointCloud2ConstIterator<float> confidence(
          *message, "confidence");
      sensor_msgs::PointCloud2ConstIterator<int32_t> root_x(
          *message, "root_x");
      sensor_msgs::PointCloud2ConstIterator<int32_t> root_y(
          *message, "root_y");
      sensor_msgs::PointCloud2ConstIterator<int32_t> root_z(
          *message, "root_z");
      sensor_msgs::PointCloud2ConstIterator<uint32_t> revision(
          *message, "revision");
      sensor_msgs::PointCloud2ConstIterator<uint16_t> point_count(
          *message, "point_count");
      sensor_msgs::PointCloud2ConstIterator<uint16_t> source_id(
          *message, "source_id");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> layer(
          *message, "layer");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> kind(
          *message, "kind");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> flags(
          *message, "flags");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> submap_edge(
          *message, "submap_edge_roots");

      for (; x != x.end() &&
             records.size() <
                 static_cast<std::size_t>(max_pvbsm_records_per_delta_);
           ++x, ++y, ++z, ++normal_x, ++normal_y, ++normal_z,
           ++extent_x, ++extent_y, ++thickness, ++confidence,
           ++root_x, ++root_y, ++root_z, ++revision, ++point_count,
           ++source_id, ++layer, ++kind, ++flags, ++submap_edge)
      {
        PvbsmRecord record;
        record.center[0] = *x;
        record.center[1] = *y;
        record.center[2] = *z;
        record.normal[0] = *normal_x;
        record.normal[1] = *normal_y;
        record.normal[2] = *normal_z;
        record.extent_x = *extent_x;
        record.extent_y = *extent_y;
        record.thickness = *thickness;
        record.confidence = *confidence;
        record.root[0] = *root_x;
        record.root[1] = *root_y;
        record.root[2] = *root_z;
        record.revision = *revision;
        record.point_count = *point_count;
        record.source_id = *source_id;
        record.layer = *layer;
        record.kind = *kind;
        record.flags = *flags;
        record.submap_edge_roots = *submap_edge;
        records.push_back(record);
      }
    }
    catch (const std::runtime_error &error)
    {
      ROS_ERROR_THROTTLE(
          2.0, "[ DAIB Explorer ] invalid PVBSM schema: %s", error.what());
      return;
    }

    bool submap_edge_mismatch = false;
    for (const PvbsmRecord &record : records)
    {
      submap_edge_mismatch =
          submap_edge_mismatch ||
          record.submap_edge_roots !=
              pvbsm_expected_submap_edge_roots_;
    }
    if (submap_edge_mismatch)
    {
      ROS_ERROR_THROTTLE(
          5.0,
          "[ DAIB Explorer PVBSM ] submap_edge_roots mismatch between "
          "FAST-LIVO2 and Explorer");
    }

    std::lock_guard<std::mutex> lock(pvbsm_mutex_);
    for (const PvbsmRecord &record : records)
      if (!message->header.frame_id.empty())
        pvbsm_source_frames_[record.source_id] = message->header.frame_id;
    pvbsm_memory_->applyDelta(records);
    const PvbsmMemoryStats &stats = pvbsm_memory_->stats();
    const std::size_t source_count = pvbsm_memory_->sourceIds().size();
    std_msgs::UInt64MultiArray stats_message;
    stats_message.data = {
        stats.root_count,
        stats.record_count,
        stats.plane_count,
        stats.residual_count,
        stats.submap_count,
        stats.accepted_root_updates,
        stats.rejected_stale_root_updates,
        stats.deleted_roots,
        stats.capacity_evictions,
        stats.source_session_resets,
        stats.detailed_root_count,
        source_count};
    pvbsm_stats_pub_.publish(stats_message);
    ROS_INFO_STREAM_THROTTLE(
        1.0, "[ DAIB Explorer PVBSM ] roots=" << stats.root_count
                 << ", detailed_roots=" << stats.detailed_root_count
                 << ", records=" << stats.record_count
                 << ", submaps=" << stats.submap_count
                 << ", sources=" << source_count
                 << ", stale_rejected="
                 << stats.rejected_stale_root_updates);
  }

  void publishReady(bool value)
  {
    const ros::WallTime now = ros::WallTime::now();
    const double heartbeat_period = 1.0 / ready_heartbeat_rate_hz_;
    if (ready_initialized_ && ready_ == value &&
        (now - last_ready_publish_).toSec() < heartbeat_period)
      return;
    ready_initialized_ = true;
    ready_ = value;
    last_ready_publish_ = now;
    std_msgs::Bool message;
    message.data = value;
    ready_pub_.publish(message);

    daib_decision::DaibModuleStatus status;
    status.header.stamp = ros::Time::now();
    status.source = daib_decision::DaibModuleStatus::SOURCE_EXPLORER;
    status.session = decision_source_session_;
    status.sequence = ++decision_status_sequence_;
    status.valid_for_s =
        static_cast<float>(2.0 / ready_heartbeat_rate_hz_);
    status.health = value
                        ? daib_decision::DaibModuleStatus::HEALTH_NOMINAL
                        : daib_decision::DaibModuleStatus::HEALTH_DEGRADED;
    status.fault_mask = value
                            ? daib_decision::DaibModuleStatus::FAULT_NONE
                            : daib_decision::DaibModuleStatus::EXPLORER_STALE;
    status.ready = value;
    status.generation = published_generation_;
    status.trajectory_valid = false;
    status.consecutive_failures = 0;
    status.metric_primary = core_ ? core_->stats().last_update_ms : 0.0;
    status.metric_secondary = core_ ? core_->stats().last_plan_ms : 0.0;
    decision_status_pub_.publish(status);
  }

  void publishDecisionEvent(uint16_t type, uint64_t generation,
                            uint64_t reason_mask = 0)
  {
    daib_decision::DaibEvent event;
    event.header.stamp = ros::Time::now();
    event.source = daib_decision::DaibModuleStatus::SOURCE_EXPLORER;
    event.event_type = type;
    event.session = decision_source_session_;
    event.sequence = ++decision_event_sequence_;
    event.valid_for_s = 2.0;
    event.generation = generation;
    event.reason_mask = reason_mask;
    decision_event_pub_.publish(event);
  }

  void publishDecisionAck(uint64_t command_id, uint8_t state,
                          uint64_t reason_mask = 0)
  {
    daib_decision::DaibActionAck ack;
    ack.header.stamp = ros::Time::now();
    ack.decision_session = latest_decision_session_;
    ack.command_id = command_id;
    ack.source = daib_decision::DaibModuleStatus::SOURCE_EXPLORER;
    ack.ack_state = state;
    ack.reason_mask = reason_mask;
    decision_ack_pub_.publish(ack);
  }

  void decisionCommandCallback(
      const daib_decision::DaibDecisionCommandConstPtr &command)
  {
    if (command->target_source !=
        daib_decision::DaibModuleStatus::SOURCE_EXPLORER)
      return;
    if (command->decision_session < latest_decision_session_ ||
        (command->decision_session == latest_decision_session_ &&
         command->command_id <= latest_decision_command_id_))
      return;
    if (command->decision_session > latest_decision_session_)
      latest_decision_command_id_ = 0;
    latest_decision_session_ = command->decision_session;
    latest_decision_command_id_ = command->command_id;

    if (command->action ==
        daib_decision::DaibDecisionCommand::ACTION_REDUCE_BUDGET)
    {
      core_->setDecisionProfile(command->profile);
      return;
    }
    if (command->action ==
            daib_decision::DaibDecisionCommand::ACTION_RESELECT_GOAL ||
        command->action ==
            daib_decision::DaibDecisionCommand::ACTION_ENTER_ESCAPE)
    {
      const bool accepted = core_->requestGoalReselection(
          command->action ==
          daib_decision::DaibDecisionCommand::ACTION_ENTER_ESCAPE);
      publishDecisionAck(
          command->command_id,
          accepted ? daib_decision::DaibActionAck::ACK_SUCCEEDED
                   : daib_decision::DaibActionAck::ACK_REJECTED);
    }
  }

  void coexploreTaskCallback(
      const daib_explorer::CoExploreTaskConstPtr &message)
  {
    if (!cooperation_enabled_ || message->robot_id == robot_id_) return;
    if (message->header.frame_id.empty())
    {
      ROS_WARN_THROTTLE(
          2.0, "[ DAIB CoExplore ] rejecting peer task without frame_id");
      return;
    }
    if (!std::isfinite(message->goal.position.x) ||
        !std::isfinite(message->goal.position.y) ||
        !std::isfinite(message->goal.position.z) ||
        !std::isfinite(message->score) ||
        !std::isfinite(message->exclusion_radius_m))
    {
      ROS_WARN_THROTTLE(
          2.0, "[ DAIB CoExplore ] rejecting non-finite peer task");
      return;
    }
    const ros::Time ros_now = ros::Time::now();
    if (!message->header.stamp.isZero())
    {
      const double age = (ros_now - message->header.stamp).toSec();
      if (age > peer_goal_ttl_s_ || age < -peer_max_future_skew_s_)
      {
        ROS_WARN_THROTTLE(
            2.0, "[ DAIB CoExplore ] rejecting stale/future peer task");
        return;
      }
    }

    geometry_msgs::PoseStamped local_goal;
    geometry_msgs::PoseStamped peer_goal;
    peer_goal.header = message->header;
    peer_goal.pose = message->goal;
    // Task arbitration uses only the position; ignore a malformed or
    // convention-dependent peer yaw while transforming between map frames.
    peer_goal.pose.orientation.x = 0.0;
    peer_goal.pose.orientation.y = 0.0;
    peer_goal.pose.orientation.z = 0.0;
    peer_goal.pose.orientation.w = 1.0;
    std::string local_frame;
    {
      std::lock_guard<std::mutex> lock(peer_mutex_);
      local_frame = planning_frame_;
    }
    if (local_frame.empty()) return;
    try
    {
      if (peer_goal.header.frame_id == local_frame)
        local_goal = peer_goal;
      else
      {
        tf_listener_.waitForTransform(
            local_frame, peer_goal.header.frame_id,
            peer_goal.header.stamp.isZero() ? ros::Time(0)
                                            : peer_goal.header.stamp,
            ros::Duration(peer_transform_timeout_s_));
        tf_listener_.transformPose(local_frame, peer_goal, local_goal);
      }
    }
    catch (const tf::TransformException &error)
    {
      ROS_WARN_THROTTLE(
          2.0, "[ DAIB CoExplore ] peer task TF unavailable: %s",
          error.what());
      return;
    }
    if (!std::isfinite(local_goal.pose.position.x) ||
        !std::isfinite(local_goal.pose.position.y) ||
        !std::isfinite(local_goal.pose.position.z))
    {
      ROS_WARN_THROTTLE(
          2.0, "[ DAIB CoExplore ] rejecting invalid transformed task");
      return;
    }

    std::lock_guard<std::mutex> lock(peer_mutex_);
    const auto previous = peer_tasks_.find(message->robot_id);
    if (previous != peer_tasks_.end())
    {
      const PeerGoalLease &lease = previous->second.lease;
      if (message->session < lease.session ||
          (message->session == lease.session &&
           message->sequence <= lease.sequence))
        return;
    }
    PeerTaskState state;
    state.lease.robot_id = message->robot_id;
    state.lease.session = message->session;
    state.lease.sequence = message->sequence;
    state.lease.generation = message->generation;
    state.lease.position = {
        local_goal.pose.position.x,
        local_goal.pose.position.y,
        local_goal.pose.position.z};
    state.lease.score = message->score;
    state.lease.exclusion_radius_m = std::max(
        0.5, message->exclusion_radius_m);
    const double local_now = ros_now.toSec();
    const double declared_expiry = message->valid_until.isZero()
                                       ? local_now + peer_goal_ttl_s_
                                       : message->valid_until.toSec();
    // A remote clock error must never create an unbounded ownership lease.
    state.lease.valid_until = std::min(
        declared_expiry, local_now + peer_goal_ttl_s_);
    state.lease.active = message->active;
    state.last_receive = ros::WallTime::now();
    peer_tasks_[message->robot_id] = state;
  }

  void updatePeerLeases(double timestamp)
  {
    if (!cooperation_enabled_) return;
    std::vector<PeerGoalLease> leases;
    std::lock_guard<std::mutex> lock(peer_mutex_);
    for (auto peer = peer_tasks_.begin(); peer != peer_tasks_.end();)
    {
      if (peer->second.lease.valid_until + peer_goal_ttl_s_ < timestamp)
      {
        peer = peer_tasks_.erase(peer);
        continue;
      }
      if (peer->second.lease.valid_until > timestamp)
        leases.push_back(peer->second.lease);
      ++peer;
    }
    core_->setPeerGoalLeases(std::move(leases));
  }

  void publishPeerHealth()
  {
    if (!cooperation_enabled_) return;
    const ros::WallTime now = ros::WallTime::now();
    std::size_t fresh_peers = 0;
    {
      std::lock_guard<std::mutex> lock(peer_mutex_);
      for (const auto &peer : peer_tasks_)
        if ((now - peer.second.last_receive).toSec() <= peer_goal_ttl_s_)
          ++fresh_peers;
    }
    const bool ready = fresh_peers >= static_cast<std::size_t>(expected_peer_count_);
    daib_decision::DaibModuleStatus status;
    status.header.stamp = ros::Time::now();
    status.source = daib_decision::DaibModuleStatus::SOURCE_PEER;
    status.session = decision_source_session_;
    status.sequence = ++decision_status_sequence_;
    status.valid_for_s = static_cast<float>(2.0 * peer_goal_ttl_s_);
    status.health = ready
                        ? daib_decision::DaibModuleStatus::HEALTH_NOMINAL
                        : daib_decision::DaibModuleStatus::HEALTH_DEGRADED;
    status.fault_mask = ready
                            ? daib_decision::DaibModuleStatus::FAULT_NONE
                            : daib_decision::DaibModuleStatus::PEER_LINK_LOST;
    status.ready = ready;
    status.generation = published_generation_;
    status.metric_primary = static_cast<double>(fresh_peers);
    status.metric_secondary = static_cast<double>(expected_peer_count_);
    decision_status_pub_.publish(status);
  }

  void publishCoexploreLease(bool force = false)
  {
    if (!cooperation_enabled_ || planning_frame_.empty()) return;
    const ros::WallTime wall_now = ros::WallTime::now();
    if (!force && !last_coexplore_publish_.isZero() &&
        (wall_now - last_coexplore_publish_).toSec() < 1.0)
      return;
    last_coexplore_publish_ = wall_now;
    daib_explorer::CoExploreTask message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = planning_frame_;
    message.robot_id = robot_id_;
    message.session = coexplore_session_;
    message.sequence = ++coexplore_sequence_;
    message.generation = leased_goal_.generation;
    message.goal.position.x = leased_goal_.position.x;
    message.goal.position.y = leased_goal_.position.y;
    message.goal.position.z = leased_goal_.position.z;
    message.goal.orientation =
        tf::createQuaternionMsgFromYaw(leased_goal_.yaw);
    message.score = leased_goal_.score;
    message.exclusion_radius_m = peer_goal_exclusion_radius_m_;
    message.valid_until = message.header.stamp + ros::Duration(peer_goal_ttl_s_);
    message.active = leased_goal_active_;
    coexplore_task_pub_.publish(message);
  }

  std::vector<Vec3> convertCloud(const sensor_msgs::PointCloud2 &cloud) const
  {
    std::vector<Vec3> points;
    const std::size_t count =
        static_cast<std::size_t>(cloud.width) * cloud.height;
    if (count == 0) return points;
    const std::size_t limit =
        static_cast<std::size_t>(max_cloud_points_to_convert_);
    const std::size_t stride =
        std::max<std::size_t>(1, (count + limit - 1) / limit);
    points.reserve(std::min(count, limit));

    sensor_msgs::PointCloud2ConstIterator<float> x_iter(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_iter(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_iter(cloud, "z");
    for (std::size_t index = 0;
         x_iter != x_iter.end();
         ++x_iter, ++y_iter, ++z_iter, ++index)
    {
      if (index % stride != 0) continue;
      if (!std::isfinite(*x_iter) ||
          !std::isfinite(*y_iter) ||
          !std::isfinite(*z_iter))
        continue;
      points.push_back({*x_iter, *y_iter, *z_iter});
      if (points.size() >= limit) break;
    }
    return points;
  }

  void publishPointCloud(const std::vector<Vec3> &points,
                         const std_msgs::Header &header,
                         const ros::Publisher &publisher) const
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(points.size());
    for (const Vec3 &point : points)
    {
      pcl::PointXYZ output_point;
      output_point.x = static_cast<float>(point.x);
      output_point.y = static_cast<float>(point.y);
      output_point.z = static_cast<float>(point.z);
      cloud.push_back(output_point);
    }
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header = header;
    publisher.publish(message);
  }

  void mapTimerCallback(const ros::TimerEvent &)
  {
    std::unique_lock<std::mutex> update_lock(update_mutex_, std::try_to_lock);
    if (!update_lock.owns_lock())
    {
      ROS_WARN_THROTTLE(
          2.0, "[ DAIB Explorer ] previous map update still running; skipping tick");
      return;
    }

    nav_msgs::OdometryConstPtr odom;
    sensor_msgs::PointCloud2ConstPtr cloud;
    bool degenerate;
    double score;
    double runtime;
    ros::WallTime odom_receive;
    ros::WallTime cloud_receive;
    uint64_t cloud_sequence;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      odom = latest_odom_;
      cloud = latest_cloud_;
      degenerate = degenerate_;
      score = degeneracy_score_;
      runtime = lio_runtime_ms_;
      odom_receive = last_odom_receive_;
      cloud_receive = last_cloud_receive_;
      cloud_sequence = cloud_sequence_;
    }

    const ros::WallTime now = ros::WallTime::now();
    if (!odom || !cloud ||
        (now - odom_receive).toSec() > input_timeout_s_ ||
        (now - cloud_receive).toSec() > input_timeout_s_)
    {
      publishReady(false);
      ROS_WARN_THROTTLE(2.0, "[ DAIB Explorer ] waiting for fresh odometry and cloud");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(peer_mutex_);
      planning_frame_ = cloud->header.frame_id;
    }
    publishCoexploreLease();
    publishPeerHealth();
    if (cloud_sequence == processed_cloud_sequence_)
    {
      publishReady(true);
      return;
    }
    if (!odom->header.frame_id.empty() && !cloud->header.frame_id.empty() &&
        odom->header.frame_id != cloud->header.frame_id)
    {
      publishReady(false);
      ROS_ERROR_THROTTLE(
          2.0, "[ DAIB Explorer ] frame mismatch: odom='%s', cloud='%s'",
          odom->header.frame_id.c_str(), cloud->header.frame_id.c_str());
      return;
    }
    if (!odom->header.stamp.isZero() && !cloud->header.stamp.isZero() &&
        std::fabs((odom->header.stamp - cloud->header.stamp).toSec()) >
            max_input_stamp_skew_s_)
    {
      publishReady(false);
      ROS_WARN_THROTTLE(
          2.0, "[ DAIB Explorer ] odometry/cloud timestamp skew is too large");
      return;
    }

    std::vector<Vec3> points;
    try
    {
      points = convertCloud(*cloud);
    }
    catch (const std::runtime_error &error)
    {
      publishReady(false);
      ROS_ERROR_THROTTLE(
          2.0, "[ DAIB Explorer ] invalid PointCloud2 schema: %s",
          error.what());
      return;
    }
    const Vec3 position{odom->pose.pose.position.x,
                        odom->pose.pose.position.y,
                        odom->pose.pose.position.z};
    const Quaternion orientation{odom->pose.pose.orientation.x,
                                 odom->pose.pose.orientation.y,
                                 odom->pose.pose.orientation.z,
                                 odom->pose.pose.orientation.w};
    const double timestamp =
        cloud->header.stamp.isZero() ? ros::Time::now().toSec()
                                    : cloud->header.stamp.toSec();
    updatePeerLeases(timestamp);
    const uint64_t previous_frontier_cycle =
        core_->stats().frontier_update_cycles;
    const uint64_t previous_goal_cycle =
        core_->stats().goal_evaluation_cycles;
    core_->setHealth(degenerate, score, runtime);
    core_->update(position, orientation, points, timestamp);
    processed_cloud_sequence_ = cloud_sequence;
    publishReady(true);

    if (core_->stats().frontier_update_cycles != previous_frontier_cycle)
    {
      publishPointCloud(
          core_->frontierPoints(
              static_cast<std::size_t>(max_published_frontiers_)),
          cloud->header, frontiers_pub_);
    }
    if (core_->stats().goal_evaluation_cycles != previous_goal_cycle)
      publishPointCloud(
          core_->validClusterFrontierPoints(),
          cloud->header, valid_cluster_frontiers_pub_);
    if (planning_cloud_pub_.getNumSubscribers() > 0)
      publishPointCloud(
          core_->occupiedPoints(
              position,
              planning_output_radius_m_,
              static_cast<std::size_t>(max_published_planning_points_)),
          cloud->header, planning_cloud_pub_);

    GoalDecision decision;
    if (core_->consumeDecision(decision))
    {
      std_msgs::String state_message;
      state_message.data = decision.state + ":" + decision.reason;
      state_pub_.publish(state_message);
      const bool selected_cluster_matches_goal =
          decision.valid &&
          core_->selectedClusterGeneration() == decision.generation;
      if (decision.valid && !selected_cluster_matches_goal)
      {
        ROS_ERROR_STREAM(
            "[ DAIB Explorer ] selected cluster generation mismatch: goal="
            << decision.generation << ", cluster="
            << core_->selectedClusterGeneration());
      }
      const std::vector<Vec3> selected_cluster_points =
          selected_cluster_matches_goal ? core_->selectedFrontierPoints()
                                        : std::vector<Vec3>{};
      publishPointCloud(
          selected_cluster_points,
          cloud->header, selected_cluster_frontiers_pub_);
      const uint64_t previous_generation = published_generation_;
      if (decision.reason == "goal_reached" ||
          decision.reason == "goal_reached_no_next_frontier")
        publishDecisionEvent(
            daib_decision::DaibEvent::GOAL_REACHED,
            previous_generation);
      else if (decision.reason == "new_obstacle" ||
               decision.reason == "goal_blocked_no_safe_frontier")
        publishDecisionEvent(
            daib_decision::DaibEvent::GOAL_BLOCKED,
            previous_generation);
      else if (decision.reason == "goal_stalled" ||
               decision.reason == "goal_stalled_no_safe_frontier" ||
               decision.reason == "goal_timeout" ||
               decision.reason == "goal_timeout_no_safe_frontier")
        publishDecisionEvent(
            daib_decision::DaibEvent::GOAL_STALLED,
            previous_generation);
      if (decision.valid)
      {
        leased_goal_ = decision;
        leased_goal_active_ = true;
        geometry_msgs::PoseStamped goal;
        goal.header = cloud->header;
        goal.pose.position.x = decision.position.x;
        goal.pose.position.y = decision.position.y;
        goal.pose.position.z = decision.position.z;
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(decision.yaw);
        goal_pub_.publish(goal);
        // ROS1 owns Header.seq and may overwrite it during publication.
        // Keep the application-level generation on its dedicated topic.
        std_msgs::UInt64 generation;
        generation.data = decision.generation;
        generation_pub_.publish(generation);
        published_generation_ = decision.generation;
        publishDecisionEvent(
            daib_decision::DaibEvent::GOAL_AVAILABLE,
            decision.generation);
        publishCoexploreLease(true);
        ROS_INFO_STREAM("[ DAIB Explorer ] generation=" << decision.generation
                        << ", state=" << decision.state
                        << ", reason=" << decision.reason
                        << ", goal=(" << decision.position.x << ", "
                        << decision.position.y << ", " << decision.position.z
                        << "), tier=" << decision.constraint_tier
                        << ", cluster_cells="
                        << selected_cluster_points.size()
                        << ", heading_change="
                        << decision.heading_change_deg << " deg"
                        << ", plan=" << decision.planning_time_ms << " ms");
      }
      else
      {
        leased_goal_active_ = false;
        publishCoexploreLease(true);
        publishDecisionEvent(
            daib_decision::DaibEvent::NO_SAFE_FRONTIER,
            previous_generation,
            daib_decision::DaibModuleStatus::NO_SAFE_FRONTIER);
      }
    }

    const ExplorerStats &stats = core_->stats();
    std::size_t pvbsm_root_count = 0;
    std::size_t pvbsm_submap_count = 0;
    if (pvbsm_memory_)
    {
      std::lock_guard<std::mutex> lock(pvbsm_mutex_);
      pvbsm_root_count = pvbsm_memory_->stats().root_count;
      pvbsm_submap_count = pvbsm_memory_->stats().submap_count;
    }
    ROS_INFO_STREAM_THROTTLE(
        1.0, "[ DAIB Explorer ] map=" << stats.free_cells << " free/"
        << stats.occupied_cells << " occupied/" << stats.frontier_cells
        << " frontier(raw)/" << stats.valid_frontier_cells << " valid/"
        << stats.rejected_stale_frontiers << " stale, cluster="
        << stats.frontier_components << " components/"
        << stats.rejected_small_clusters << " small_rejected/"
        << stats.frontier_clusters << " valid, cluster_ms="
        << stats.last_cluster_ms << ", "
        << stats.candidates_scored << " candidates, reject="
        << stats.rejected_no_viewpoint << " no_viewpoint/"
        << stats.rejected_distance << " distance/"
        << stats.rejected_vertical_distance << " vertical/"
        << stats.rejected_heading << " heading/"
        << stats.rejected_known_free_path << " known_path/"
        << stats.rejected_failed_goal << " failed_goal"
        << ", visited=" << stats.visited_cells
        << ", exploration_memory=" << stats.exploration_memory_cells
        << " cells/" << stats.stable_exploration_memory_cells
        << " stable, history=" << stats.historical_clusters_checked
        << " checked/" << stats.historical_clusters_observed
        << " observed/" << stats.rejected_historical_clusters
        << " rejected/" << stats.historical_probe_cells << " probes"
        // Compatibility aliases: existing log parsers can keep consuming
        // observed/submaps, but both now come from the single PVBSM memory.
        << ", observed=" << pvbsm_root_count
        << ", submaps=" << pvbsm_submap_count
        << ", memory_source=pvbsm"
        << ", lio_ema=" << stats.smoothed_lio_time_ms << " ms"
        << ", update=" << stats.last_update_ms << " ms"
        << ", plan=" << stats.last_plan_ms << " ms"
        << ", budget_scale=" << stats.budget_scale
        << ", cycles=" << stats.map_updates << " map/"
        << stats.goal_status_checks << " blocked-check/"
        << stats.frontier_update_cycles << " frontier/"
        << stats.goal_evaluation_cycles << " goal/"
        << stats.long_term_update_cycles << " memory"
        << ", budgets=" << stats.effective_raycasts << " rays/"
        << stats.effective_frontier_updates << " frontier/"
        << stats.effective_frontier_evaluations << " candidates"
        << ", pvbsm=" << stats.pvbsm_scored_candidates << " scored/"
        << stats.pvbsm_unseen_candidates << " unseen"
        << ", pvbsm_best_adjustment="
        << stats.pvbsm_best_adjustment
        << ", mcsvf=" << stats.safe_viewpoint_candidates << " viewpoints/"
        << stats.reachability_checks << " active_goal_astar_checks/"
        << stats.reachability_budget_exhaustions << " exhausted"
        << ", stalled=" << stats.stalled_goals
        << ", failed_cooldown=" << stats.failed_goals_in_cooldown);
  }
};

}  // namespace daib_explorer

int main(int argc, char **argv)
{
  ros::init(argc, argv, "daib_explorer");
  daib_explorer::ExplorerNode node;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
