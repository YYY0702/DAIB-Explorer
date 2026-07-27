#include "daib_explorer/explorer_core.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

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
#include <tf/transform_datatypes.h>

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

    odom_sub_ = nh_.subscribe(odom_topic_, 5, &ExplorerNode::odomCallback, this);
    cloud_sub_ =
        nh_.subscribe(cloud_topic_, 1, &ExplorerNode::cloudCallback, this);
    degenerate_sub_ = nh_.subscribe(
        degenerate_topic_, 1, &ExplorerNode::degenerateCallback, this);
    score_sub_ =
        nh_.subscribe(score_topic_, 1, &ExplorerNode::scoreCallback, this);
    runtime_sub_ =
        nh_.subscribe(runtime_topic_, 1, &ExplorerNode::runtimeCallback, this);

    goal_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1, true);
    frontiers_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(frontiers_topic_, 1, true);
    planning_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(planning_cloud_topic_, 1);
    ready_pub_ = nh_.advertise<std_msgs::Bool>(ready_topic_, 1, true);
    state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 1, true);
    generation_pub_ =
        nh_.advertise<std_msgs::UInt64>(generation_topic_, 1, true);

    map_timer_ = nh_.createTimer(
        ros::Duration(1.0 / map_update_rate_hz_),
        &ExplorerNode::mapTimerCallback, this);
    publishReady(false);
    ROS_INFO_STREAM("[ DAIB Explorer ] isolated node ready; odom="
                    << odom_topic_ << ", cloud=" << cloud_topic_
                    << ", map_update_rate=" << map_update_rate_hz_ << " Hz");
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber degenerate_sub_;
  ros::Subscriber score_sub_;
  ros::Subscriber runtime_sub_;
  ros::Publisher goal_pub_;
  ros::Publisher frontiers_pub_;
  ros::Publisher planning_cloud_pub_;
  ros::Publisher ready_pub_;
  ros::Publisher state_pub_;
  ros::Publisher generation_pub_;
  ros::Timer map_timer_;
  std::unique_ptr<ExplorerCore> core_;

  std::mutex input_mutex_;
  std::mutex update_mutex_;
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
  std::string goal_topic_ = "/daib_explorer/goal";
  std::string frontiers_topic_ = "/daib_explorer/frontiers";
  std::string planning_cloud_topic_ = "/daib_explorer/planning_cloud";
  std::string ready_topic_ = "/daib_explorer/ready";
  std::string state_topic_ = "/daib_explorer/state";
  std::string generation_topic_ = "/daib_explorer/generation";
  double map_update_rate_hz_ = 10.0;
  double input_timeout_s_ = 1.0;
  double ready_heartbeat_rate_hz_ = 1.0;
  double max_input_stamp_skew_s_ = 0.2;
  int max_cloud_points_to_convert_ = 6000;
  int max_published_frontiers_ = 2000;
  double planning_output_radius_m_ = 12.0;
  int max_published_planning_points_ = 6000;
  bool ready_ = false;
  bool ready_initialized_ = false;

  void readParameters(ExplorerConfig &config)
  {
    private_nh_.param("topics/odom", odom_topic_, odom_topic_);
    private_nh_.param("topics/cloud", cloud_topic_, cloud_topic_);
    private_nh_.param(
        "topics/degenerate", degenerate_topic_, degenerate_topic_);
    private_nh_.param("topics/degeneracy_score", score_topic_, score_topic_);
    private_nh_.param("topics/lio_runtime_ms", runtime_topic_, runtime_topic_);
    private_nh_.param("topics/goal", goal_topic_, goal_topic_);
    private_nh_.param("topics/frontiers", frontiers_topic_, frontiers_topic_);
    private_nh_.param(
        "topics/planning_cloud", planning_cloud_topic_, planning_cloud_topic_);
    private_nh_.param("topics/ready", ready_topic_, ready_topic_);
    private_nh_.param("topics/state", state_topic_, state_topic_);
    private_nh_.param(
        "topics/generation", generation_topic_, generation_topic_);

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
    map_update_rate_hz_ = std::max(0.2, map_update_rate_hz_);
    input_timeout_s_ = std::max(0.1, input_timeout_s_);
    ready_heartbeat_rate_hz_ = std::max(0.1, ready_heartbeat_rate_hz_);
    max_input_stamp_skew_s_ = std::max(0.0, max_input_stamp_skew_s_);
    max_cloud_points_to_convert_ = std::max(64, max_cloud_points_to_convert_);
    max_published_frontiers_ = std::max(1, max_published_frontiers_);
    planning_output_radius_m_ = std::max(1.0, planning_output_radius_m_);
    max_published_planning_points_ =
        std::max(1, max_published_planning_points_);

    private_nh_.param("robot_id", config.robot_id, config.robot_id);
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
    private_nh_.param("max_coverage_points_per_update",
                      config.max_coverage_points_per_update,
                      config.max_coverage_points_per_update);
    private_nh_.param("submap_translation_threshold_m",
                      config.submap_translation_threshold_m,
                      config.submap_translation_threshold_m);
    private_nh_.param("submap_rotation_threshold_deg",
                      config.submap_rotation_threshold_deg,
                      config.submap_rotation_threshold_deg);
    private_nh_.param("replan_interval_s",
                      config.replan_interval_s,
                      config.replan_interval_s);
    private_nh_.param("goal_min_hold_time_s",
                      config.goal_min_hold_time_s,
                      config.goal_min_hold_time_s);
    private_nh_.param("goal_blocked_confirm_updates",
                      config.goal_blocked_confirm_updates,
                      config.goal_blocked_confirm_updates);
    private_nh_.param("same_goal_tolerance_m",
                      config.same_goal_tolerance_m,
                      config.same_goal_tolerance_m);
    private_nh_.param(
        "goal_timeout_s", config.goal_timeout_s, config.goal_timeout_s);
    private_nh_.param("goal_reached_distance_m",
                      config.goal_reached_distance_m,
                      config.goal_reached_distance_m);
    private_nh_.param("min_goal_distance_m",
                      config.min_goal_distance_m,
                      config.min_goal_distance_m);
    private_nh_.param("max_goal_distance_m",
                      config.max_goal_distance_m,
                      config.max_goal_distance_m);
    private_nh_.param("max_goal_vertical_distance_m",
                      config.max_goal_vertical_distance_m,
                      config.max_goal_vertical_distance_m);
    private_nh_.param("goal_switch_margin",
                      config.goal_switch_margin,
                      config.goal_switch_margin);
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
    if (cloud_sequence == processed_cloud_sequence_) return;
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
    const uint64_t previous_frontier_cycle =
        core_->stats().frontier_update_cycles;
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
      if (decision.valid)
      {
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
        ROS_INFO_STREAM("[ DAIB Explorer ] generation=" << decision.generation
                        << ", state=" << decision.state
                        << ", reason=" << decision.reason
                        << ", goal=(" << decision.position.x << ", "
                        << decision.position.y << ", " << decision.position.z
                        << "), plan=" << decision.planning_time_ms << " ms");
      }
    }

    const ExplorerStats &stats = core_->stats();
    ROS_INFO_STREAM_THROTTLE(
        1.0, "[ DAIB Explorer ] map=" << stats.free_cells << " free/"
        << stats.occupied_cells << " occupied/" << stats.frontier_cells
        << " frontier, visited=" << stats.visited_cells
        << ", observed=" << stats.observed_cells
        << ", submaps=" << stats.submap_count
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
        << stats.effective_frontier_evaluations << " candidates");
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
