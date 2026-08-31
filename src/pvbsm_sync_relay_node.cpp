#include <daib_explorer/PvbsmSync.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/UInt64MultiArray.h>

#include <algorithm>
#include <cstdint>
#include <deque>
#include <map>
#include <random>
#include <string>
#include <unordered_map>

namespace
{

uint32_t Fnv1a(const sensor_msgs::PointCloud2 &message)
{
  uint32_t value = 2166136261U;
  const auto mix = [&value](uint8_t byte)
  {
    value ^= byte;
    value *= 16777619U;
  };
  const auto mix32 = [&mix](uint32_t word)
  {
    for (unsigned shift = 0; shift < 32U; shift += 8U)
      mix(static_cast<uint8_t>((word >> shift) & 0xffU));
  };
  mix32(message.height);
  mix32(message.width);
  mix32(message.point_step);
  mix32(message.row_step);
  mix(static_cast<uint8_t>(message.is_bigendian));
  mix(static_cast<uint8_t>(message.is_dense));
  for (char character : message.header.frame_id)
    mix(static_cast<uint8_t>(character));
  for (const sensor_msgs::PointField &field : message.fields)
  {
    for (char character : field.name)
      mix(static_cast<uint8_t>(character));
    mix32(field.offset);
    mix(field.datatype);
    mix32(field.count);
  }
  for (uint8_t byte : message.data) mix(byte);
  return value;
}

class PvbsmSyncRelay
{
public:
  PvbsmSyncRelay() : private_nh_("~"), rng_(0xDA1B2026U)
  {
    int robot_id = 0;
    private_nh_.param("robot_id", robot_id, robot_id);
    robot_id_ = static_cast<uint16_t>(
        std::max(0, std::min(65534, robot_id)));
    private_nh_.param(
        "local_delta_topic", local_delta_topic_,
        std::string("/daib_slam/pvbsm_delta"));
    private_nh_.param(
        "peer_delta_topic", peer_delta_topic_,
        std::string("/daib_coexplore/peer_pvbsm_delta"));
    private_nh_.param(
        "sync_bus_topic", sync_bus_topic_,
        std::string("/daib_coexplore/pvbsm_sync"));
    private_nh_.param(
        "stats_topic", stats_topic_,
        std::string("/daib_coexplore/pvbsm_sync_stats"));
    private_nh_.param("cache_packets", cache_packets_, 512);
    private_nh_.param("max_nack_span", max_nack_span_, 64);
    private_nh_.param("reorder_capacity", reorder_capacity_, 128);
    private_nh_.param("summary_rate_hz", summary_rate_hz_, 1.0);
    private_nh_.param("drop_probability", drop_probability_, 0.0);
    cache_packets_ = std::max(8, cache_packets_);
    max_nack_span_ = std::max(1, max_nack_span_);
    reorder_capacity_ = std::max(8, reorder_capacity_);
    summary_rate_hz_ = std::max(0.2, summary_rate_hz_);
    drop_probability_ = std::max(0.0, std::min(0.95, drop_probability_));
    drop_distribution_ =
        std::bernoulli_distribution(drop_probability_);

    session_ = ros::WallTime::now().toNSec();
    bus_pub_ = nh_.advertise<daib_explorer::PvbsmSync>(
        sync_bus_topic_, 64, false);
    peer_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        peer_delta_topic_, 8, false);
    stats_pub_ = nh_.advertise<std_msgs::UInt64MultiArray>(
        stats_topic_, 1, true);
    local_sub_ = nh_.subscribe(
        local_delta_topic_, 8, &PvbsmSyncRelay::localCallback, this,
        ros::TransportHints().tcpNoDelay());
    bus_sub_ = nh_.subscribe(
        sync_bus_topic_, 128, &PvbsmSyncRelay::busCallback, this,
        ros::TransportHints().tcpNoDelay());
    summary_timer_ = nh_.createTimer(
        ros::Duration(1.0 / summary_rate_hz_),
        &PvbsmSyncRelay::summaryTimer, this);

    ROS_INFO_STREAM("[ DAIB PVBSM Sync ] robot=" << robot_id_
                    << ", session=" << session_
                    << ", cache=" << cache_packets_
                    << ", simulated_drop=" << drop_probability_);
  }

private:
  struct ReceiveState
  {
    uint64_t session = 0;
    uint64_t next_sequence = 1;
    std::map<uint64_t, daib_explorer::PvbsmSync> pending;
    ros::WallTime last_nack;
  };

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber local_sub_;
  ros::Subscriber bus_sub_;
  ros::Publisher bus_pub_;
  ros::Publisher peer_pub_;
  ros::Publisher stats_pub_;
  ros::Timer summary_timer_;
  std::string local_delta_topic_;
  std::string peer_delta_topic_;
  std::string sync_bus_topic_;
  std::string stats_topic_;
  uint16_t robot_id_ = 0;
  uint64_t session_ = 0;
  uint64_t next_send_sequence_ = 1;
  int cache_packets_ = 512;
  int max_nack_span_ = 64;
  int reorder_capacity_ = 128;
  double summary_rate_hz_ = 1.0;
  double drop_probability_ = 0.0;
  std::deque<daib_explorer::PvbsmSync> send_cache_;
  std::unordered_map<uint16_t, ReceiveState> receivers_;
  std::mt19937 rng_;
  std::bernoulli_distribution drop_distribution_;
  uint64_t sent_data_ = 0;
  uint64_t delivered_data_ = 0;
  uint64_t simulated_drops_ = 0;
  uint64_t checksum_rejects_ = 0;
  uint64_t nack_sent_ = 0;
  uint64_t replayed_data_ = 0;
  uint64_t cache_misses_ = 0;

  void localCallback(const sensor_msgs::PointCloud2ConstPtr &message)
  {
    daib_explorer::PvbsmSync packet;
    packet.header.stamp = ros::Time::now();
    packet.header.frame_id = message->header.frame_id;
    packet.packet_type = daib_explorer::PvbsmSync::DATA;
    packet.source_robot_id = robot_id_;
    packet.target_robot_id = daib_explorer::PvbsmSync::BROADCAST;
    packet.session = session_;
    packet.sequence = next_send_sequence_++;
    packet.newest_sequence = packet.sequence;
    packet.checksum = Fnv1a(*message);
    packet.delta = *message;
    send_cache_.push_back(packet);
    while (send_cache_.size() > static_cast<std::size_t>(cache_packets_))
      send_cache_.pop_front();
    bus_pub_.publish(packet);
    ++sent_data_;
  }

  void busCallback(const daib_explorer::PvbsmSyncConstPtr &message)
  {
    if (message->packet_type == daib_explorer::PvbsmSync::NACK)
    {
      if (message->target_robot_id == robot_id_) resend(*message);
      return;
    }
    if (message->source_robot_id == robot_id_) return;
    if (message->target_robot_id != daib_explorer::PvbsmSync::BROADCAST &&
        message->target_robot_id != robot_id_)
      return;

    ReceiveState &state = receivers_[message->source_robot_id];
    if (state.session != message->session)
    {
      state = ReceiveState{};
      state.session = message->session;
    }

    if (message->packet_type == daib_explorer::PvbsmSync::SUMMARY)
    {
      if (message->newest_sequence >= state.next_sequence)
        requestMissing(
            message->source_robot_id, state,
            state.next_sequence, message->newest_sequence);
      return;
    }
    if (message->packet_type != daib_explorer::PvbsmSync::DATA) return;
    if (drop_probability_ > 0.0 && drop_distribution_(rng_))
    {
      ++simulated_drops_;
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[ DAIB PVBSM Sync ] simulated drop source="
                   << message->source_robot_id << " seq="
                   << message->sequence);
      return;
    }
    if (message->checksum != Fnv1a(message->delta))
    {
      ++checksum_rejects_;
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[ DAIB PVBSM Sync ] checksum reject source="
                   << message->source_robot_id << " seq="
                   << message->sequence);
      requestMissing(
          message->source_robot_id, state,
          message->sequence, message->sequence);
      return;
    }
    if (message->sequence < state.next_sequence) return;
    if (message->sequence > state.next_sequence)
    {
      if (state.pending.size() <
          static_cast<std::size_t>(reorder_capacity_))
        state.pending.emplace(message->sequence, *message);
      requestMissing(
          message->source_robot_id, state,
          state.next_sequence, message->sequence - 1U);
      return;
    }
    deliver(*message, state);
    while (true)
    {
      const auto pending = state.pending.find(state.next_sequence);
      if (pending == state.pending.end()) break;
      const daib_explorer::PvbsmSync packet = pending->second;
      state.pending.erase(pending);
      deliver(packet, state);
    }
  }

  void deliver(
      const daib_explorer::PvbsmSync &packet, ReceiveState &state)
  {
    peer_pub_.publish(packet.delta);
    ++delivered_data_;
    state.next_sequence = packet.sequence + 1U;
  }

  void requestMissing(
      uint16_t peer, ReceiveState &state,
      uint64_t first, uint64_t last)
  {
    if (last < first) return;
    const ros::WallTime now = ros::WallTime::now();
    if (!state.last_nack.isZero() &&
        (now - state.last_nack).toSec() < 0.05)
      return;
    state.last_nack = now;
    daib_explorer::PvbsmSync request;
    request.header.stamp = ros::Time::now();
    request.packet_type = daib_explorer::PvbsmSync::NACK;
    request.source_robot_id = robot_id_;
    request.target_robot_id = peer;
    request.session = session_;
    request.request_from = first;
    request.request_to = std::min(
        last, first + static_cast<uint64_t>(max_nack_span_ - 1));
    bus_pub_.publish(request);
    ++nack_sent_;
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[ DAIB PVBSM Sync ] request missing source=" << peer
                 << " range=" << request.request_from << ".."
                 << request.request_to);
  }

  void resend(const daib_explorer::PvbsmSync &request)
  {
    if (request.request_to < request.request_from) return;
    bool found_any = false;
    for (const daib_explorer::PvbsmSync &packet : send_cache_)
    {
      if (packet.sequence < request.request_from ||
          packet.sequence > request.request_to)
        continue;
      daib_explorer::PvbsmSync replay = packet;
      replay.target_robot_id = request.source_robot_id;
      replay.header.stamp = ros::Time::now();
      bus_pub_.publish(replay);
      found_any = true;
      ++replayed_data_;
    }
    if (!found_any)
    {
      ++cache_misses_;
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "[ DAIB PVBSM Sync ] requested range outside cache: "
                   << request.request_from << ".." << request.request_to);
    }
  }

  void summaryTimer(const ros::TimerEvent &)
  {
    daib_explorer::PvbsmSync summary;
    summary.header.stamp = ros::Time::now();
    summary.packet_type = daib_explorer::PvbsmSync::SUMMARY;
    summary.source_robot_id = robot_id_;
    summary.target_robot_id = daib_explorer::PvbsmSync::BROADCAST;
    summary.session = session_;
    summary.newest_sequence = next_send_sequence_ - 1U;
    bus_pub_.publish(summary);
    std_msgs::UInt64MultiArray stats;
    stats.data = {
        sent_data_, delivered_data_, simulated_drops_, checksum_rejects_,
        nack_sent_, replayed_data_, cache_misses_,
        static_cast<uint64_t>(send_cache_.size())};
    stats_pub_.publish(stats);
  }
};

} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pvbsm_sync_relay");
  PvbsmSyncRelay relay;
  ros::spin();
  return 0;
}
