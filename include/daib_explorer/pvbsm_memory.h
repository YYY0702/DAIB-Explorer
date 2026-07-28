#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

namespace daib_explorer
{

struct PvbsmRecord
{
  float center[3] = {0.0F, 0.0F, 0.0F};
  float normal[3] = {0.0F, 0.0F, 0.0F};
  float extent_x = 0.0F;
  float extent_y = 0.0F;
  float thickness = 0.0F;
  float confidence = 0.0F;
  int32_t root[3] = {0, 0, 0};
  uint32_t revision = 0;
  uint16_t point_count = 0;
  uint16_t source_id = 0;
  uint8_t layer = 0;
  uint8_t kind = 0;
  uint8_t flags = 0;
  uint8_t submap_edge_roots = 8;
};

struct PvbsmMemoryStats
{
  std::size_t root_count = 0;
  std::size_t record_count = 0;
  std::size_t plane_count = 0;
  std::size_t residual_count = 0;
  std::size_t submap_count = 0;
  uint64_t accepted_root_updates = 0;
  uint64_t rejected_stale_root_updates = 0;
  uint64_t deleted_roots = 0;
  uint64_t capacity_evictions = 0;
  uint64_t source_session_resets = 0;
};

class PvbsmMemory
{
public:
  explicit PvbsmMemory(std::size_t max_records = 200000);

  void applyDelta(const std::vector<PvbsmRecord> &records);
  const PvbsmMemoryStats &stats() const { return stats_; }

private:
  struct RootKey
  {
    uint16_t source_id = 0;
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;
    bool operator==(const RootKey &other) const;
  };

  struct RootKeyHash
  {
    std::size_t operator()(const RootKey &key) const;
  };

  struct RootVersion
  {
    RootKey key;
    uint32_t revision = 0;
  };

  std::size_t max_records_;
  std::unordered_map<RootKey, std::vector<PvbsmRecord>, RootKeyHash> roots_;
  std::unordered_map<RootKey, uint32_t, RootKeyHash> revisions_;
  std::deque<RootVersion> age_queue_;
  PvbsmMemoryStats stats_;

  void eraseRoot(const RootKey &key);
  void clearSource(uint16_t source_id);
  void enforceCapacity();
  void refreshStats();
};

} // namespace daib_explorer
