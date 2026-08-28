#pragma once

#include "daib_explorer/pvbsm_types.h"

#include <array>
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
  // root_count is persistent observed coverage. detailed_root_count is the
  // bounded plane/residual cache and may be smaller after demotion.
  std::size_t root_count = 0;
  std::size_t detailed_root_count = 0;
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
  std::vector<PvbsmExplorationHint> queryExplorationHints(
      const std::vector<PvbsmQueryPoint> &points,
      uint16_t source_id,
      double root_voxel_size_m,
      uint8_t submap_edge_roots,
      std::size_t covered_root_target) const;
  std::vector<uint16_t> sourceIds() const;
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

  struct SubmapKey
  {
    uint16_t source_id = 0;
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;
    bool operator==(const SubmapKey &other) const;
  };

  struct SubmapKeyHash
  {
    std::size_t operator()(const SubmapKey &key) const;
  };

  struct SubmapEvidence
  {
    std::size_t root_count = 0;
    std::size_t plane_count = 0;
    std::size_t residual_count = 0;
    double plane_confidence_sum = 0.0;
    uint8_t edge_roots = 8;
    // One bit per root in the logical submap. With the default 8^3 layout this
    // is only 64 bytes and survives detailed-geometry demotion.
    std::vector<uint64_t> observed_words;
  };

  std::size_t max_records_;
  std::unordered_map<RootKey, std::vector<PvbsmRecord>, RootKeyHash> roots_;
  std::unordered_map<RootKey, uint32_t, RootKeyHash> revisions_;
  std::unordered_map<SubmapKey, SubmapEvidence, SubmapKeyHash>
      submap_evidence_;
  std::deque<RootVersion> age_queue_;
  PvbsmMemoryStats stats_;
  std::size_t record_count_ = 0;
  std::size_t plane_count_ = 0;
  std::size_t residual_count_ = 0;
  std::size_t observed_root_count_ = 0;

  bool eraseRoot(const RootKey &key, bool preserve_coverage);
  void clearSource(uint16_t source_id);
  void enforceCapacity();
  void compactAgeQueue();
  void refreshStats();
  SubmapKey submapKey(const PvbsmRecord &record) const;
  SubmapKey submapKey(
      const RootKey &root, uint8_t submap_edge_roots) const;
  std::size_t submapRootIndex(
      const RootKey &root, uint8_t submap_edge_roots) const;
  bool markRootObserved(const PvbsmRecord &record);
  bool clearRootObserved(
      const RootKey &root, uint8_t submap_edge_roots);
  void addRootEvidence(const std::vector<PvbsmRecord> &records);
  void removeRootEvidence(
      const std::vector<PvbsmRecord> &records, bool preserve_coverage);
};

} // namespace daib_explorer
