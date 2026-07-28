#include "daib_explorer/pvbsm_memory.h"

#include <algorithm>
#include <tuple>
#include <unordered_set>

namespace daib_explorer
{
namespace
{
template <typename T>
void HashCombine(std::size_t &seed, const T &value)
{
  seed ^= std::hash<T>{}(value) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
}

int64_t FloorDivide(int32_t value, uint8_t divisor)
{
  const int64_t safe_divisor = std::max<int64_t>(1, divisor);
  int64_t quotient = static_cast<int64_t>(value) / safe_divisor;
  const int64_t remainder = static_cast<int64_t>(value) % safe_divisor;
  if (remainder < 0) --quotient;
  return quotient;
}

struct SubmapKey
{
  uint16_t source_id = 0;
  int64_t x = 0;
  int64_t y = 0;
  int64_t z = 0;
  bool operator==(const SubmapKey &other) const
  {
    return source_id == other.source_id &&
           x == other.x && y == other.y && z == other.z;
  }
};

struct SubmapKeyHash
{
  std::size_t operator()(const SubmapKey &key) const
  {
    std::size_t seed = 0;
    HashCombine(seed, key.source_id);
    HashCombine(seed, key.x);
    HashCombine(seed, key.y);
    HashCombine(seed, key.z);
    return seed;
  }
};
} // namespace

PvbsmMemory::PvbsmMemory(std::size_t max_records)
    : max_records_(std::max<std::size_t>(1, max_records))
{
}

bool PvbsmMemory::RootKey::operator==(const RootKey &other) const
{
  return source_id == other.source_id &&
         x == other.x && y == other.y && z == other.z;
}

std::size_t PvbsmMemory::RootKeyHash::operator()(const RootKey &key) const
{
  std::size_t seed = 0;
  HashCombine(seed, key.source_id);
  HashCombine(seed, key.x);
  HashCombine(seed, key.y);
  HashCombine(seed, key.z);
  return seed;
}

void PvbsmMemory::eraseRoot(const RootKey &key)
{
  const auto root = roots_.find(key);
  if (root == roots_.end()) return;
  roots_.erase(root);
}

void PvbsmMemory::clearSource(uint16_t source_id)
{
  bool had_state = false;
  for (auto root = roots_.begin(); root != roots_.end();)
  {
    if (root->first.source_id == source_id)
    {
      had_state = true;
      root = roots_.erase(root);
    }
    else
    {
      ++root;
    }
  }
  for (auto revision = revisions_.begin(); revision != revisions_.end();)
  {
    if (revision->first.source_id == source_id)
    {
      had_state = true;
      revision = revisions_.erase(revision);
    }
    else
    {
      ++revision;
    }
  }
  if (had_state) ++stats_.source_session_resets;
}

void PvbsmMemory::applyDelta(const std::vector<PvbsmRecord> &records)
{
  std::unordered_set<uint16_t> session_start_sources;
  for (const PvbsmRecord &record : records)
    if ((record.flags & 2U) != 0U)
      session_start_sources.insert(record.source_id);
  for (uint16_t source_id : session_start_sources)
  {
    bool previous_session_advanced = false;
    for (const auto &revision : revisions_)
      if (revision.first.source_id == source_id && revision.second > 1U)
      {
        previous_session_advanced = true;
        break;
      }
    if (previous_session_advanced) clearSource(source_id);
  }

  std::unordered_map<RootKey, std::vector<PvbsmRecord>, RootKeyHash> grouped;
  for (const PvbsmRecord &record : records)
  {
    const RootKey key{
        record.source_id, record.root[0], record.root[1], record.root[2]};
    grouped[key].push_back(record);
  }

  for (auto &entry : grouped)
  {
    const RootKey &key = entry.first;
    std::vector<PvbsmRecord> &root_records = entry.second;
    if (root_records.empty()) continue;
    const uint32_t revision = root_records.front().revision;
    const auto previous = revisions_.find(key);
    if (previous != revisions_.end() && revision <= previous->second)
    {
      ++stats_.rejected_stale_root_updates;
      continue;
    }

    revisions_[key] = revision;
    age_queue_.push_back({key, revision});
    const bool deleted = std::any_of(
        root_records.begin(), root_records.end(),
        [](const PvbsmRecord &record) { return record.kind == 2; });
    if (deleted)
    {
      eraseRoot(key);
      ++stats_.deleted_roots;
      ++stats_.accepted_root_updates;
      continue;
    }

    root_records.erase(
        std::remove_if(
            root_records.begin(), root_records.end(),
            [revision, &key](const PvbsmRecord &record) {
              return record.revision != revision ||
                     record.source_id != key.source_id ||
                     (record.kind != 0 && record.kind != 1);
            }),
        root_records.end());
    if (root_records.empty()) continue;
    if (root_records.size() > max_records_)
      root_records.resize(max_records_);
    roots_[key] = std::move(root_records);
    ++stats_.accepted_root_updates;
  }

  enforceCapacity();
  refreshStats();
}

void PvbsmMemory::enforceCapacity()
{
  std::size_t record_count = 0;
  for (const auto &entry : roots_) record_count += entry.second.size();
  while (record_count > max_records_ && !age_queue_.empty())
  {
    const RootVersion oldest = age_queue_.front();
    age_queue_.pop_front();
    const auto revision = revisions_.find(oldest.key);
    if (revision == revisions_.end() ||
        revision->second != oldest.revision)
      continue;
    const auto root = roots_.find(oldest.key);
    if (root == roots_.end()) continue;
    record_count -= root->second.size();
    roots_.erase(root);
    revisions_.erase(oldest.key);
    ++stats_.capacity_evictions;
  }
}

void PvbsmMemory::refreshStats()
{
  stats_.root_count = roots_.size();
  stats_.record_count = 0;
  stats_.plane_count = 0;
  stats_.residual_count = 0;
  std::unordered_set<SubmapKey, SubmapKeyHash> submaps;
  for (const auto &entry : roots_)
  {
    stats_.record_count += entry.second.size();
    if (entry.second.empty()) continue;
    const PvbsmRecord &first = entry.second.front();
    submaps.insert({
        first.source_id,
        FloorDivide(first.root[0], first.submap_edge_roots),
        FloorDivide(first.root[1], first.submap_edge_roots),
        FloorDivide(first.root[2], first.submap_edge_roots)});
    for (const PvbsmRecord &record : entry.second)
    {
      if (record.kind == 0) ++stats_.plane_count;
      if (record.kind == 1) ++stats_.residual_count;
    }
  }
  stats_.submap_count = submaps.size();
}

} // namespace daib_explorer
