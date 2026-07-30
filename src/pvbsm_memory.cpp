#include "daib_explorer/pvbsm_memory.h"

#include <algorithm>
#include <cmath>
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

uint8_t SafeSubmapEdge(uint8_t edge)
{
  // The protocol defaults to 8. Bound malformed/mismatched messages so one
  // record cannot force an excessive bitmap allocation.
  return std::max<uint8_t>(1, std::min<uint8_t>(16, edge));
}

int64_t FloorModulo(int32_t value, uint8_t divisor)
{
  const int64_t safe_divisor = std::max<int64_t>(1, divisor);
  int64_t remainder = static_cast<int64_t>(value) % safe_divisor;
  if (remainder < 0) remainder += safe_divisor;
  return remainder;
}

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

bool PvbsmMemory::SubmapKey::operator==(const SubmapKey &other) const
{
  return source_id == other.source_id &&
         x == other.x && y == other.y && z == other.z;
}

std::size_t PvbsmMemory::SubmapKeyHash::operator()(
    const SubmapKey &key) const
{
  std::size_t seed = 0;
  HashCombine(seed, key.source_id);
  HashCombine(seed, key.x);
  HashCombine(seed, key.y);
  HashCombine(seed, key.z);
  return seed;
}

PvbsmMemory::SubmapKey PvbsmMemory::submapKey(
    const PvbsmRecord &record) const
{
  const uint8_t edge = SafeSubmapEdge(record.submap_edge_roots);
  return {
      record.source_id,
      FloorDivide(record.root[0], edge),
      FloorDivide(record.root[1], edge),
      FloorDivide(record.root[2], edge)};
}

PvbsmMemory::SubmapKey PvbsmMemory::submapKey(
    const RootKey &root, uint8_t submap_edge_roots) const
{
  const uint8_t edge = SafeSubmapEdge(submap_edge_roots);
  return {
      root.source_id,
      FloorDivide(root.x, edge),
      FloorDivide(root.y, edge),
      FloorDivide(root.z, edge)};
}

std::size_t PvbsmMemory::submapRootIndex(
    const RootKey &root, uint8_t submap_edge_roots) const
{
  const uint8_t edge = SafeSubmapEdge(submap_edge_roots);
  const std::size_t x =
      static_cast<std::size_t>(FloorModulo(root.x, edge));
  const std::size_t y =
      static_cast<std::size_t>(FloorModulo(root.y, edge));
  const std::size_t z =
      static_cast<std::size_t>(FloorModulo(root.z, edge));
  return (x * edge + y) * edge + z;
}

bool PvbsmMemory::markRootObserved(const PvbsmRecord &record)
{
  const RootKey root{
      record.source_id, record.root[0], record.root[1], record.root[2]};
  const uint8_t edge = SafeSubmapEdge(record.submap_edge_roots);
  SubmapEvidence &submap = submap_evidence_[submapKey(record)];
  submap.edge_roots = edge;
  const std::size_t bit_count =
      static_cast<std::size_t>(edge) * edge * edge;
  const std::size_t word_count = (bit_count + 63U) / 64U;
  if (submap.observed_words.size() != word_count)
    submap.observed_words.resize(word_count, 0U);
  const std::size_t index = submapRootIndex(root, edge);
  const std::size_t word = index / 64U;
  const uint64_t mask = uint64_t{1} << (index % 64U);
  if ((submap.observed_words[word] & mask) != 0U) return false;
  submap.observed_words[word] |= mask;
  ++submap.root_count;
  ++observed_root_count_;
  return true;
}

bool PvbsmMemory::clearRootObserved(
    const RootKey &root, uint8_t submap_edge_roots)
{
  const uint8_t edge = SafeSubmapEdge(submap_edge_roots);
  const SubmapKey key = submapKey(root, edge);
  auto submap = submap_evidence_.find(key);
  if (submap == submap_evidence_.end()) return false;
  const std::size_t index = submapRootIndex(root, edge);
  const std::size_t word = index / 64U;
  if (word >= submap->second.observed_words.size()) return false;
  const uint64_t mask = uint64_t{1} << (index % 64U);
  if ((submap->second.observed_words[word] & mask) == 0U) return false;
  submap->second.observed_words[word] &= ~mask;
  if (submap->second.root_count > 0) --submap->second.root_count;
  if (observed_root_count_ > 0) --observed_root_count_;
  if (submap->second.root_count == 0 &&
      submap->second.plane_count == 0 &&
      submap->second.residual_count == 0)
    submap_evidence_.erase(submap);
  return true;
}

void PvbsmMemory::addRootEvidence(
    const std::vector<PvbsmRecord> &records)
{
  if (records.empty()) return;
  markRootObserved(records.front());
  SubmapEvidence &submap = submap_evidence_[submapKey(records.front())];
  for (const PvbsmRecord &record : records)
  {
    ++record_count_;
    if (record.kind == 0)
    {
      ++plane_count_;
      ++submap.plane_count;
      submap.plane_confidence_sum +=
          std::max(0.0F, std::min(1.0F, record.confidence));
    }
    else if (record.kind == 1)
    {
      ++residual_count_;
      ++submap.residual_count;
    }
  }
}

void PvbsmMemory::removeRootEvidence(
    const std::vector<PvbsmRecord> &records,
    bool preserve_coverage)
{
  if (records.empty()) return;
  const SubmapKey key = submapKey(records.front());
  auto submap = submap_evidence_.find(key);
  for (const PvbsmRecord &record : records)
  {
    if (record_count_ > 0) --record_count_;
    if (record.kind == 0)
    {
      if (plane_count_ > 0) --plane_count_;
      if (submap != submap_evidence_.end())
      {
        if (submap->second.plane_count > 0)
          --submap->second.plane_count;
        submap->second.plane_confidence_sum = std::max(
            0.0,
            submap->second.plane_confidence_sum -
                std::max(0.0F, std::min(1.0F, record.confidence)));
      }
    }
    else if (record.kind == 1)
    {
      if (residual_count_ > 0) --residual_count_;
      if (submap != submap_evidence_.end() &&
          submap->second.residual_count > 0)
        --submap->second.residual_count;
    }
  }
  if (!preserve_coverage)
  {
    const RootKey root{
        records.front().source_id,
        records.front().root[0],
        records.front().root[1],
        records.front().root[2]};
    clearRootObserved(root, records.front().submap_edge_roots);
  }
}

bool PvbsmMemory::eraseRoot(
    const RootKey &key, bool preserve_coverage)
{
  const auto root = roots_.find(key);
  if (root == roots_.end()) return false;
  removeRootEvidence(root->second, preserve_coverage);
  roots_.erase(root);
  return true;
}

void PvbsmMemory::clearSource(uint16_t source_id)
{
  bool had_state = false;
  std::vector<RootKey> roots_to_remove;
  for (const auto &root : roots_)
    if (root.first.source_id == source_id)
      roots_to_remove.push_back(root.first);
  for (const RootKey &root : roots_to_remove)
  {
    had_state = true;
    eraseRoot(root, false);
  }
  for (auto submap = submap_evidence_.begin();
       submap != submap_evidence_.end();)
  {
    if (submap->first.source_id == source_id)
    {
      had_state = true;
      observed_root_count_ =
          submap->second.root_count > observed_root_count_
              ? 0
              : observed_root_count_ - submap->second.root_count;
      submap = submap_evidence_.erase(submap);
    }
    else
    {
      ++submap;
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
      if (!eraseRoot(key, false))
        clearRootObserved(key, root_records.front().submap_edge_roots);
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
    eraseRoot(key, true);
    roots_[key] = std::move(root_records);
    addRootEvidence(roots_[key]);
    ++stats_.accepted_root_updates;
  }

  enforceCapacity();
  compactAgeQueue();
  refreshStats();
}

void PvbsmMemory::enforceCapacity()
{
  while (record_count_ > max_records_ && !age_queue_.empty())
  {
    const RootVersion oldest = age_queue_.front();
    age_queue_.pop_front();
    const auto revision = revisions_.find(oldest.key);
    if (revision == revisions_.end() ||
        revision->second != oldest.revision)
      continue;
    const auto root = roots_.find(oldest.key);
    if (root == roots_.end()) continue;
    eraseRoot(oldest.key, true);
    ++stats_.capacity_evictions;
  }
}

void PvbsmMemory::compactAgeQueue()
{
  const std::size_t threshold = roots_.size() * 4U + 1024U;
  if (age_queue_.size() <= threshold) return;
  std::vector<RootVersion> current;
  current.reserve(roots_.size());
  for (const auto &root : roots_)
  {
    const auto revision = revisions_.find(root.first);
    if (revision != revisions_.end())
      current.push_back({root.first, revision->second});
  }
  std::sort(
      current.begin(), current.end(),
      [](const RootVersion &left, const RootVersion &right)
      {
        return left.revision < right.revision;
      });
  age_queue_.assign(current.begin(), current.end());
}

void PvbsmMemory::refreshStats()
{
  stats_.root_count = observed_root_count_;
  stats_.detailed_root_count = roots_.size();
  stats_.record_count = record_count_;
  stats_.plane_count = plane_count_;
  stats_.residual_count = residual_count_;
  stats_.submap_count = submap_evidence_.size();
}

std::vector<PvbsmExplorationHint> PvbsmMemory::queryExplorationHints(
    const std::vector<PvbsmQueryPoint> &points,
    uint16_t source_id,
    double root_voxel_size_m,
    uint8_t submap_edge_roots,
    std::size_t covered_root_target) const
{
  const double voxel_size = std::max(0.01, root_voxel_size_m);
  const uint8_t edge = SafeSubmapEdge(submap_edge_roots);
  const double coverage_target =
      static_cast<double>(std::max<std::size_t>(1, covered_root_target));
  std::vector<PvbsmExplorationHint> hints;
  hints.reserve(points.size());
  for (const PvbsmQueryPoint &point : points)
  {
    const RootKey root{
        source_id,
        static_cast<int32_t>(std::floor(point.x / voxel_size)),
        static_cast<int32_t>(std::floor(point.y / voxel_size)),
        static_cast<int32_t>(std::floor(point.z / voxel_size))};
    PvbsmExplorationHint hint;
    const SubmapKey submap{
        source_id,
        FloorDivide(root.x, edge),
        FloorDivide(root.y, edge),
        FloorDivide(root.z, edge)};
    const auto evidence = submap_evidence_.find(submap);
    if (evidence != submap_evidence_.end())
    {
      const std::size_t index = submapRootIndex(root, edge);
      const std::size_t word = index / 64U;
      if (word < evidence->second.observed_words.size())
      {
        const uint64_t mask = uint64_t{1} << (index % 64U);
        hint.root_observed =
            (evidence->second.observed_words[word] & mask) != 0U;
      }
      hint.submap_observed = true;
      hint.submap_coverage =
          std::min(1.0, evidence->second.root_count / coverage_target);
      const double structural_evidence =
          evidence->second.plane_confidence_sum +
          0.5 * evidence->second.residual_count;
      hint.structural_support =
          std::min(1.0, structural_evidence / coverage_target);
    }
    hints.push_back(hint);
  }
  return hints;
}

} // namespace daib_explorer
