#pragma once

namespace daib_explorer
{

struct PvbsmQueryPoint
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct PvbsmExplorationHint
{
  bool root_observed = false;
  bool submap_observed = false;
  double submap_coverage = 0.0;
  double structural_support = 0.0;
};

} // namespace daib_explorer
