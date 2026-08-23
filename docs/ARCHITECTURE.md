# Architecture and data ownership

```text
LiDAR / IMU / Camera
        |
        v
FAST-LIVO2YYY (10 Hz target, localization critical)
  - LIO/VIO and DAIB-LIO
  - visual keyframe selection
  - bounded local/long-term visual feature memory
        |
        | standard ROS1 messages, no shared source library
        | synchronized odom + bounded planning cloud
        | + degeneracy + LIO runtime
        | + 1 Hz DAIB-PVBSM delta
        v
DAIB-Explorer (default 10 Hz, best effort)
  - rolling occupancy map
  - incremental frontier set
  - coarse trajectory-visit memory
  - optional mission-lifetime stable-observation statistics (default off)
  - bounded planar/residual-voxel long-term geometry cache
  - degeneracy-aware information-budgeted goal selection
        |
        | PoseStamped goal + planning cloud + ready watchdog
        v
ego-planner-swarmYYY
  - DAIB goal watchdog and generation deduplication
  - local occupied-cloud collision map
  - dynamically feasible B-spline generation/replanning
        |
        v
PX4
```

Within DAIB-Explorer, only occupancy integration and current-goal blockage
checks follow the 10 Hz input. Dirty-frontier processing runs at 2 Hz, goal
candidate evaluation at 1 Hz, and trajectory-visit maintenance at 1 Hz.
When explicitly enabled for validation, mission-lifetime observation memory
reuses the bounded ray samples from each accepted cloud, with per-frame
deduplication and a multi-frame stability threshold. It is disabled by default
and does not define the authoritative long-term map. Structural coverage and
submap ownership come from PVBSM. These stages share one serialized core rather
than independent worker threads, so rate separation does not introduce map
races.

PVBSM affects only the 1 Hz candidate score. Candidate positions are queried
as one batch under one short memory lock. The score continuously penalizes
well-covered submaps and exact represented roots, rewards unseen submaps, and
adds a bounded structural-support term during degeneracy. The rolling
occupancy map remains the sole collision authority.

Before scoring, DAIB-MCSVF groups the active frontier set into 18-neighbor
connected components and searches up to eight wall-clear known-free viewpoints
for each retained component. It prioritises viewpoints close to the current
flight height, rejects candidates beyond the 120-degree heading bound, and
uses continuous travel-heading and arrival-yaw costs within the feasible set.
Bounded A* is reserved for the 2 Hz active-goal blockage check rather than run
for every candidate. Goal stagnation and failed-point cooldown suppress simple
local repetition; an explicit global maze-escape tier is not yet implemented.
These operations stay outside FAST-LIVO2.

## Authoritative map layers and optional statistics

1. **Rolling occupancy map**: bounded by `planning_map_radius_m`, updated at
   `map_update_rate_hz`, and used for collision/frontier queries.
2. **Active frontier set**: updated only around cells whose occupancy state
   changed. Per-update work is capped by `frontier_update_budget`.
3. **Structural exploration memory**: coarse visited trajectory cells plus
   DAIB-PVBSM. Its detailed plane/residual cache is bounded, while a compact
   per-submap observed-root bitmap survives detailed-record demotion. With the
   default 8x8x8 block, the coverage bitmap is 512 bits (64 bytes) per
   represented submap. PVBSM remains the only structural submap
   representation.

The optional mission observation statistics store only coarse counters and
evidence flags. They are disabled by default, cleared on restart, and do not
copy FAST-LIVO2's estimator octree or visual feature map. They therefore do not
constitute a fourth authoritative map layer.

FAST-LIVO2 local-map retirement and PVBSM forgetting are deliberately
separate. A root leaving the estimator window arrives as one archived
plane/residual summary and remains observed. When the detailed-record capacity
is reached, Explorer removes only detailed geometry and retains the observed
bit and submap coverage count. Only a hard-deletion record, source-session
reset, or mission reset clears long-term coverage.

The rolling occupancy and active frontier layers can be discarded and rebuilt
if Explorer restarts; SLAM localization is unaffected. PVBSM is the structural
layer intended for later persistence and dual-UAV exchange.

## Compute isolation

The node uses a queue of one for planning clouds and performs work only from a
timer. LIO runtime is smoothed locally and changes only the explorer's ray,
frontier-update and candidate-evaluation budgets. No callback into FAST-LIVO2
exists. If one 10 Hz update overlaps the next timer tick, the new tick is
discarded rather than queued.

## Multi-UAV extension point

The implemented 64-byte PVBSM record carries `source_id`, revision, root voxel
coordinates and submap edge length. The receiver derives a spatial submap key,
rejects stale per-root revisions and keeps remote/lightweight geometry outside
the high-frequency local rolling map. A later radio transport still needs
frame alignment, packet integrity, missing-revision recovery and conflict
fusion before records from another UAV are accepted.
