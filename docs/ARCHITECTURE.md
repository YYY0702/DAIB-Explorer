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
candidate evaluation at 1 Hz, and trajectory-visit maintenance at 1 Hz. The
remaining 1 Hz core task stores only visited trajectory cells;
structural coverage and submap ownership come from PVBSM. These stages share
one serialized core rather than independent worker threads, so rate separation
does not introduce map races.

PVBSM affects only the 1 Hz candidate score. Candidate positions are queried
as one batch under one short memory lock. The score continuously penalizes
well-covered submaps and exact represented roots, rewards unseen submaps, and
adds a bounded structural-support term during degeneracy. The rolling
occupancy map remains the sole collision authority.

## Three map layers

1. **Rolling occupancy map**: bounded by `planning_map_radius_m`, updated at
   `map_update_rate_hz`, and used for collision/frontier queries.
2. **Active frontier set**: updated only around cells whose occupancy state
   changed. Per-update work is capped by `frontier_update_budget`.
3. **Long-term exploration memory**: coarse visited trajectory cells plus
   DAIB-PVBSM. Its detailed plane/residual cache is bounded, while a compact
   per-submap observed-root bitmap survives detailed-record demotion. With the
   default 8x8x8 block, the coverage bitmap is 512 bits (64 bytes) per
   represented submap. PVBSM is the only structural submap representation;
   Explorer no longer builds a second point-cloud-derived submap summary.
   This prevents repeated coverage without copying FAST-LIVO2's estimator
   octree or visual feature map.

FAST-LIVO2 local-map retirement and PVBSM forgetting are deliberately
separate. A root leaving the estimator window arrives as one archived
plane/residual summary and remains observed. When the detailed-record capacity
is reached, Explorer removes only detailed geometry and retains the observed
bit and submap coverage count. Only a hard-deletion record, source-session
reset, or mission reset clears long-term coverage.

The first two layers can be discarded/rebuilt if the explorer restarts; SLAM
localization is unaffected. Persisting and exchanging the third layer is the
future dual-UAV extension.

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
