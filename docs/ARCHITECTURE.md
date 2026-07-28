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
  - coverage memory and submap summaries
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
candidate evaluation at 1 Hz, and long-term coverage/submap maintenance at
1 Hz. These stages share one serialized core rather than independent worker
threads, so rate separation does not introduce map races.

## Three map layers

1. **Rolling occupancy map**: bounded by `planning_map_radius_m`, updated at
   `map_update_rate_hz`, and used for collision/frontier queries.
2. **Active frontier set**: updated only around cells whose occupancy state
   changed. Per-update work is capped by `frontier_update_budget`.
3. **Long-term exploration memory**: coarse visited and LiDAR-observed cells,
   versioned submap summaries, and the bounded DAIB-PVBSM plane/residual cache.
   It prevents repeated coverage and retains compact geometry without copying
   FAST-LIVO2's estimator octree or visual feature map.

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
