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
        v
DAIB-Explorer (default 10 Hz, best effort)
  - rolling occupancy map
  - incremental frontier set
  - coverage memory and submap summaries
  - degeneracy-aware information-budgeted goal selection
        |
        | PoseStamped goal + planning cloud + ready watchdog
        v
EGO-Swarm adapter / trajectory planner (future repository/module)
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
3. **Long-term exploration memory**: coarse visited and LiDAR-observed cells
   plus versioned submap summaries with full 3D anchor orientation. It prevents
   repeated coverage without copying FAST-LIVO2's visual feature map.

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

Each submap ID reserves the upper 16 bits for `robot_id` and lower bits for a
monotonic local ID. A later transport message should contain ID, version,
anchor, bounds, coarse coverage and compressed occupancy/frontier deltas. Merge
that data into a separate remote-memory index; never insert remote voxels into
the high-frequency local rolling map without frame alignment and version
checks.
