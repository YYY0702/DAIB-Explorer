# DAIB-Explorer

Degeneracy-Aware Information-Budgeted Exploration for resource-constrained
UAVs. This ROS1 package owns exploration occupancy, incremental frontiers,
coarse trajectory-visit memory, PVBSM structural memory and single-UAV goal
selection. FAST-LIVO2 remains responsible only for localization/mapping and
publishes observations through standard ROS messages.

Explorer contains an optional mission-lifetime coarse observation-memory
implementation. It is disabled in the default flight profile and does not
participate in goal selection. When enabled for validation, it records cells
traversed by the already budgeted LiDAR rays and requires observations from
multiple distinct cloud frames before a cell becomes stable; it is never used
as current free-space or collision evidence.

## Why it is a separate process

- FAST-LIVO2 stays on its real-time LIO/VIO path even if frontier extraction is
  slow or the explorer crashes.
- The cloud subscriber queue is `1`; a 10 Hz timer consumes only the latest
  cloud, so exploration never accumulates stale frames.
- EGO-Swarm can later consume `/daib_explorer/goal` and
  `/daib_explorer/planning_cloud` without depending on FAST-LIVO2 headers.
- A future multi-UAV transport can exchange PVBSM deltas without changing the
  high-rate local planning path.

The internal schedule is deliberately multi-rate:

| Stage | Default rate |
|---|---:|
| Occupancy integration and current-goal blocking check | 10 Hz |
| Incremental dirty-frontier processing/publication | 2 Hz |
| Goal candidate evaluation/replanning | 1 Hz |
| Coarse trajectory-visit memory | 1 Hz |

When explicitly enabled, mission-lifetime observation memory updates with every
accepted cloud because it reuses the same bounded ray set as occupancy
integration. Per-frame deduplication prevents repeated points in one cloud from
satisfying the stable observation threshold.

Dirty cells accumulate in a deduplicated set between frontier cycles, so the
lower frontier rate does not discard occupancy changes.

Goal selection uses DAIB-MCSVF (Motion-Constrained Safe-Viewpoint Frontier):
raw frontier voxels are grouped into 18-neighbor connected components with at
least ten planning voxels. Frontier validity itself remains based on face-only
6-neighbor occupancy. Up to eight observation poses are searched in known free
space for each cluster, prioritising poses within 0.5 m of the current flight
height. A free frontier voxel is used as a conservative fallback when sparse
rays cannot support the ideal standoff pose. The candidate layer retains the
required clearance, distance, relative-height, known-free-ratio and 120-degree
heading constraints. Inside those bounds, information and PVBSM novelty are
balanced against continuous distance, travel-heading and arrival-yaw costs.

Mission-lifetime observation memory is an optional staged capability and is
disabled by default. After its growth, coordinate consistency and overlap with
PVBSM coverage have been validated, `exploration_memory_enabled` may be enabled
in record-only mode while `exploration_memory_filter_enabled` remains false.
Only a later validated configuration may allow the filter to suppress clusters
whose unknown-side probes are at least 70 percent stably observed. The raw
valid cluster topic remains unchanged for before/after diagnosis.

Vertical motion is limited relative to the current UAV pose rather than a
fixed map altitude: with the default `max_goal_vertical_distance_m: 3.0`, a
candidate must satisfy `current_z - 3 m <= goal_z <= current_z + 3 m`. Explorer
does not impose an absolute altitude layer or a task-area geofence.

The accepted goal follows a persistent FUEL-style task policy: it is not
replaced merely because a higher-scoring frontier appears. Replacement occurs
only after arrival, persistently blocked active-goal reachability, or 15 s
without at least 0.25 m of progress. Blocked or stalled goals enter a 30 s
spatial cooldown so replanning cannot immediately select the same failed area.
The old absolute `goal_timeout_s` is retained only for compatibility and is
disabled by default.

## ROS contract

Inputs:

| Topic | Type | Meaning |
|---|---|---|
| `/daib_slam/odom` | `nav_msgs/Odometry` | LIO pose with sensor timestamp |
| `/daib_slam/planning_cloud` | `sensor_msgs/PointCloud2` | Bounded world-frame LIO cloud with the same timestamp |
| `/daib_slam/degenerate` | `std_msgs/Bool` | Lidar geometric degeneracy |
| `/daib_slam/degeneracy_score` | `std_msgs/Float64` | Normalized minimum eigenvalue |
| `/daib_slam/lio_runtime_ms` | `std_msgs/Float64` | Current LIO latency |
| `/daib_slam/pvbsm_delta` | `sensor_msgs/PointCloud2` | 1 Hz planar/residual-voxel submap delta |
| `/daib_decision/command` | `DaibDecisionCommand` | Budget profile and explicit reselect/escape command |

Outputs:

| Topic | Type | Consumer |
|---|---|---|
| `/daib_explorer/goal` | `geometry_msgs/PoseStamped` | EGO-Swarm bridge; timestamp and pose identify the goal |
| `/daib_explorer/frontiers` | `sensor_msgs/PointCloud2` | RViz / validation |
| `/daib_explorer/valid_cluster_frontiers` | `sensor_msgs/PointCloud2` | All frontier voxels in connected components with at least `min_frontier_cluster_cells`; independent of viewpoint and goal selection |
| `/daib_explorer/selected_cluster_frontiers` | `sensor_msgs/PointCloud2` | Complete frontier cluster that produced the current goal; empty when no goal is valid |
| `/daib_explorer/planning_cloud` | `sensor_msgs/PointCloud2` | Rolling occupied-voxel centers for local planning |
| `/daib_explorer/ready` | `std_msgs/Bool` | Planner watchdog; latched state plus 1 Hz heartbeat |
| `/daib_explorer/state` | `std_msgs/String` | Validation/debug |
| `/daib_explorer/generation` | `std_msgs/UInt64` | Planner acknowledgement and monitoring |
| `/daib_explorer/pvbsm_memory_stats` | `std_msgs/UInt64MultiArray` | Persistent coverage plus bounded detailed-geometry statistics |
| `/daib_decision/module_status` | `DaibModuleStatus` | Native Explorer readiness/health heartbeat |
| `/daib_decision/event` | `DaibEvent` | Goal available/reached/blocked/stalled/no-frontier lifecycle |
| `/daib_decision/action_ack` | `DaibActionAck` | Completion of reselect/escape commands targeted at Explorer |

The EGO planning cloud is a view of occupied cells within 12 m of the current
vehicle position, capped at 6000 points by default. This limits ROS
serialization and local-planner work without pruning the Explorer's rolling
occupancy map or frontier set.

Each retained frontier cluster can contribute up to
`max_viewpoints_per_cluster` known-free viewpoints. Candidates within
`viewpoint_same_height_tolerance_m` of the current odometry height are used
whenever any are available; bounded vertical candidates are considered only
when no same-height candidate survives. Distance, wall clearance, line of
sight, known-free path ratio and failed-goal cooldown remain hard constraints.
Heading is a score cost instead of a hard rejection.

The goal quaternion contains zero roll and pitch. Its yaw points from the
selected viewpoint toward that viewpoint's frontier/unknown observation
target, rather than along the path from the current vehicle position to the
viewpoint. Downstream bridge, planner and controller support for this arrival
yaw must be validated separately.

DAIB-PVBSM runs on a separate low-rate subscriber callback. It stores
versioned plane primitives and residual voxels by source/root identity, rejects
stale updates, applies deletion records and groups roots into spatial
submaps. At the 1 Hz goal-evaluation stage it rewards frontiers in unseen
submaps and penalizes candidates in well-covered submaps or already represented
root voxels. During LiDAR degeneracy, nearby retained structure contributes a
small observability-support bonus. It does not feed the 10 Hz collision map, so
malformed or delayed long-term-map traffic cannot alter the current
flight-safety path.

The added score is:

`unseen bonus - submap coverage penalty - observed-root penalty + degenerate structural-support bonus`.

`pvbsm_root_voxel_size_m` must equal FAST-LIVO2 `lio/voxel_size`;
`robot_id` and `pvbsm_submap_edge_roots` must match FAST-LIVO2
`daib_pvbsm/robot_id` and `daib_pvbsm/submap_edge_roots`.

The odometry and cloud are generated from the same LIO update and must have
matching timestamps and `header.frame_id`. The current FAST-LIVO2 configuration
publishes both in `camera_init`; a mismatch is rejected instead of silently
planning in mixed coordinate frames.

`/daib_explorer/generation` is the application-level goal generation used for
monitoring and acknowledgement. ROS1 owns `PoseStamped.header.seq`, so the
planning bridge must not use that transport field as the DAIB generation. The
bridge identifies duplicate goals by timestamp and pose and consumes the
separate generation topic for telemetry.

## Build and run

Place this repository in a catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/YYY0702/DAIB-Explorer-Degeneracy-Aware-Information-Budgeted-Exploration-for-Resource-Constrained-UAVs.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch daib_explorer explorer.launch
```

For bag playback, pass `use_sim_time:=true` and start the bag with `--clock`.
Start FAST-LIVO2 first or at the same time; the explorer publishes
`/daib_explorer/ready=false` until fresh odometry and cloud data arrive.
Use [`docs/RUNTIME_VALIDATION.md`](docs/RUNTIME_VALIDATION.md) for the complete
pre-EGO acceptance test.

## Safety boundary

The published goal is a task-level destination, not a dynamically feasible
trajectory. Do not connect it directly to PX4. Use the DAIB planning bridge and
resource-constrained launch in `ego-planner-swarmYYY` to validate the goal,
consume the local occupied cloud and generate a collision-free B-spline.
The resulting `PositionCommand` is still a controller-facing interface rather
than a direct PX4 setpoint connection.
