# DAIB-Explorer

Degeneracy-Aware Information-Budgeted Exploration for resource-constrained
UAVs. This ROS1 package owns exploration occupancy, incremental frontiers,
coverage memory, submap summaries and single-UAV goal selection. FAST-LIVO2
remains responsible only for localization/mapping and publishes observations
through standard ROS messages.

## Why it is a separate process

- FAST-LIVO2 stays on its real-time LIO/VIO path even if frontier extraction is
  slow or the explorer crashes.
- The cloud subscriber queue is `1`; a 10 Hz timer consumes only the latest
  cloud, so exploration never accumulates stale frames.
- EGO-Swarm can later consume `/daib_explorer/goal` and
  `/daib_explorer/planning_cloud` without depending on FAST-LIVO2 headers.
- A future multi-UAV transport can exchange submap summaries without changing
  the SLAM process.

The internal schedule is deliberately multi-rate:

| Stage | Default rate |
|---|---:|
| Occupancy integration and current-goal blocking check | 10 Hz |
| Incremental dirty-frontier processing/publication | 2 Hz |
| Goal candidate evaluation/replanning | 1 Hz |
| Coverage memory and submap maintenance | 1 Hz |

Dirty cells accumulate in a deduplicated set between frontier cycles, so the
lower frontier rate does not discard occupancy changes.

## ROS contract

Inputs:

| Topic | Type | Meaning |
|---|---|---|
| `/daib_slam/odom` | `nav_msgs/Odometry` | LIO pose with sensor timestamp |
| `/daib_slam/planning_cloud` | `sensor_msgs/PointCloud2` | Bounded world-frame LIO cloud with the same timestamp |
| `/daib_slam/degenerate` | `std_msgs/Bool` | Lidar geometric degeneracy |
| `/daib_slam/degeneracy_score` | `std_msgs/Float64` | Normalized minimum eigenvalue |
| `/daib_slam/lio_runtime_ms` | `std_msgs/Float64` | Current LIO latency |

Outputs:

| Topic | Type | Consumer |
|---|---|---|
| `/daib_explorer/goal` | `geometry_msgs/PoseStamped` | EGO-Swarm adapter; `header.seq` carries the goal generation |
| `/daib_explorer/frontiers` | `sensor_msgs/PointCloud2` | RViz / validation |
| `/daib_explorer/planning_cloud` | `sensor_msgs/PointCloud2` | Rolling occupied-voxel centers for local planning |
| `/daib_explorer/ready` | `std_msgs/Bool` | Planner watchdog; latched state plus 1 Hz heartbeat |
| `/daib_explorer/state` | `std_msgs/String` | Validation/debug |
| `/daib_explorer/generation` | `std_msgs/UInt64` | Suppress duplicate replans |

The EGO planning cloud is a view of occupied cells within 12 m of the current
vehicle position, capped at 6000 points by default. This limits ROS
serialization and local-planner work without pruning the Explorer's rolling
occupancy map, frontier set or long-term coverage memory.

The odometry and cloud are generated from the same LIO update and must have
matching timestamps and `header.frame_id`. The current FAST-LIVO2 configuration
publishes both in `camera_init`; a mismatch is rejected instead of silently
planning in mixed coordinate frames.

`/daib_explorer/goal` and `/daib_explorer/generation` carry the same generation
number. Embedding it in `PoseStamped.header.seq` makes the goal and its
deduplication key atomic for the planner adapter; the UInt64 topic is retained
for monitoring and future multi-UAV transports.

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
trajectory. Do not connect it directly to PX4. Use the DAIB adapter and
resource-constrained launch in `ego-planner-swarmYYY` to validate the goal,
consume the local occupied cloud and generate a collision-free B-spline.
The resulting `PositionCommand` is still a controller-facing interface rather
than a direct PX4 setpoint connection.
