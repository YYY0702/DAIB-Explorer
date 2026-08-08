# Runtime validation before EGO-Swarm

This checklist validates the complete SLAM-to-exploration chain without
connecting a trajectory planner or flight controller.

## 1. Interface contract

Start FAST-LIVO2 and DAIB-Explorer, then verify:

```bash
rostopic hz /daib_slam/odom
rostopic hz /daib_slam/planning_cloud
rostopic echo -n 1 /daib_slam/odom/header
rostopic echo -n 1 /daib_slam/planning_cloud/header
rostopic echo /daib_explorer/ready
rostopic hz /daib_slam/pvbsm_delta
rostopic echo /daib_slam/pvbsm_stats
rostopic echo /daib_explorer/pvbsm_memory_stats
```

`/daib_explorer/pvbsm_memory_stats` keeps its original first ten fields and
appends `detailed_root_count` as field 11. Field 1 (`root_count`) is now the
persistent observed-root count; it must not fall merely because field 11 or
the detailed `record_count` is reduced by capacity demotion.

For sliding-retirement validation, use a trajectory long enough to cross the
enabled 100-root LIO window and verify:

- the FAST log reports non-zero `Delete ... root voxels`;
- PVBSM stats field 12 (`archived record count`) becomes non-zero;
- PVBSM deletion count stays zero during ordinary sliding;
- Explorer `root_count` does not drop when roots leave the LIO window;
- the archive backlog drains instead of growing continuously.

To exercise demotion without a very long run, temporarily override
`pvbsm_memory_max_records` with a test-only value such as 1000.
`record_count` and `detailed_root_count` must stay bounded, while
`root_count` and observed-submap coverage remain. Restore 200000 for the
normal onboard configuration.

Acceptance criteria:

- odometry and planning cloud are non-empty and update near the LIO rate;
- both use `camera_init`;
- corresponding timestamps differ by no more than 0.2 s (normally they are
  exactly equal);
- `ready` becomes `true` after both inputs arrive and becomes `false` within
  `input_timeout_s` after FAST-LIVO2 stops.
- PVBSM is approximately 1 Hz, its payload stays at or below 32 KiB normally
  and 48 KiB while degenerate, and the Explorer root/record counters grow.
- PVBSM pending roots may spike but must not grow continuously; exporter time
  should normally remain within `daib_pvbsm/export_budget_ms`.

## 2. Exploration behavior

```bash
rostopic hz /daib_explorer/frontiers
rostopic echo /daib_explorer/state
rostopic echo /daib_explorer/generation
rostopic echo /daib_explorer/goal
```

Acceptance criteria:

- free, occupied, frontier and visited counters grow in the log; the
  compatibility `observed` and `submaps` values grow after PVBSM deltas arrive,
  and the same line reports `memory_source=pvbsm`;
- the `cycles` log ratio approaches `10 map : 10 blocked-check : 2 frontier :
  1 goal : 1 memory` per second;
- a valid goal has frame `camera_init` and lies near a frontier;
- generation does not change during `goal_min_hold_time_s`;
- one transient occupied update does not switch the goal;
- a continuously blocked goal switches after
  `goal_blocked_confirm_updates`;
- a timed-out identical goal refreshes internally without increasing
  generation.
- the status log reports non-zero `pvbsm=... scored`; candidates in unseen
  submaps contribute a positive `pvbsm_best_adjustment`;
- replaying a previously mapped region reduces or makes the PVBSM adjustment
  negative instead of repeatedly rewarding the same frontier.
- every valid goal is in a known-free voxel and has at least
  `min_wall_clearance_m` from occupied voxel centers;
- normal logs report non-zero `mcsvf=... clusters/... viewpoints`, and goals
  obey the configured 60/120 degree heading tiers plus indoor/outdoor height
  and climb limits;
- a maze replay may increase `reachability_checks`, but
  `reachability_budget_exhaustions` should stay rare and the 1 Hz plan time
  must remain within the board budget;
- repeated local goals with little odometry displacement eventually publish
  reason `loop_escape`; after `loop_escape_duration_s`, the state returns to
  normal constraints.

## 3. Compute isolation

Run the same bag in three configurations:

1. FAST-LIVO2 only;
2. FAST-LIVO2 plus an idle DAIB-Explorer waiting for topics;
3. FAST-LIVO2 plus active DAIB-Explorer.

Record LIO mean/P95/P99 latency, total CPU, RSS and achieved rate. FAST-LIVO2
alone should match its pre-decoupling localization output except that the old
in-process exploration work is gone. With an explorer subscriber, FAST-LIVO2
serializes the bounded planning cloud and runs the low-rate, time-budgeted
PVBSM dirty-root exporter.

Kill DAIB-Explorer while the bag is running. `/aft_mapped_to_init`, the regular
FAST-LIVO2 map outputs and trajectory must continue. Restart the explorer; it
must rebuild its rolling map without restarting FAST-LIVO2.

## 4. Accuracy comparison

Use identical bag start time and parameters. Compare the FAST-LIVO2-only
trajectory against the decoupled run with APE/RPE. The explorer has no topic or
callback path back into FAST-LIVO2, so pose differences beyond normal
run-to-run floating-point scheduling variation indicate an external resource
or configuration problem, not an intended data-flow dependency.

## Known boundary

The rolling occupancy map is reconstructible and is intentionally lost if the
explorer process restarts. Coarse trajectory visits and PVBSM geometry
currently live in RAM. Persistent storage and the radio transport/fusion layer
are later extensions. None of these affect SLAM localization state.
