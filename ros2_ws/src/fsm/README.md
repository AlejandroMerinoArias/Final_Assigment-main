# FSM Package (`fsm`)

This package contains the **mission finite-state machine** (`mission_fsm_node`) that coordinates takeoff, cave-entry navigation, exploration-goal dispatching, recovery logic, and return/landing.

## What the node does today

The FSM runs an 8-state mission loop:

1. `INIT` – wait for odometry (`/current_state_est`) and `/mission/start`.
2. `TAKEOFF` – publish a waypoint at current XY + configured takeoff altitude.
3. `GOTO_ENTRANCE` – publish waypoint to the fixed cave entrance (`[-320, 10, 18]`).
4. `EXPLORE` – request goals from `/exploration/get_goal`, dispatch to planner, monitor progress.
5. `REFINE_GOAL` – retry same XY with alternate Z values when planning/progress fails.
6. `LANTERN_FOUND` – de-duplicate and count lantern detections, then resume exploration.
7. `RETURN` – send planner goal back to start XY at takeoff altitude.
8. `LAND` – publish direct trajectory to start position at ground level.

### Mission completion criterion

`TARGET_LANTERN_COUNT` is currently hardcoded to **1** lantern, so the FSM leaves `EXPLORE` once one unique lantern is accepted.

---

## Key behaviors implemented in code

### 1) Exploration goals are service-driven

The FSM requests goals using async client calls to:

- `/exploration/get_goal` (`exploring/srv/GetExplorationGoal`)

It tracks `goal_request_pending_`, request epochs, request timeout, consecutive failures, and time since last successful goal to avoid duplicate/stale requests and log stuck goal-selection conditions.

### 2) Planner routing by parameter

`planner_type` selects the goal topic:

- `RRT` → `/planner/goal`
- `A_star` → `/planner_a/goal`

Planner feedback is consumed from `/planner/status`.

### 3) Z-refinement recovery is parameterized

On planner failure or low-progress/stuck cases, FSM enters `REFINE_GOAL` and builds a retry altitude list around the strategic goal Z using:

- `z_retry_max_attempts`
- `z_retry_step`

Pattern is center, then alternating lower/higher levels until attempts are exhausted.

### 4) Goal blacklisting after exhausted retries

If all refinement attempts fail, FSM publishes `geometry_msgs/PointStamped` to:

- `/exploration/blacklist_goal`

so exploration manager can reject that frontier in future requests.

### 5) Macroplanning graph + travel mode

When `macroplanning_enabled` is true, FSM maintains checkpoint graph nodes/edges, potential nodes from depth points, and travel-mode handoffs. It also publishes:

- `/fsm/checkpoint_markers` (graph visualization)
- `/exploration/priority_target` (single-edge priority cue)
- `/exploration/punishment_target` (one-shot predecessor repulsion cue)

### 6) Frozen watchdog fallback

If the drone stays within `freeze_reset_distance_threshold` longer than `freeze_timeout_s` while in `EXPLORE`/`REFINE_GOAL`, FSM triggers recovery:

- cancels current motion,
- resets exploration-side transient state,
- re-enters `EXPLORE` while preserving macroplanning graph structure.

---

## ROS interfaces

### Subscriptions

- `/current_state_est` (`nav_msgs/Odometry`)
- `/detected_lanterns` (`geometry_msgs/PoseStamped`)
- `/planner/status` (`std_msgs/String`)
- `/mission/start` (`std_msgs/Empty`)
- `/exploration/map_ready` (`std_msgs/Bool`)
- `/camera/depth/points_world` (`sensor_msgs/PointCloud2`)
- `waypoints` (`nav_msgs/Path`) – planner path monitor for travel/potential goals

### Publications

- `/command/trajectory` (`trajectory_msgs/MultiDOFJointTrajectory`)
- `waypoints` (`nav_msgs/Path`) – waypoint-path commands (takeoff/entrance)
- `/fsm/cancel` (`std_msgs/Empty`)
- `/fsm/state` (`std_msgs/String`)
- `/planner/goal` (`geometry_msgs/PoseStamped`)
- `/planner_a/goal` (`geometry_msgs/PoseStamped`)
- `/enable_mapping` (`std_msgs/Bool`)
- `/exploration/blacklist_goal` (`geometry_msgs/PointStamped`)
- `/exploration/priority_target` (`geometry_msgs/PointStamped`)
- `/exploration/punishment_target` (`geometry_msgs/PointStamped`)
- `/fsm/drone_marker` (`visualization_msgs/Marker`)
- `/fsm/checkpoint_markers` (`visualization_msgs/MarkerArray`)

### Service clients

- `/exploration/get_goal` (`exploring/srv/GetExplorationGoal`)

---

## Important parameters

### Declared by `mission_fsm_node`

- `planner_type` (node default: `A_star`)
- `takeoff_altitude` (node default: `2.0`)
- `lantern_dedup_threshold` (default `2.5`)
- `min_exploration_goal_distance` (default `2.0`)
- `explore_goal_selection_timeout` (default `60.0`)
- `explore_goal_selection_max_failures` (default `50`)
- `replan_interval_s` (default `3.0`)
- `z_retry_max_attempts` (default `3`)
- `z_retry_step` (default `1.0`)
- `macroplanning_enabled` (default `true`)
- `nodes_distance`, `node_radius`, `max_potential_node_range`
- `potential_node_backoff_distance`, `potential_angle_threshold_deg`
- `seen_point_timeout_s`, `goal_request_timeout_s`, `force_explorer_timeout_s`
- `single_edge_priority_reached_radius`
- `waypoint_obstacle_clearance`
- `freeze_reset_distance_threshold`, `freeze_timeout_s`

### Overrides used in `launch/mission.launch.py`

By default, mission launch overrides several FSM params:

- `planner_type:=RRT`
- `takeoff_altitude:=5.0`
- `min_exploration_goal_distance:=3.0`
- `z_retry_max_attempts:=5`
- `z_retry_step:=2.0`

---

## Launch

Run full mission stack (FSM + planners + perception + mapping + RViz):

```bash
ros2 launch fsm mission.launch.py
```

Useful overrides:

```bash
ros2 launch fsm mission.launch.py planner_type:=A_star takeoff_altitude:=4.0
ros2 launch fsm mission.launch.py record_bag:=true bag_filename:=mission_run
```

Start mission trigger:

```bash
ros2 topic pub --once /mission/start std_msgs/msg/Empty "{}"
```
