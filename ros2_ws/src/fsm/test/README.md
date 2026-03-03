# Mission FSM Test Suite (`fsm/test`)

This directory contains GoogleTest-based tests for `control::MissionFsmNode`.

The tests instantiate the real FSM node plus a companion ROS 2 test node that:

- publishes simulated mission inputs (odometry, planner status, mission start, lantern detections),
- subscribes to `/fsm/state`, and
- asserts mission-state transitions.

---

## Build and run

```bash
# Build only the fsm package (including tests)
colcon build --packages-select fsm

# Run only fsm tests
colcon test --packages-select fsm

# Show detailed test results
colcon test-result --all --verbose
```

> Note: Package name is `fsm` (lowercase), matching `package.xml` / CMake setup.

---

## Test fixture overview

The `MissionFsmNodeTest` fixture creates:

- `node_`: the `MissionFsmNode` under test,
- `tester_node_`: helper ROS node for driving inputs and observing outputs,
- helper methods:
  - `PublishOdom(x, y, z)`
  - `PublishStartSignal()`
  - `Spin(duration)`

Observed output:

- `/fsm/state` (`std_msgs/msg/String`), captured in `last_state_`.

---

## Current test cases

1. **`InitialStateIsInit`**
   Verifies startup state is `INIT`, and odometry alone does not leave `INIT`.

2. **`TransitionsToTakeoffAfterOdomAndSignal`**
   Verifies transition to `TAKEOFF` happens only after both odometry and `/mission/start` are received.

3. **`TransitionsToGotoEntranceAfterTakeoffHeightReached`**
   Verifies `TAKEOFF -> GOTO_ENTRANCE` after reaching commanded takeoff altitude.

4. **`TransitionsToExploreAfterReachingEntrance`**
   Verifies `GOTO_ENTRANCE -> EXPLORE` when odometry reaches the configured cave-entrance target.

5. **`LanternDetectionInterruptsExplore`**
   Fast-forwards to `EXPLORE`, publishes lantern detections, and checks that the FSM enters `LANTERN_FOUND`.

6. **`PlannerFailureWithoutActiveGoalIsIgnored`**
   Verifies a `PLAN_FAILED` status does not force `REFINE_GOAL` when no goal is active.

7. **`FullMissionLifecycle`**
   Simulates an end-to-end mission flow and verifies transitions through takeoff, entrance, exploration, return, and landing phases.

---

## Important maintenance note

This README documents the **current test source** in `mission_fsm_node_test.cpp`.

FSM runtime behavior in production has evolved (e.g., lantern completion threshold, macroplanning behavior). If tests are updated, keep this README synchronized with:

- `ros2_ws/src/fsm/test/mission_fsm_node_test.cpp`, and
- `ros2_ws/src/fsm/src/mission_fsm_node.cpp`.
