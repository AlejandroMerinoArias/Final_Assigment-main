# Mission FSM Node Tests

This directory contains the unit and integration tests for the `MissionFsmNode` in the `FSM` package.

## Running Tests

You can run the tests using `colcon test`:

```bash
# Build the package (and tests)
colcon build --packages-select FSM

# Run tests
colcon test --packages-select FSM

# View detailed results
colcon test-result --all --verbose
```

## Test Structure

The tests use **GoogleTest** (`gtest`) and the `rclcpp` framework.

### `MissionFsmNodeTest` Fixture
A test fixture is set up to create an isolated environment for each test case:
- **`node_`**: The instance of `MissionFsmNode` under test.
- **`tester_node_`**: A companion node that inputs data into the FSM and subscribes to its outputs.
- **`PublishStartSignal()`**: Helper to simulate the user start signal.
- **Initialization**: Sets up publishers for Odometry, Lanterns, Planner Status, and Start Signal.

## Test Cases

### 1. `InitialStateIsInit`
Verifies that the node starts in the `INIT` state and remains there even if odometry is received, waiting for the explicit start signal.

### 2. `TransitionsToTakeoffAfterOdomAndSignal`
Verifies that the FSM transitions to `TAKEOFF` only after **both** valid odometry and the user start signal are received.

### 3. `TransitionsToGotoEntranceAfterTakeoffHeightReached`
Simulates the drone reaching the target takeoff altitude (2.0m) and asserts the transition to `GOTO_ENTRANCE`.

### 4. `TransitionsToExploreAfterReachingEntrance`
Simulates the drone reaching the cave entrance coordinates and asserts the transition to `EXPLORE`.

### 5. `LanternDetectionInterruptsExplore`
Verifies the interrupt logic:
- Puts the FSM in `EXPLORE` mode.
- Publishes a lantern detection.
- Asserts that the FSM immediately transitions to `LANTERN_FOUND`.
- Verifies it automatically returns to `EXPLORE` (or `RETURN`) after processing.

### 6. `PlannerFailureTriggersRefineGoal`
Verifies the recovery logic:
- Simulates a `PLAN_FAILED` message from the path planner.
- Asserts the FSM transitions to `REFINE_GOAL` to attempt a Z-height recovery.

### 7. `FullMissionLifecycle`
A comprehensive end-to-end simulation of the mission:
1.  **Start**: Init -> Takeoff -> Goto Entrance -> Explore.
2.  **Lantern Loop**: Simulates finding 4 distinct lanterns. Verifies the loop `EXPLORE` -> `LANTERN_FOUND` -> `EXPLORE`.
3.  **Completion**: Verifies that after the 4th lantern, the FSM transitions to `RETURN`.
4.  **End**: Simulates returning to start and landing.
