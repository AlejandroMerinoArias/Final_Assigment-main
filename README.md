# Subterranean Sentinel: Robust Autonomous MAV Operation in Challenging 3D Environments
## Contributors

| Name | Main Contributions |
|------|------------------|
| Salaheldin Hassan | <ul><li>System Design & Architecture</li><li>FSM Design & Implementation</li><li>Exploration Pipeline Design & Implementation</li><li>RRT* Path Planning</li><li>ROSbag recording, and analysis using Jupyter Notebook</li><li>Cloud Gating</li></ul> |
| Alejandro Merino | <ul><li>Trajectory Generation</li><li>Trajectory Smoothing</li><li>Lantern Detection</li><li>Improve Exploration to find the last lantern</li></ul> |
| Ahmed Abdelghany | <ul><li>Mapping Pipeline (Octomap Configuration)</li></ul> |
| Verina Guirgis | <ul><li>Global Planning (A* Implementation)</li></ul> |
| Bassel Abdelhaleem | <ul><li>Perception Pipeline (Depth to Pointcloud conversion)</li></ul> |


## Prerequisites

### ROS 2 System Dependencies

The following ROS 2 packages must be installed from the system package manager before building the workspace:

```bash
# Perception pipeline (depth to pointcloud conversion)
sudo apt-get install -y ros-jazzy-depth-image-proc

# Mapping (OctoMap voxel grid server)
sudo apt-get install -y ros-jazzy-octomap-server ros-jazzy-octomap ros-jazzy-octomap-msgs

# TF2 utilities
sudo apt-get install -y ros-jazzy-tf2-ros ros-jazzy-tf2-sensor-msgs ros-jazzy-tf2-geometry-msgs

# Computer vision bridge
sudo apt-get install -y ros-jazzy-cv-bridge

# Colcon & Rosdep 
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep build-essential

# Optimization & Trajectory dependencies
sudo apt-get install -y libyaml-cpp-dev libnlopt-dev libgflags-dev

# Or install all at once:
sudo apt-get install -y \
    ros-jazzy-depth-image-proc \
    ros-jazzy-octomap-server \
    ros-jazzy-octomap \
    ros-jazzy-octomap-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-sensor-msgs \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-cv-bridge \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    libyaml-cpp-dev \
    libnlopt-dev \
    libgflags-dev
```

## Usage

### Launching the Simulation

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch simulation simulation.launch.py
```
### Running the Mission

1.  **Launch the Mission FSM**:
    ```bash
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch fsm mission.launch.py
    ```
    
    *Optional:* To automatically record a ROS bag of the flight (topics like trajectory, goals, and state), use:
    ```bash
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch fsm mission.launch.py record_bag:=true bag_filename:=my_test_run
    ```
    *(This will save to `rosbags/my_test_run_<timestamp>/.`)*

2.  **Start the Mission**:
    Publish an empty message to the `/mission/start` topic to trigger the FSM:
    ```bash
    source /opt/ros/jazzy/setup.bash
    ros2 topic pub --once /mission/start std_msgs/msg/Empty "{}"
    ```

### Building with Docker

If you prefer to avoid installing dependencies on your host system, you can use the provided `Dockerfile` to compile and run the project inside an isolated Ubuntu 24.04 (ROS 2 Jazzy) environment.

**1. Build the Docker Image:**
Run the following from the root of the repository:
```bash
docker build -t autonomous-sys:latest .
```
*(Note: If you run into DNS resolution issues pulling ROS packages, you can pass `--network=host` to the build command).*

**2. Run the Container:**
Once built, launch an interactive shell:
```bash
docker run -it --rm \
    --network=host \
    autonomous-sys:latest bash
```
From within the container environment, `ros2` commands are now fully operational, and the workspace `install/setup.bash` is sourced automatically via `entrypoint.sh`.

## System Architecture

This project addresses the classic autonomous robotics problem: **Perception $\rightarrow$ Mapping $\rightarrow$ Planning $\rightarrow$ Control.**

Based on the project requirements (Unity simulation, semantic camera, finding objects, 3D mapping) and best practices in ROS 2 architecture, the system is broken down into modular packages and nodes.

This design ensures **modularity** and **standard compliance**.

**Mission Flow:** The system starts with a pre-defined**waypoint** to navigate the drone to the cave entrance. Once at the entrance, it switches to **autonomous exploration** mode, using the exploration manager and global planner to navigate through the cave, detect lanterns, and build the map.

---

### 1. Workspace Structure (`src` folder)

The workspace is split logically across multiple packages.

*   `fsm` (Mission logic & launch)
*   `perception` (Lantern detection & pointcloud processing)
*   `mapping_pkg` (Octomap configuration & cloud gating)
*   `planning` (Global Path planning)
*   `exploring` (Frontier exploration logic)
*   `trajectory_generation` (Trajectory tracking & smoothing)
*   *(Provided packages: `simulation`, `unity_sim`, `controller_pkg`, `mav_msgs`, `mav_planning_msgs`)*

#### Actual Folder Structure

```text
ros2_ws/src/
│
├── fsm/                              # Mission State Machine (Main Brain) & Launch
│   ├── src/mission_fsm_node.cpp      
│   └── launch/mission.launch.py      # Entry point for mission
│
├── perception/                       # Perception Pipeline
│   └── src/lantern_detector_node.cpp # Semantic camera → lantern poses
│
├── mapping_pkg/                      # Octomap Configuration & Pointcloud gating
│   └── src/cloud_gate_node.cpp            # Cloud gating logic
│
├── planning/                         # Path Planning
│   ├── src/global_planner_node.cpp   # RRT* on octomap
│   └── src/global_planner_node_a.cpp # A* on octomap
│
├── exploring/                        # Exploration Logic
│   └── src/exploration_manager.cpp   # Frontier exploration logic
│
├── trajectory_generation/            # Trajectory & Control
│   └── src/trajectory_generation_node.cpp # Path → smooth trajectory
│
├── controller_pkg/                   # Quadrotor Controller (Provided)
│
├── simulation/                       # Simulation Launch & Config (World/Models)
│
├── unity_sim/                        # Unity-ROS Bridge (Provided)
│
├── mav_msgs/                         # Messages for MAV control
└── mav_planning_msgs/                # Messages for planning
```
---

### 2. System Architecture Diagram

```mermaid
graph TD
    subgraph Unity_Simulation ["Unity Simulation (Provided)"]
        Depth_Camera[Depth Camera]
        Semantic_Camera[Semantic Camera]
        State_Odometry["State/Odometry"]
    end

    subgraph Perception_Pkg [perception]
        depth_to_cloud["depth_image_proc"]
        pcl_transformer[pointcloud_transformer_node]
        lantern_detector[lantern_detector_node]
    end

    subgraph Mapping_Pkg [mapping_pkg]
        cloud_gate[cloud_gate]
        octomap_server["octomap_server"]
    end

    subgraph Planning_Pkg [planning]
        global_planner[global_planner]
    end

    subgraph Exploring_Pkg [exploring]
        exploration_manager[exploration_manager]
    end

    subgraph Trajectory_Pkg [trajectory_generation]
        trajectory_generation[trajectory_generation]
    end

    subgraph FSM_Pkg [fsm]
        mission_fsm_node["mission_fsm_node<br/>(Main Brain - FSM)"]
    end

    subgraph Controller_Pkg ["quadrotor_controller (Provided)"]
        controller_node[controller_node]
    end
    
    subgraph Unity_Sim_Input ["Unity Simulation (Drone Control)"]
        Unity_Input[Unity Input]
    end

    %% Connections
    Depth_Camera -->|/realsense/depth/image| depth_to_cloud
    Semantic_Camera -->|/realsense/semantic/image_rect_raw| lantern_detector
    
    depth_to_cloud -->|"/camera/depth/points<br/>(PointCloud2)"| pcl_transformer
    depth_to_cloud -->|"/camera/depth/points<br/>(PointCloud2)"| cloud_gate
    cloud_gate -->|"/gated_cloud"| octomap_server
    
    lantern_detector -->|"/detected_lanterns<br/>(PoseStamped)"| mission_fsm_node
    
    octomap_server -->|/octomap_binary| global_planner
    
    global_planner -->|"waypoints<br/>(nav_msgs/Path)"| trajectory_generation
    
    %% FSM Orchestration loop (Order tweaked for vertical layout separation)
    exploration_manager -- "2. Response: Goal Pose" --> mission_fsm_node
    mission_fsm_node -- "1. Service call: /exploration/get_goal" --> exploration_manager
    mission_fsm_node -- "3. Publish to /planner/goal" --> global_planner
    global_planner -- "4. /planner/status (Success/Fail)" --> mission_fsm_node
    
    trajectory_generation -->|"command/trajectory<br/>(MultiDOFJointTrajectory)"| controller_node
    
    State_Odometry -.->|/current_state_est| controller_node
    State_Odometry -.->|/current_state_est| trajectory_generation
    State_Odometry -.->|/current_state_est| global_planner
    
    controller_node -->|rotor_speed_cmds| Unity_Input
    
    %% Styles
    style Unity_Simulation fill:#e1f5fe,stroke:#01579b
    style Perception_Pkg fill:#fff3e0,stroke:#e65100
    style Mapping_Pkg fill:#e8f5e9,stroke:#2e7d32
    style Planning_Pkg fill:#f3e5f5,stroke:#7b1fa2
    style Exploring_Pkg fill:#f3e5f5,stroke:#7b1fa2
    style Trajectory_Pkg fill:#fce4ec,stroke:#c2185b
    style FSM_Pkg fill:#fce4ec,stroke:#c2185b
    style Controller_Pkg fill:#eeeeee,stroke:#212121
    style Unity_Sim_Input fill:#e1f5fe,stroke:#01579b
```

---

### 3. Package Documentation

The intricate details of each subsystem - including nodes, topics, services, parameters, and algorithms - are documented in their respective package `README.md` files:

*   **[`fsm`](ros2_ws/src/fsm/README.md):** The Mission State Machine. Manages Takeoff, GoTo Entrance, Exploration, Z-Retry Recovery, and Return behaviors.
*   **[`exploring`](ros2_ws/src/exploring/README.md):** Frontier exploration logic, dynamic stuck-detection, and goal evaluation.
*   **[`perception`](ros2_ws/src/perception/README.md):** Lantern localization algorithm, triangulating 3D poses from semantic and depth images.
*   **[`planning`](ros2_ws/src/planning/README.md):** Global path planning (A* & RRT*) on the OctoMap.
*   **[`mapping_pkg`](ros2_ws/src/mapping_pkg/README.md):** Pointcloud gating and `octomap_server` integration.
*   **[`trajectory_generation`](ros2_ws/src/trajectory_generation/README.md):** Generates minimum-snap smooth setpoints from the planner's waypoints.

---

### 4. The `tf2` Tree

The following coordinate frames are used in the system:

1.  **`world` / `map`**: The fixed starting frame.
2.  **`odom`**: Provided by the Simulation Bridge.
3.  **`base_link`**: The center of the drone.
4.  **`camera_link`**: The position of the camera relative to the drone.
    * A **Static Transform Publisher** in the launch file defines where the camera is mounted on the drone.


## Notes

- The system is designed to use the **noisy topics** (`/pose_est`, `/twist_est`, `/current_state_est`). These are the true poses/velocities with artificial noise added to simulate realistic conditions. The true topics (`/true_pose`, `/true_twist`, `/current_state`) are for debugging only.
- The Unity simulation must be running before launching the ROS 2 nodes, or nodes will wait for connections.
- State corruption is probabilistic and uses random walk for drift accumulation.
- All transforms account for Unity's left-handed coordinate system conversion to ROS's right-handed system.