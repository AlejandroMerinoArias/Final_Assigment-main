![Schematics of the Drone](DroneforCave.png)

## Sensor topics & TF frames (audit)

This repository already exposes the Unity simulator sensors through `simulation/launch/unity_ros.launch.py`
and wires the TF tree in `simulation/launch/simulation.launch.py`. Below is the confirmed mapping for
perception nodes so downstream packages can subscribe to consistent topics and frames.

### Unity sensor remappings (ROS 2)

The `unity_ros` node remaps Unity topics into ROS 2 sensor topics:

| Unity stream | Remapped ROS 2 topic (default) | Message |
| --- | --- | --- |
| `Quadrotor/Sensors/RGBCameraLeft/image_raw` | `/realsense/rgb/image_rect_raw_left` | `sensor_msgs/Image` |
| `Quadrotor/Sensors/RGBCameraLeft/camera_info` | `/realsense/rgb/camera_info_left` | `sensor_msgs/CameraInfo` |
| `Quadrotor/Sensors/RGBCameraRight/image_raw` | `/realsense/rgb/image_rect_raw_right` | `sensor_msgs/Image` |
| `Quadrotor/Sensors/RGBCameraRight/camera_info` | `/realsense/rgb/camera_info_right` | `sensor_msgs/CameraInfo` |
| `Quadrotor/Sensors/DepthCamera/image_raw` | `/realsense/depth/image_rect_raw` | `sensor_msgs/Image` (`16UC1`) |
| `Quadrotor/Sensors/DepthCamera/camera_info` | `/realsense/depth/camera_info` | `sensor_msgs/CameraInfo` |
| `Quadrotor/IMU` | `/interpolate_imu/imu` | `sensor_msgs/Imu` |
| `Quadrotor/TrueState/pose` | `/true_pose` | `geometry_msgs/PoseStamped` |
| `Quadrotor/TrueState/twist` | `/true_twist` | `geometry_msgs/TwistStamped` |

These remappings are defined in `unity_ros.launch.py` and should be used as-is by perception nodes unless
you explicitly override launch arguments with custom topic names.【F:ros2_ws/src/simulation/launch/unity_ros.launch.py†L22-L52】

### TF frame conventions

The simulator publishes TF frames and static transforms so perception nodes can safely assume:

- `world` is the global frame.
- `true_body` is the TF child frame for the simulator's true pose (`world -> true_body`).
- Camera frames are attached to the body via the `camera` and `depth_camera` static frames.

Static TFs are configured in `simulation.launch.py`:

| TF parent | TF child | Purpose |
| --- | --- | --- |
| `/Quadrotor/TrueState` | `/true_body` | Links Unity true-state to ROS `true_body` frame |
| `/true_body` | `/camera` | RGB camera frame for left/right cameras |
| `/true_body` | `/depth_camera` | Depth camera frame |
| `/camera` | `/Quadrotor/RGBCameraLeft` | Left RGB camera |
| `/camera` | `/Quadrotor/RGBCameraRight` | Right RGB camera |
| `/depth_camera` | `/Quadrotor/DepthCamera` | Depth camera |

These definitions live in `simulation.launch.py` and should be the canonical TF tree for perception
processing (RGB, depth, IMU, mapping).【F:ros2_ws/src/simulation/launch/simulation.launch.py†L107-L151】

### Frame usage in state publishers

The Unity true-state parser publishes pose and twist messages stamped in the `body` frame, while TF
broadcasting uses `world -> true_body`. Downstream perception should rely on TF for frame transforms
and prefer `true_body` (TF) over the `body` string in message headers when building the map or fusing
detections.【F:ros2_ws/src/simulation/src/true_state_parser.h†L49-L92

### State estimation input selection

While bootstrapping the perception stack, prefer the **corrupted state estimate** topics so your
pipeline matches the intended noise conditions:

- `/current_state_est` (primary `nav_msgs/Odometry` input for mapping/VIO)
- `/pose_est` and `/twist_est` if you need pose/twist separately

These are produced by `state_estimate_corruptor_node` from `/true_pose` and `/true_twist`, and the
same node also republishes `current_state` for the controller pipeline.【F:ros2_ws/src/simulation/src/state_estimate_corruptor_node.cpp†L32-L149】

For debugging only, you may temporarily use `/true_pose` and `/true_twist` (or their derived TF)
to isolate perception errors, but the baseline should be `/current_state_est` to ensure robustness.

### Mapping package (depth-to-voxel occupancy)

The new `mapping_pkg` provides a minimal depth-based mapping node that:

- Uses `depth_image_proc` to generate a point cloud from the depth image and camera info.
- Subscribes to `/realsense/depth/points` from `depth_image_proc`.
- Uses the state estimate (`/current_state_est`) as the world-frame anchor.
- Transforms points into the `world` frame and feeds them into **OctoMap** via `octomap_server`.

Outputs:

- `mapping/points_world` (`sensor_msgs/PointCloud2`): world-frame point cloud.
- OctoMap outputs from `octomap_server` (binary/full map topics, occupancy markers).

See `mapping_pkg/mapping_node.py` for parameters such as `pointcloud_topic`, `output_topic`,
`downsample`, and `max_range_m`, and the launch file for OctoMap configuration.【F:ros2_ws/src/mapping_pkg/mapping_pkg/mapping_node.py†L66-L159】【F:ros2_ws/src/mapping_pkg/launch/mapping.launch.py†L1-L41】