#include "trajectory_generation/planner.hpp"

#include <utility>

TrajectoryPlanner::TrajectoryPlanner(rclcpp::Node & node)
: node_(&node),
  current_pose_(Eigen::Affine3d::Identity()),
  current_velocity_(Eigen::Vector3d::Zero()),
  current_angular_velocity_(Eigen::Vector3d::Zero()),
  odom_received_(false),
  max_v_(0.2),
  max_a_(0.2),
  max_ang_v_(0.0),
  max_ang_a_(0.0),
  waypoint_velocities_(),
  waypoint_accelerations_(),
  default_waypoint_velocity_(),
  default_waypoint_acceleration_()
{
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  To Do: Load Trajectory Parameters from parameter file (ROS2)
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //
  //
  // ~~~~ begin solution
  try {
    node_->declare_parameter<double>("max_v", max_v_);
    node_->declare_parameter<double>("max_a", max_a_);

    // Optional angular limits (not used yet in this template, but declared for completeness).
    node_->declare_parameter<double>("max_ang_v", max_ang_v_);
    node_->declare_parameter<double>("max_ang_a", max_ang_a_);

    node_->declare_parameter<std::vector<double>>(
      "waypoint_velocities", waypoint_velocities_);
    node_->declare_parameter<std::vector<double>>(
      "waypoint_accelerations", waypoint_accelerations_);
    node_->declare_parameter<std::vector<double>>(
      "default_waypoint_velocity", default_waypoint_velocity_);
    node_->declare_parameter<std::vector<double>>(
      "default_waypoint_acceleration", default_waypoint_acceleration_);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(),
      "Exception while declaring parameters: %s. Using defaults.", e.what());
  }

  try {
    (void)node_->get_parameter("max_v", max_v_);
    (void)node_->get_parameter("max_a", max_a_);
    (void)node_->get_parameter("max_ang_v", max_ang_v_);
    (void)node_->get_parameter("max_ang_a", max_ang_a_);
    (void)node_->get_parameter("waypoint_velocities", waypoint_velocities_);
    (void)node_->get_parameter("waypoint_accelerations", waypoint_accelerations_);
    (void)node_->get_parameter("default_waypoint_velocity", default_waypoint_velocity_);
    (void)node_->get_parameter("default_waypoint_acceleration", default_waypoint_acceleration_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(),
      "Exception while reading parameters: %s. Using defaults.", e.what());
  }

  RCLCPP_INFO(node_->get_logger(),
    "Loaded trajectory params: max_v=%.3f [m/s], max_a=%.3f [m/s^2], "
    "max_ang_v=%.3f [rad/s], max_ang_a=%.3f [rad/s^2]",
    max_v_, max_a_, max_ang_v_, max_ang_a_);
  // ~~~~ end solution
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Subscriber for Odometry
  sub_odom_ =
    node_->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&TrajectoryPlanner::uavOdomCallback, this, std::placeholders::_1));
}

// Callback to get current Pose of UAV
void TrajectoryPlanner::uavOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  // store current position in our planner
  tf2::fromMsg(odom->pose.pose, current_pose_);

  // store current velocity
  tf2::fromMsg(odom->twist.twist.linear, current_velocity_);
  odom_received_ = true;
}

bool TrajectoryPlanner::hasOdom() const
{
  return odom_received_;
}

// Method to set maximum speed.
void TrajectoryPlanner::setMaxSpeed(const double max_v)
{
  max_v_ = max_v;
}

bool TrajectoryPlanner::loadWaypointConstraints(
  const nav_msgs::msg::Path & path,
  std::vector<Eigen::Vector3d> * velocities,
  std::vector<Eigen::Vector3d> * accelerations) const
{
  const size_t waypoint_count = path.poses.size();
  if (waypoint_count == 0) {
    return false;
  }

  const size_t expected_size = waypoint_count * 3;
  velocities->clear();
  accelerations->clear();

  if (!waypoint_velocities_.empty()) {
    if (waypoint_velocities_.size() == expected_size) {
      velocities->reserve(waypoint_count);
      for (size_t i = 0; i < waypoint_count; ++i) {
        const size_t idx = i * 3;
        velocities->emplace_back(
          waypoint_velocities_[idx],
          waypoint_velocities_[idx + 1],
          waypoint_velocities_[idx + 2]);
      }
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "waypoint_velocities has %zu entries (expected %zu). Ignoring per-waypoint velocities.",
        waypoint_velocities_.size(), expected_size);
    }
  } else if (default_waypoint_velocity_.size() == 3) {
    velocities->assign(
      waypoint_count,
      Eigen::Vector3d(
        default_waypoint_velocity_[0],
        default_waypoint_velocity_[1],
        default_waypoint_velocity_[2]));
  }

  if (!waypoint_accelerations_.empty()) {
    if (waypoint_accelerations_.size() == expected_size) {
      accelerations->reserve(waypoint_count);
      for (size_t i = 0; i < waypoint_count; ++i) {
        const size_t idx = i * 3;
        accelerations->emplace_back(
          waypoint_accelerations_[idx],
          waypoint_accelerations_[idx + 1],
          waypoint_accelerations_[idx + 2]);
      }
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "waypoint_accelerations has %zu entries (expected %zu). Ignoring per-waypoint accelerations.",
        waypoint_accelerations_.size(), expected_size);
    }
  } else if (default_waypoint_acceleration_.size() == 3) {
    accelerations->assign(
      waypoint_count,
      Eigen::Vector3d(
        default_waypoint_acceleration_[0],
        default_waypoint_acceleration_[1],
        default_waypoint_acceleration_[2]));
  }

  return true;
}

// Plans a trajectory from the current position through a path of waypoints.
bool TrajectoryPlanner::planTrajectoryFromPath(
  const nav_msgs::msg::Path & path,
  mav_trajectory_generation::Trajectory * trajectory)
{
  if (path.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Received empty path, skipping trajectory planning.");
    return false;
  }

  // 3 Dimensional trajectory => through Cartesian space, no orientation
  const int dimension = 3;

  // Array for all waypoints and their constraints
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimize up to 4th order derivative (SNAP)
  const int derivative_to_optimize =
    mav_trajectory_generation::derivative_order::SNAP;

  // we have 2 vertices:
  // Start = current position
  // End   = desired position and optional velocity
  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  /******* Configure start point *******/
  // set start point constraints to current position and set all derivatives to zero
  start.makeStartOrEnd(
    current_pose_.translation(),
    derivative_to_optimize);

  // set start point's velocity to be constrained to current velocity
  start.addConstraint(
    mav_trajectory_generation::derivative_order::VELOCITY,
    current_velocity_);

  // add waypoint to list
  vertices.push_back(start);

  std::vector<Eigen::Vector3d> waypoint_velocities;
  std::vector<Eigen::Vector3d> waypoint_accelerations;
  (void)loadWaypointConstraints(path, &waypoint_velocities, &waypoint_accelerations);

  /******* Configure trajectory (intermediate waypoints) *******/
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const geometry_msgs::msg::PoseStamped & waypoint = path.poses[i];
    const Eigen::Vector3d waypoint_position(
      waypoint.pose.position.x,
      waypoint.pose.position.y,
      waypoint.pose.position.z);
    const bool is_last = (i == path.poses.size() - 1);

    if (is_last) {
      end.makeStartOrEnd(waypoint_position, derivative_to_optimize);

      if (i < waypoint_velocities.size()) {
        end.addConstraint(
          mav_trajectory_generation::derivative_order::VELOCITY,
          waypoint_velocities[i]);
      }

      if (i < waypoint_accelerations.size()) {
        end.addConstraint(
          mav_trajectory_generation::derivative_order::ACCELERATION,
          waypoint_accelerations[i]);
      }
    } else {
      mav_trajectory_generation::Vertex middle(dimension);
      middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoint_position);

      if (i < waypoint_velocities.size()) {
        middle.addConstraint(
          mav_trajectory_generation::derivative_order::VELOCITY,
          waypoint_velocities[i]);
      }
      if (i < waypoint_accelerations.size()) {
        middle.addConstraint(
          mav_trajectory_generation::derivative_order::ACCELERATION,
          waypoint_accelerations[i]);
      }

      vertices.push_back(middle);
    }
  }

  /******* Configure end point *******/
  // add waypoint to list
  vertices.push_back(end);

  // estimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(
    dimension, parameters);
  opt.setupFromVertices(
    vertices, segment_times,
    derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(
    mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
  opt.addMaximumMagnitudeConstraint(
    mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

  // solve trajectory
  opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(trajectory);

  return true;
}

void TrajectoryPlanner::drawMavTrajectory(
    const mav_trajectory_generation::Trajectory& trajectory,
    double distance, const std::string& frame_id,
    visualization_msgs::msg::MarkerArray* marker_array) {
    // sample trajectory
    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
    bool success = mav_trajectory_generation::sampleWholeTrajectory(
        trajectory, 0.1, &trajectory_points);
    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Could not sample trajectory.");
        return;
    }

    // draw trajectory
    marker_array->markers.clear();

    visualization_msgs::msg::Marker line_strip;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    // orange
    std_msgs::msg::ColorRGBA line_strip_color;
    line_strip_color.r = 1.0;
    line_strip_color.g = 0.5;
    line_strip_color.b = 0.0;
    line_strip_color.a = 1.0;
    line_strip.color = line_strip_color;
    line_strip.scale.x = 0.01;
    line_strip.ns = "path";

    double accumulated_distance = 0.0;
    Eigen::Vector3d last_position = Eigen::Vector3d::Zero();
    double scale = 0.3;
    double diameter = 0.3;
    for (size_t i = 0; i < trajectory_points.size(); ++i) {
        const mav_msgs::EigenTrajectoryPoint& point = trajectory_points[i];

        accumulated_distance += (last_position - point.position_W).norm();
        if (accumulated_distance > distance) {
            accumulated_distance = 0.0;
            mav_msgs::EigenMavState mav_state;
            mav_msgs::EigenMavStateFromEigenTrajectoryPoint(point, &mav_state);
            mav_state.orientation_W_B = point.orientation_W_B;

            visualization_msgs::msg::MarkerArray axes_arrows;
            axes_arrows.markers.resize(3);

            // x axis
            visualization_msgs::msg::Marker arrow_marker = axes_arrows.markers[0];
            arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker.action = visualization_msgs::msg::Marker::ADD;
            std_msgs::msg::ColorRGBA color;
            color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 1.0;
            arrow_marker.color = color;  // x - red
            arrow_marker.points.resize(2);
            arrow_marker.points[0] = tf2::toMsg(
                mav_state.position_W);
            arrow_marker.points[1] = tf2::toMsg(
                Eigen::Vector3d(mav_state.position_W +
                mav_state.orientation_W_B * Eigen::Vector3d::UnitX() * scale)
            );
            arrow_marker.scale.x = diameter * 0.1;
            arrow_marker.scale.y = diameter * 0.2;
            arrow_marker.scale.z = 0;

            // y axis
            visualization_msgs::msg::Marker arrow_marker_y = axes_arrows.markers[1];
            arrow_marker_y.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker_y.action = visualization_msgs::msg::Marker::ADD;
            std_msgs::msg::ColorRGBA color_y;
            color_y.r = 0.0; color_y.g = 1.0; color_y.b = 0.0; color_y.a = 1.0;
            arrow_marker_y.color = color_y;  // y - green
            arrow_marker_y.points.resize(2);
            arrow_marker_y.points[0] = tf2::toMsg(
                mav_state.position_W);
            arrow_marker_y.points[1] = tf2::toMsg(
                Eigen::Vector3d(mav_state.position_W +
                mav_state.orientation_W_B * Eigen::Vector3d::UnitY() * scale)
            );
            arrow_marker_y.scale.x = diameter * 0.1;
            arrow_marker_y.scale.y = diameter * 0.2;
            arrow_marker_y.scale.z = 0;

            // z axis
            visualization_msgs::msg::Marker arrow_marker_z = axes_arrows.markers[2];
            arrow_marker_z.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker_z.action = visualization_msgs::msg::Marker::ADD;
            std_msgs::msg::ColorRGBA color_z;
            color_z.r = 0.0; color_z.g = 0.0; color_z.b = 1.0; color_z.a = 1.0;
            arrow_marker_z.color = color_z;  // z - blue
            arrow_marker_z.points.resize(2);
            arrow_marker_z.points[0] = tf2::toMsg(
                mav_state.position_W);
            arrow_marker_z.points[1] = tf2::toMsg(
                Eigen::Vector3d(mav_state.position_W +
                mav_state.orientation_W_B * Eigen::Vector3d::UnitZ() * scale)
            );
            arrow_marker_z.scale.x = diameter * 0.1;
            arrow_marker_z.scale.y = diameter * 0.2;
            arrow_marker_z.scale.z = 0;

            // append to marker array
            for (size_t j = 0; j < axes_arrows.markers.size(); ++j) {
                axes_arrows.markers[j].header.frame_id = frame_id;
                axes_arrows.markers[j].ns = "pose";
                marker_array->markers.push_back(axes_arrows.markers[j]);
            }
        }
        last_position = point.position_W;
        geometry_msgs::msg::Point last_position_msg;
        last_position_msg = tf2::toMsg(last_position);
        line_strip.points.push_back(last_position_msg);
    }
    marker_array->markers.push_back(line_strip);

    std_msgs::msg::Header header;
    header.frame_id = frame_id;
    header.stamp = node_->now();
    for (size_t i = 0; i < marker_array->markers.size(); ++i) {
        marker_array->markers[i].header = header;
        marker_array->markers[i].id = i;
        marker_array->markers[i].lifetime = rclcpp::Duration::from_seconds(0.0);
        marker_array->markers[i].action = visualization_msgs::msg::Marker::ADD;
    }
}
