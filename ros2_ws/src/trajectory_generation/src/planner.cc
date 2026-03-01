#include "../include/trajectory_generation/planner.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace {

std::vector<double> estimateSegmentTimesContinuous(
    const mav_trajectory_generation::Vertex::Vector &vertices,
    const double max_v, const double max_a) {
  CHECK_GE(vertices.size(), 2);

  const size_t vertex_count = vertices.size();
  std::vector<Eigen::Vector3d> positions(vertex_count, Eigen::Vector3d::Zero());
  std::vector<double> distances(vertex_count - 1, 0.0);

  for (size_t i = 0; i < vertex_count; ++i) {
    Eigen::VectorXd pos_xd;
    vertices[i].getConstraint(mav_trajectory_generation::derivative_order::POSITION,
                              &pos_xd);
    positions[i] = Eigen::Vector3d(pos_xd);
    if (i > 0) {
      distances[i - 1] = (positions[i] - positions[i - 1]).norm();
    }
  }

  std::vector<double> segment_times;
  segment_times.reserve(vertex_count - 1);

  constexpr double kMinSegmentTime = 0.08;
  constexpr double kMinVertexSpeed = 0.05;
  constexpr double kTurnSlowdownFloor = 0.20;

  std::vector<double> waypoint_speeds(vertex_count, max_v);
  waypoint_speeds.front() = 0.0;
  waypoint_speeds.back() = 0.0;

  for (size_t i = 1; i + 1 < vertex_count; ++i) {
    const Eigen::Vector3d in = positions[i] - positions[i - 1];
    const Eigen::Vector3d out = positions[i + 1] - positions[i];
    const double in_norm = in.norm();
    const double out_norm = out.norm();
    if (in_norm < 1e-4 || out_norm < 1e-4) {
      waypoint_speeds[i] = 0.0;
      continue;
    }

    const double corner_cos =
        std::clamp((in / in_norm).dot(out / out_norm), -1.0, 1.0);
    const double turn_factor =
        std::max(kTurnSlowdownFloor, std::sqrt(std::max(0.0, (corner_cos + 1.0) * 0.5)));
    waypoint_speeds[i] = std::max(kMinVertexSpeed, max_v * turn_factor);
  }

  for (size_t i = 0; i < vertex_count; ++i) {
    Eigen::VectorXd vel_xd;
    if (vertices[i].getConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                                  &vel_xd)) {
      waypoint_speeds[i] = std::min(max_v, Eigen::Vector3d(vel_xd).norm());
    }
  }

  for (size_t i = 0; i + 1 < vertex_count; ++i) {
    const double d = distances[i];
    if (d < 1e-4) {
      waypoint_speeds[i + 1] = 0.0;
      continue;
    }
    const double vmax_next = std::sqrt(std::max(0.0, waypoint_speeds[i] * waypoint_speeds[i] +
                                                         2.0 * max_a * d));
    waypoint_speeds[i + 1] = std::min(waypoint_speeds[i + 1], vmax_next);
  }

  for (size_t i = vertex_count - 1; i > 0; --i) {
    const double d = distances[i - 1];
    if (d < 1e-4) {
      waypoint_speeds[i - 1] = 0.0;
      continue;
    }
    const double vmax_prev = std::sqrt(std::max(0.0, waypoint_speeds[i] * waypoint_speeds[i] +
                                                         2.0 * max_a * d));
    waypoint_speeds[i - 1] = std::min(waypoint_speeds[i - 1], vmax_prev);
  }

  for (size_t i = 0; i + 1 < vertex_count; ++i) {
    const double d = distances[i];
    if (d < 1e-4) {
      segment_times.push_back(kMinSegmentTime);
      continue;
    }

    const double v0 = waypoint_speeds[i];
    const double v1 = waypoint_speeds[i + 1];
    const double v_peak_sq = max_a * d + 0.5 * (v0 * v0 + v1 * v1);
    const double v_peak = std::sqrt(std::max(0.0, v_peak_sq));

    double segment_time = 0.0;
    if (v_peak <= max_v + 1e-6) {
      segment_time = (std::max(0.0, v_peak - v0) + std::max(0.0, v_peak - v1)) /
                     std::max(1e-6, max_a);
    } else {
      const double t_acc = std::max(0.0, (max_v - v0) / std::max(1e-6, max_a));
      const double t_dec = std::max(0.0, (max_v - v1) / std::max(1e-6, max_a));
      const double d_acc = std::max(0.0, (max_v * max_v - v0 * v0) / (2.0 * std::max(1e-6, max_a)));
      const double d_dec = std::max(0.0, (max_v * max_v - v1 * v1) / (2.0 * std::max(1e-6, max_a)));
      const double d_cruise = std::max(0.0, d - d_acc - d_dec);
      segment_time = t_acc + t_dec + d_cruise / std::max(1e-3, max_v);
    }

    segment_times.push_back(std::max(kMinSegmentTime, segment_time));
  }

  return segment_times;
}

}  // namespace

TrajectoryPlanner::TrajectoryPlanner(rclcpp::Node &node)
    : node_(&node), current_pose_(Eigen::Affine3d::Identity()),
      current_velocity_(Eigen::Vector3d::Zero()),
      current_angular_velocity_(Eigen::Vector3d::Zero()), odom_received_(false),
      max_v_(0.2), max_a_(0.2), max_ang_v_(0.0), max_ang_a_(0.0),
      waypoint_velocities_(), waypoint_accelerations_(),
      default_waypoint_velocity_(), default_waypoint_acceleration_(),
      enable_auto_waypoint_tangents_(true), auto_tangent_velocity_ratio_(0.55),
      auto_tangent_min_speed_(0.25), auto_tangent_turn_slowdown_(0.15) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  To Do: Load Trajectory Parameters from parameter file (ROS2)
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //
  //
  // ~~~~ begin solution
  try {
    node_->declare_parameter<double>("max_v", max_v_);
    node_->declare_parameter<double>("max_a", max_a_);

    // Optional angular limits (not used yet in this template, but declared for
    // completeness).
    node_->declare_parameter<double>("max_ang_v", max_ang_v_);
    node_->declare_parameter<double>("max_ang_a", max_ang_a_);

    node_->declare_parameter<std::vector<double>>("waypoint_velocities",
                                                  waypoint_velocities_);
    node_->declare_parameter<std::vector<double>>("waypoint_accelerations",
                                                  waypoint_accelerations_);
    node_->declare_parameter<std::vector<double>>("default_waypoint_velocity",
                                                  default_waypoint_velocity_);
    node_->declare_parameter<std::vector<double>>(
        "default_waypoint_acceleration", default_waypoint_acceleration_);
    node_->declare_parameter<bool>("enable_auto_waypoint_tangents",
                                   enable_auto_waypoint_tangents_);
    node_->declare_parameter<double>("auto_tangent_velocity_ratio",
                                     auto_tangent_velocity_ratio_);
    node_->declare_parameter<double>("auto_tangent_min_speed",
                                     auto_tangent_min_speed_);
    node_->declare_parameter<double>("auto_tangent_turn_slowdown",
                                     auto_tangent_turn_slowdown_);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Exception while declaring parameters: %s. Using defaults.",
                 e.what());
  }

  try {
    (void)node_->get_parameter("max_v", max_v_);
    (void)node_->get_parameter("max_a", max_a_);
    (void)node_->get_parameter("max_ang_v", max_ang_v_);
    (void)node_->get_parameter("max_ang_a", max_ang_a_);
    (void)node_->get_parameter("waypoint_velocities", waypoint_velocities_);
    (void)node_->get_parameter("waypoint_accelerations",
                               waypoint_accelerations_);
    (void)node_->get_parameter("default_waypoint_velocity",
                               default_waypoint_velocity_);
    (void)node_->get_parameter("default_waypoint_acceleration",
                               default_waypoint_acceleration_);
    (void)node_->get_parameter("enable_auto_waypoint_tangents",
                               enable_auto_waypoint_tangents_);
    (void)node_->get_parameter("auto_tangent_velocity_ratio",
                               auto_tangent_velocity_ratio_);
    (void)node_->get_parameter("auto_tangent_min_speed",
                               auto_tangent_min_speed_);
    (void)node_->get_parameter("auto_tangent_turn_slowdown",
                               auto_tangent_turn_slowdown_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Exception while reading parameters: %s. Using defaults.",
                 e.what());
  }

  RCLCPP_INFO(node_->get_logger(),
              "Loaded trajectory params: max_v=%.3f [m/s], max_a=%.3f [m/s^2], "
              "max_ang_v=%.3f [rad/s], max_ang_a=%.3f [rad/s^2]",
              max_v_, max_a_, max_ang_v_, max_ang_a_);
  RCLCPP_INFO(node_->get_logger(),
              "Auto tangent constraints: enabled=%s, ratio=%.2f, min_speed=%.2f, "
              "turn_slowdown=%.2f",
              enable_auto_waypoint_tangents_ ? "true" : "false",
              auto_tangent_velocity_ratio_, auto_tangent_min_speed_,
              auto_tangent_turn_slowdown_);

  if (default_waypoint_velocity_.size() == 3) {
    RCLCPP_INFO(node_->get_logger(),
                "Using default_waypoint_velocity: [%.3f, %.3f, %.3f] m/s for "
                "fly-through behavior",
                default_waypoint_velocity_[0], default_waypoint_velocity_[1],
                default_waypoint_velocity_[2]);
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "No default_waypoint_velocity set; optimizer will choose "
                "waypoint velocities freely");
  }
  // ~~~~ end solution
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Subscriber for Odometry
  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&TrajectoryPlanner::uavOdomCallback, this,
                std::placeholders::_1));

  // Timer for position logging (1Hz)
  position_log_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TrajectoryPlanner::positionLogCallback, this));
}

void TrajectoryPlanner::positionLogCallback() {
  if (odom_received_) {
    const auto &trans = current_pose_.translation();
    RCLCPP_INFO(node_->get_logger(),
                "[POS_LOG] Current Position: [%.2f, %.2f, %.2f]", trans.x(),
                trans.y(), trans.z());
  }
}

// Callback to get current Pose of UAV
void TrajectoryPlanner::uavOdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr odom) {
  // store current position in our planner
  tf2::fromMsg(odom->pose.pose, current_pose_);

  // store current velocity
  tf2::fromMsg(odom->twist.twist.linear, current_velocity_);
  odom_received_ = true;
}

bool TrajectoryPlanner::hasOdom() const { return odom_received_; }

// Method to set maximum speed.
void TrajectoryPlanner::setMaxSpeed(const double max_v) { max_v_ = max_v; }

bool TrajectoryPlanner::loadWaypointConstraints(
    const nav_msgs::msg::Path &path, std::vector<Eigen::Vector3d> *velocities,
    std::vector<Eigen::Vector3d> *accelerations) const {
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
        velocities->emplace_back(waypoint_velocities_[idx],
                                 waypoint_velocities_[idx + 1],
                                 waypoint_velocities_[idx + 2]);
      }
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "waypoint_velocities has %zu entries (expected %zu). "
                  "Ignoring per-waypoint velocities.",
                  waypoint_velocities_.size(), expected_size);
    }
  } else if (default_waypoint_velocity_.size() == 3) {
    velocities->assign(waypoint_count,
                       Eigen::Vector3d(default_waypoint_velocity_[0],
                                       default_waypoint_velocity_[1],
                                       default_waypoint_velocity_[2]));
  }

  if (!waypoint_accelerations_.empty()) {
    if (waypoint_accelerations_.size() == expected_size) {
      accelerations->reserve(waypoint_count);
      for (size_t i = 0; i < waypoint_count; ++i) {
        const size_t idx = i * 3;
        accelerations->emplace_back(waypoint_accelerations_[idx],
                                    waypoint_accelerations_[idx + 1],
                                    waypoint_accelerations_[idx + 2]);
      }
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "waypoint_accelerations has %zu entries (expected %zu). "
                  "Ignoring per-waypoint accelerations.",
                  waypoint_accelerations_.size(), expected_size);
    }
  } else if (default_waypoint_acceleration_.size() == 3) {
    accelerations->assign(waypoint_count,
                          Eigen::Vector3d(default_waypoint_acceleration_[0],
                                          default_waypoint_acceleration_[1],
                                          default_waypoint_acceleration_[2]));
  }

  return true;
}

Eigen::Vector3d TrajectoryPlanner::computeAutoWaypointVelocity(
    const std::vector<Eigen::Vector3d> &path_points,
    const size_t waypoint_index) const {
  if (path_points.empty() || waypoint_index >= path_points.size()) {
    return Eigen::Vector3d::Zero();
  }

  if (waypoint_index == 0 || waypoint_index + 1 >= path_points.size()) {
    return Eigen::Vector3d::Zero();
  }

  const Eigen::Vector3d to_prev = path_points[waypoint_index] - path_points[waypoint_index - 1];
  const Eigen::Vector3d to_next = path_points[waypoint_index + 1] - path_points[waypoint_index];
  const double len_prev = to_prev.norm();
  const double len_next = to_next.norm();
  if (len_prev < 1e-3 || len_next < 1e-3) {
    return Eigen::Vector3d::Zero();
  }

  const Eigen::Vector3d dir_prev = to_prev / len_prev;
  const Eigen::Vector3d dir_next = to_next / len_next;
  const double corner_cos = std::clamp(dir_prev.dot(dir_next), -1.0, 1.0);

  // For near-colinear waypoint chains (typical A* output), do not inject
  // hard velocity equality constraints at every midpoint. Let the optimizer
  // keep speed continuity naturally; this reduces residual stop-and-go.
  constexpr double kColinearCosThreshold = 0.995;  // ~5.7 degrees.
  if (corner_cos > kColinearCosThreshold) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d tangent = dir_prev + dir_next;
  if (tangent.norm() < 1e-4) {
    tangent = dir_next;
  } else {
    tangent.normalize();
  }

  const double turn_factor = std::max(auto_tangent_turn_slowdown_, (corner_cos + 1.0) * 0.5);
  const double local_distance = std::min(len_prev, len_next);
  const double accel_limited_speed = std::sqrt(std::max(0.0, max_a_ * local_distance));
  const double desired_speed = std::min(max_v_ * auto_tangent_velocity_ratio_, accel_limited_speed);
  const double speed = std::clamp(desired_speed * turn_factor, auto_tangent_min_speed_, max_v_);
  return tangent * speed;
}

// Plans a trajectory from the current position through a path of waypoints.
bool TrajectoryPlanner::planTrajectoryFromPath(
    const nav_msgs::msg::Path &path,
    mav_trajectory_generation::Trajectory *trajectory) {
  if (path.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "Received empty path, skipping trajectory planning.");
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
  // Set start point constraints to current position and set higher derivatives
  // (accel, jerk, snap) to zero, but use actual current velocity for smooth
  // stitching
  start.makeStartOrEnd(current_pose_.translation(), derivative_to_optimize);

  // Use actual current velocity instead of forcing zero for smooth replanning
  // Apply a small threshold to avoid noise when nearly at rest
  const double kMinVelocityThreshold = 0.01; // m/s
  Eigen::Vector3d start_velocity = current_velocity_;
  if (start_velocity.norm() < kMinVelocityThreshold) {
    start_velocity = Eigen::Vector3d::Zero();
  } else if (start_velocity.norm() > max_v_) {
    start_velocity = start_velocity.normalized() * max_v_;
    RCLCPP_WARN(node_->get_logger(),
                "Start velocity %.2f m/s exceeds max_v %.2f; clamping for stable "
                "replanning.",
                current_velocity_.norm(), max_v_);
  } 
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      start_velocity);

  // add waypoint to list
  vertices.push_back(start);

  std::vector<Eigen::Vector3d> waypoint_velocities;
  std::vector<Eigen::Vector3d> waypoint_accelerations;
  (void)loadWaypointConstraints(path, &waypoint_velocities,
                                &waypoint_accelerations);

  std::vector<Eigen::Vector3d> path_points;
  path_points.reserve(path.poses.size() + 1);
  path_points.push_back(current_pose_.translation());
  for (const auto &waypoint : path.poses) {
    path_points.emplace_back(waypoint.pose.position.x, waypoint.pose.position.y,
                             waypoint.pose.position.z);
  }

  /******* Configure trajectory (intermediate waypoints) *******/
  RCLCPP_INFO(node_->get_logger(),
              "[TRAJ_DIAG] Processing %zu waypoints from path",
              path.poses.size());
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const geometry_msgs::msg::PoseStamped &waypoint = path.poses[i];
    const Eigen::Vector3d waypoint_position(waypoint.pose.position.x,
                                            waypoint.pose.position.y,
                                            waypoint.pose.position.z);
    const bool is_last = (i == path.poses.size() - 1);

    RCLCPP_INFO(node_->get_logger(),
                "[TRAJ_DIAG] Processing waypoint %zu/%zu: [%.2f, %.2f, %.2f], "
                "is_last=%d",
                i, path.poses.size(), waypoint_position.x(),
                waypoint_position.y(), waypoint_position.z(), is_last);

    // Get previous vertex position for distance checks
    Eigen::Vector3d prev_pos;
    if (vertices.empty()) {
      // Should not happen as we added start
      prev_pos = current_pose_.translation();
    } else {
      Eigen::VectorXd pos;
      if (!vertices.back().getConstraint(
              mav_trajectory_generation::derivative_order::POSITION, &pos)) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to get position constraint from previous vertex!");
        prev_pos = current_pose_.translation();
      } else if (pos.size() != 3) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Previous vertex position has wrong size: %ld (expected 3)",
            pos.size());
        prev_pos = current_pose_.translation();
      } else {
        prev_pos = pos;
      }
    }

    // Skip waypoints that are too close to the previous vertex
    // This prevents zero-length segments which cause segment_time == 0
    // assertions
    const double kMinVertexDist = 0.1; // [m]
    const double dist_to_prev = (waypoint_position - prev_pos).norm();
    if (dist_to_prev < kMinVertexDist) {
      RCLCPP_WARN(node_->get_logger(),
                  "Skipping waypoint %zu (too close to previous, dist=%.3f m). "
                  "Waypoint: [%.2f, %.2f, %.2f], Prev: [%.2f, %.2f, %.2f]",
                  i, dist_to_prev, waypoint_position.x(), waypoint_position.y(),
                  waypoint_position.z(), prev_pos.x(), prev_pos.y(),
                  prev_pos.z());
      continue;
    }

    // Additionally, skip waypoints that are "behind" the drone when moving
    // This happens when the planner includes the start position as first
    // waypoint, but the drone has already moved forward during
    // planning/replanning IMPORTANT: Only skip if we have more waypoints, to
    // avoid filtering out everything
    if (i == 0 && path.poses.size() > 1 && current_velocity_.norm() > 0.2) {
      Eigen::Vector3d to_waypoint =
          waypoint_position - current_pose_.translation();
      Eigen::Vector3d vel_normalized = current_velocity_.normalized();
      double forward_projection = to_waypoint.dot(vel_normalized);

      RCLCPP_INFO(node_->get_logger(),
                  "[TRAJ_DIAG] First waypoint forward projection check: %.3f m "
                  "(threshold 0.3 m)",
                  forward_projection);

      // If waypoint is behind us (negative projection) or very close ahead,
      // skip it
      if (forward_projection <
          0.3) { // Less than 30cm ahead in direction of motion
        RCLCPP_INFO(node_->get_logger(),
                    "Skipping first waypoint %zu (behind or too close in "
                    "direction of motion, projection=%.3f m)",
                    i, forward_projection);
        continue;
      }
    }

    if (is_last) {
      // End waypoint: use makeStartOrEnd to set position and zero higher
      // derivatives Then optionally override with velocity/acceleration
      // constraints for fly-through
      RCLCPP_INFO(node_->get_logger(),
                  "[TRAJ_DIAG] Adding end waypoint %zu at [%.2f, %.2f, %.2f]",
                  i, waypoint_position.x(), waypoint_position.y(),
                  waypoint_position.z());
      end.makeStartOrEnd(waypoint_position, derivative_to_optimize);

      if (i < waypoint_velocities.size()) {
        end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                          waypoint_velocities[i]);
      }

      if (i < waypoint_accelerations.size()) {
        end.addConstraint(
            mav_trajectory_generation::derivative_order::ACCELERATION,
            waypoint_accelerations[i]);
      }
    } else {
      // Intermediate waypoints: only constrain position (and optionally
      // vel/accel) This makes them fly-through points rather than rest stops
      RCLCPP_INFO(
          node_->get_logger(),
          "[TRAJ_DIAG] Adding intermediate waypoint %zu at [%.2f, %.2f, %.2f]",
          i, waypoint_position.x(), waypoint_position.y(),
          waypoint_position.z());
      mav_trajectory_generation::Vertex middle(dimension);
      middle.addConstraint(
          mav_trajectory_generation::derivative_order::POSITION,
          waypoint_position);

      if (i < waypoint_velocities.size()) {
        middle.addConstraint(
            mav_trajectory_generation::derivative_order::VELOCITY,
            waypoint_velocities[i]);
      } else if (enable_auto_waypoint_tangents_) {
        const Eigen::Vector3d auto_velocity =
            computeAutoWaypointVelocity(path_points, i + 1);
        if (auto_velocity.norm() > 1e-3) {
          middle.addConstraint(
              mav_trajectory_generation::derivative_order::VELOCITY,
              auto_velocity);
        }
      }
      if (i < waypoint_accelerations.size()) {
        middle.addConstraint(
            mav_trajectory_generation::derivative_order::ACCELERATION,
            waypoint_accelerations[i]);
      }

      vertices.push_back(middle);
    }
  }

  RCLCPP_INFO(node_->get_logger(),
              "[TRAJ_DIAG] Finished processing waypoints. Vertices so far: %zu "
              "(start + intermediate waypoints)",
              vertices.size());

  /******* Configure end point *******/
  // add end waypoint to list (if it was configured in the loop)
  vertices.push_back(end);

  RCLCPP_INFO(node_->get_logger(),
              "[TRAJ_DIAG] After adding end vertex: %zu total vertices",
              vertices.size());

  // Check if we have at least one waypoint after filtering
  if (vertices.size() < 2) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "All %zu waypoints were filtered out (too close or behind drone). "
        "Cannot plan trajectory with only start vertex. "
        "Current pos: [%.2f, %.2f, %.2f], vel: [%.2f, %.2f, %.2f] (norm=%.3f "
        "m/s)",
        path.poses.size(), current_pose_.translation().x(),
        current_pose_.translation().y(), current_pose_.translation().z(),
        current_velocity_.x(), current_velocity_.y(), current_velocity_.z(),
        current_velocity_.norm());
    if (path.poses.size() > 0) {
      const auto &first_wp = path.poses[0];
      RCLCPP_ERROR(node_->get_logger(),
                   "  First waypoint was at: [%.2f, %.2f, %.2f]",
                   first_wp.pose.position.x, first_wp.pose.position.y,
                   first_wp.pose.position.z);
    }
    return false;
  }

  // If only 2 vertices, add a midpoint to give optimizer freedom
  if (vertices.size() == 2) {
    Eigen::VectorXd start_pos_xd, end_pos_xd;
    vertices[0].getConstraint(
        mav_trajectory_generation::derivative_order::POSITION, &start_pos_xd);
    vertices[1].getConstraint(
        mav_trajectory_generation::derivative_order::POSITION, &end_pos_xd);
    Eigen::Vector3d start_pos = start_pos_xd;
    Eigen::Vector3d end_pos = end_pos_xd;
    Eigen::Vector3d midpoint = (start_pos + end_pos) / 2.0;

    mav_trajectory_generation::Vertex middle(dimension);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                         midpoint);
    vertices.insert(vertices.begin() + 1, middle);
    RCLCPP_INFO(node_->get_logger(),
                "Inserted midpoint for 2-vertex path at [%.2f, %.2f, %.2f]",
                midpoint.x(), midpoint.y(), midpoint.z());
  }

  // estimate initial segment times
  std::vector<double> segment_times;
  // === DIAGNOSTIC: Log waypoints ===
  RCLCPP_INFO(node_->get_logger(),
              "[TRAJ_DIAG] Planning trajectory from %zu path waypoints -> %zu "
              "vertices:",
              path.poses.size(), vertices.size());
  Eigen::VectorXd start_vel_constraint;
  vertices[0].getConstraint(
      mav_trajectory_generation::derivative_order::VELOCITY,
      &start_vel_constraint);
  RCLCPP_INFO(node_->get_logger(),
              "  Start: [%.2f, %.2f, %.2f], vel=[%.2f, %.2f, %.2f] (measured: "
              "[%.2f, %.2f, %.2f])",
              current_pose_.translation().x(), current_pose_.translation().y(),
              current_pose_.translation().z(), start_vel_constraint[0],
              start_vel_constraint[1], start_vel_constraint[2],
              current_velocity_.x(), current_velocity_.y(),
              current_velocity_.z());
  if (path.poses.size() != vertices.size() - 1) {
    RCLCPP_INFO(
        node_->get_logger(),
        "  Note: %zu waypoints were skipped (too close or behind drone)",
        path.poses.size() - (vertices.size() - 1));
  }

  // Use continuous timing that accounts for optional waypoint velocity
  // constraints. This avoids the rest-to-rest bias from per-segment velocity
  // ramps, which can induce unnecessary braking at dense intermediate
  // waypoints (e.g., A* paths on straight corridors).
  segment_times = estimateSegmentTimesContinuous(vertices, max_v_, max_a_);

  // === DIAGNOSTIC: Log segment times ===
  {
    std::string times_str = "Segment times: ";
    for (size_t i = 0; i < segment_times.size(); ++i) {
      times_str += std::to_string(segment_times[i]) + "s ";
    }
    RCLCPP_INFO(node_->get_logger(), "[TRAJ_DIAG] %s", times_str.c_str());
  }

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // Polynomial order
  const int N = 10;

  // First, try the full nonlinear optimization with velocity/acc constraints.
  // If it fails to converge (which can happen for some waypoint / constraint
  // combinations), fall back to a simple linear optimization that at least
  // provides a smooth, feasible path (without strict max_v/max_a guarantees).
  {
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(
        dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(
        mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(
        mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

    try {
      const int result = opt.optimize();
      if (result > 0) {
        RCLCPP_INFO(node_->get_logger(),
                    "Nonlinear optimization succeeded (code %d: %s).", result,
                    result == 1   ? "SUCCESS"
                    : result == 2 ? "STOPVAL_REACHED"
                    : result == 3 ? "FTOL_REACHED"
                    : result == 4 ? "XTOL_REACHED"
                    : result == 5 ? "MAXEVAL_REACHED"
                    : result == 6 ? "MAXTIME_REACHED"
                                  : "OTHER_SUCCESS");

        opt.getTrajectory(trajectory);

        // =========================================================
        // ADD THIS CRITICAL FIX HERE
        // =========================================================
        // Always scale the trajectory to be dynamically feasible.
        trajectory->scaleSegmentTimesToMeetConstraints(max_v_, max_a_);
        // =========================================================

        // === TRAJECTORY DIAGNOSTICS ===
        double traj_duration = trajectory->getMaxTime();
        double max_v_actual = 0.0;
        double max_a_actual = 0.0;
        double max_pos_deviation = 0.0;

        // Sample trajectory to check constraints and deviation
        const double dt = 0.05;
        for (double t = 0.0; t <= traj_duration; t += dt) {
          mav_msgs::EigenTrajectoryPoint pt;
          mav_trajectory_generation::sampleTrajectoryAtTime(*trajectory, t,
                                                            &pt);

          // Check max velocity/acceleration
          double v_norm = pt.velocity_W.norm();
          double a_norm = pt.acceleration_W.norm();
          if (v_norm > max_v_actual)
            max_v_actual = v_norm;
          if (a_norm > max_a_actual)
            max_a_actual = a_norm;

          // Check deviation from straight line (approximate)
          // Ideally we check distance to the polyline formed by waypoints,
          // but here we just log sampled positions for manual inspection if
          // needed.
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "[Nonlinear Diagnostics] Duration: %.2f s | Max V: %.2f m/s (Limit "
            "%.2f) | Max A: %.2f m/s^2 (Limit %.2f)",
            traj_duration, max_v_actual, max_v_, max_a_actual, max_a_);

        // Log a few sampled positions to see if it's wandering
        RCLCPP_INFO(node_->get_logger(),
                    "[Nonlinear Diagnostics] Sampled Positions:");
        for (double t = 0.0; t <= traj_duration; t += traj_duration / 5.0) {
          mav_msgs::EigenTrajectoryPoint pt;
          mav_trajectory_generation::sampleTrajectoryAtTime(*trajectory, t,
                                                            &pt);
          RCLCPP_INFO(node_->get_logger(), "  t=%.2f: [%.2f, %.2f, %.2f]", t,
                      pt.position_W.x(), pt.position_W.y(), pt.position_W.z());
        }

        return true;
      } else {
        RCLCPP_WARN(node_->get_logger(),
                    "Nonlinear trajectory optimization FAILED (code %d). "
                    "Falling back to linear optimization.",
                    result);
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(node_->get_logger(),
                  "Exception during nonlinear trajectory optimization: %s. "
                  "Falling back to linear optimization.",
                  e.what());
    }
  }

  // Fallback: linear polynomial optimization (no hard max_v/max_a constraints).
  mav_trajectory_generation::PolynomialOptimization<N> opt_lin(dimension);
  opt_lin.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt_lin.solveLinear();
  opt_lin.getTrajectory(trajectory);

  // After linear optimization, check if we exceed limits and scale time
  // The linear optimization does not enforce hard constraints, so we need to
  // check if the resulting trajectory violates velocity or acceleration limits.
  // If it does, we scale the segment times to ensure feasibility.
  trajectory->scaleSegmentTimesToMeetConstraints(max_v_, max_a_);

  // === DIAGNOSTIC: Log trajectory characteristics ===
  {
    double max_vel = 0.0, max_acc = 0.0;
    const double sample_dt = 0.1;
    for (double t = 0.0; t <= trajectory->getMaxTime(); t += sample_dt) {
      mav_msgs::EigenTrajectoryPoint pt;
      if (mav_trajectory_generation::sampleTrajectoryAtTime(*trajectory, t,
                                                            &pt)) {
        double vel_mag = pt.velocity_W.norm();
        double acc_mag = pt.acceleration_W.norm();
        if (vel_mag > max_vel)
          max_vel = vel_mag;
        if (acc_mag > max_acc)
          max_acc = acc_mag;
      }
    }
    RCLCPP_INFO(node_->get_logger(),
                "[TRAJ_DIAG] Duration=%.2fs, MaxVel=%.2f m/s (limit=%.2f), "
                "MaxAcc=%.2f m/s² (limit=%.2f), Segments=%zu",
                trajectory->getMaxTime(), max_vel, max_v_, max_acc, max_a_,
                trajectory->segments().size());
  }

  return true;
}

void TrajectoryPlanner::drawMavTrajectory(
    const mav_trajectory_generation::Trajectory &trajectory, double distance,
    const std::string &frame_id,
    visualization_msgs::msg::MarkerArray *marker_array) {
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
    const mav_msgs::EigenTrajectoryPoint &point = trajectory_points[i];

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
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      color.a = 1.0;
      arrow_marker.color = color; // x - red
      arrow_marker.points.resize(2);
      arrow_marker.points[0] = tf2::toMsg(mav_state.position_W);
      arrow_marker.points[1] = tf2::toMsg(Eigen::Vector3d(
          mav_state.position_W +
          mav_state.orientation_W_B * Eigen::Vector3d::UnitX() * scale));
      arrow_marker.scale.x = diameter * 0.1;
      arrow_marker.scale.y = diameter * 0.2;
      arrow_marker.scale.z = 0;

      // y axis
      visualization_msgs::msg::Marker arrow_marker_y = axes_arrows.markers[1];
      arrow_marker_y.type = visualization_msgs::msg::Marker::ARROW;
      arrow_marker_y.action = visualization_msgs::msg::Marker::ADD;
      std_msgs::msg::ColorRGBA color_y;
      color_y.r = 0.0;
      color_y.g = 1.0;
      color_y.b = 0.0;
      color_y.a = 1.0;
      arrow_marker_y.color = color_y; // y - green
      arrow_marker_y.points.resize(2);
      arrow_marker_y.points[0] = tf2::toMsg(mav_state.position_W);
      arrow_marker_y.points[1] = tf2::toMsg(Eigen::Vector3d(
          mav_state.position_W +
          mav_state.orientation_W_B * Eigen::Vector3d::UnitY() * scale));
      arrow_marker_y.scale.x = diameter * 0.1;
      arrow_marker_y.scale.y = diameter * 0.2;
      arrow_marker_y.scale.z = 0;

      // z axis
      visualization_msgs::msg::Marker arrow_marker_z = axes_arrows.markers[2];
      arrow_marker_z.type = visualization_msgs::msg::Marker::ARROW;
      arrow_marker_z.action = visualization_msgs::msg::Marker::ADD;
      std_msgs::msg::ColorRGBA color_z;
      color_z.r = 0.0;
      color_z.g = 0.0;
      color_z.b = 1.0;
      color_z.a = 1.0;
      arrow_marker_z.color = color_z; // z - blue
      arrow_marker_z.points.resize(2);
      arrow_marker_z.points[0] = tf2::toMsg(mav_state.position_W);
      arrow_marker_z.points[1] = tf2::toMsg(Eigen::Vector3d(
          mav_state.position_W +
          mav_state.orientation_W_B * Eigen::Vector3d::UnitZ() * scale));
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
