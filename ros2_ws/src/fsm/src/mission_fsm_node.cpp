#include "fsm/mission_fsm_node.hpp"

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <queue>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace control {

namespace {
double xy_distance(const geometry_msgs::msg::Point &a,
                   const geometry_msgs::msg::Point &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double point_to_segment_distance_xy(const geometry_msgs::msg::Point &p,
                                    const geometry_msgs::msg::Point &a,
                                    const geometry_msgs::msg::Point &b) {
  const double abx = b.x - a.x;
  const double aby = b.y - a.y;
  const double apx = p.x - a.x;
  const double apy = p.y - a.y;

  const double ab_len_sq = abx * abx + aby * aby;
  if (ab_len_sq < 1e-9) {
    return xy_distance(p, a);
  }

  const double t = std::clamp((apx * abx + apy * aby) / ab_len_sq, 0.0, 1.0);
  geometry_msgs::msg::Point projection;
  projection.x = a.x + t * abx;
  projection.y = a.y + t * aby;
  projection.z = p.z;
  return xy_distance(p, projection);
}
} // namespace

MissionFsmNode::MissionFsmNode()
    : Node("mission_fsm_node"), current_state_(MissionState::INIT),
      pose_received_(false), mission_start_signal_received_(false),
      goal_active_(false), goal_request_pending_(false),
      lanterns_found_count_(0), lantern_dedup_threshold_(2.5),
      takeoff_altitude_(2.0), z_retry_index_(0), goal_set_time_(this->now()),
      min_exploration_goal_distance_(2.0), consecutive_too_close_rejections_(0),
      last_successful_exploration_goal_time_(this->now()),
      last_replan_time_(this->now()) {
  // Initialize positions
  start_position_.x = 0.0;
  start_position_.y = 0.0;
  start_position_.z = 0.0;

  // Initialize goal tracking
  last_pose_at_goal_set_.x = 0.0;
  last_pose_at_goal_set_.y = 0.0;
  last_pose_at_goal_set_.z = 0.0;

  // Declare and read parameters
  this->declare_parameter("lantern_dedup_threshold", lantern_dedup_threshold_);
  lantern_dedup_threshold_ =
      this->get_parameter("lantern_dedup_threshold").as_double();
  this->declare_parameter("min_exploration_goal_distance",
                          min_exploration_goal_distance_);
  min_exploration_goal_distance_ =
      this->get_parameter("min_exploration_goal_distance").as_double();
  this->declare_parameter("explore_goal_selection_timeout",
                          explore_goal_selection_timeout_);
  explore_goal_selection_timeout_ =
      this->get_parameter("explore_goal_selection_timeout").as_double();
  this->declare_parameter("explore_goal_selection_max_failures",
                          explore_goal_selection_max_failures_);
  explore_goal_selection_max_failures_ =
      this->get_parameter("explore_goal_selection_max_failures").as_int();

  // Mid-flight replanning interval (seconds)
  this->declare_parameter("replan_interval_s", replan_interval_s_);
  replan_interval_s_ = this->get_parameter("replan_interval_s").as_double();
  this->declare_parameter("z_retry_max_attempts", z_retry_max_attempts_);
  z_retry_max_attempts_ = this->get_parameter("z_retry_max_attempts").as_int();
  this->declare_parameter("z_retry_step", z_retry_step_);
  z_retry_step_ = this->get_parameter("z_retry_step").as_double();

  // Declare and read planner type parameter (default: RRT)
  this->declare_parameter("planner_type", "A_star");
  planner_type_ = this->get_parameter("planner_type").as_string();

  // Takeoff altitude: how many metres above start position to ascend
  this->declare_parameter("takeoff_altitude", takeoff_altitude_);
  takeoff_altitude_ = this->get_parameter("takeoff_altitude").as_double();

  this->declare_parameter("macroplanning_enabled", macroplanning_enabled_);
  macroplanning_enabled_ =
      this->get_parameter("macroplanning_enabled").as_bool();
  this->declare_parameter("nodes_distance", nodes_distance_);
  nodes_distance_ = this->get_parameter("nodes_distance").as_double();
  this->declare_parameter("node_radius", node_radius_);
  node_radius_ = this->get_parameter("node_radius").as_double();
  this->declare_parameter("max_potential_node_range",
                          max_potential_node_range_);
  max_potential_node_range_ =
      this->get_parameter("max_potential_node_range").as_double();
  this->declare_parameter("potential_node_backoff_distance",
                          potential_node_backoff_distance_);
  potential_node_backoff_distance_ =
      this->get_parameter("potential_node_backoff_distance").as_double();
  this->declare_parameter("potential_angle_threshold_deg",
                          potential_angle_threshold_deg_);
  potential_angle_threshold_deg_ =
      this->get_parameter("potential_angle_threshold_deg").as_double();
  this->declare_parameter("seen_point_timeout_s", seen_point_timeout_s_);
  seen_point_timeout_s_ =
      this->get_parameter("seen_point_timeout_s").as_double();
  this->declare_parameter("goal_request_timeout_s", goal_request_timeout_s_);
  goal_request_timeout_s_ =
      this->get_parameter("goal_request_timeout_s").as_double();
  this->declare_parameter("force_explorer_timeout_s",
                          force_explorer_timeout_s_);
  force_explorer_timeout_s_ =
      this->get_parameter("force_explorer_timeout_s").as_double();
  this->declare_parameter("single_edge_priority_reached_radius",
                          single_edge_priority_reached_radius_);
  single_edge_priority_reached_radius_ =
      this->get_parameter("single_edge_priority_reached_radius").as_double();
  this->declare_parameter("waypoint_obstacle_clearance",
                          waypoint_obstacle_clearance_);
  waypoint_obstacle_clearance_ =
      this->get_parameter("waypoint_obstacle_clearance").as_double();
  this->declare_parameter("freeze_reset_distance_threshold",
                          freeze_reset_distance_threshold_);
  freeze_reset_distance_threshold_ =
      this->get_parameter("freeze_reset_distance_threshold").as_double();
  this->declare_parameter("freeze_timeout_s", freeze_timeout_s_);
  freeze_timeout_s_ = this->get_parameter("freeze_timeout_s").as_double();

  // Cave entrance - Main entrance
  cave_entrance_.x = -320.0;
  cave_entrance_.y = 10.0;
  cave_entrance_.z = 18.0;

  // Z-retry altitudes: start with default, then try lower, then higher
  // z_retry_altitudes_ is now built dynamically per goal in
  // request_exploration_goal()
  // --- Subscribers ---
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/current_state_est", 10,
      std::bind(&MissionFsmNode::odometry_callback, this,
                std::placeholders::_1));

  lantern_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/detected_lanterns", 10,
      std::bind(&MissionFsmNode::lantern_callback, this,
                std::placeholders::_1));

  planner_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/planner/status", 10,
      std::bind(&MissionFsmNode::planner_status_callback, this,
                std::placeholders::_1));

  start_mission_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/mission/start", 10,
      std::bind(&MissionFsmNode::start_mission_callback, this,
                std::placeholders::_1));

  exploration_map_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/exploration/map_ready", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        exploration_map_ready_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Exploration map_ready flag set to: %s",
                    exploration_map_ready_ ? "true" : "false");
      });

  depth_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/points_world", rclcpp::SensorDataQoS(),
      std::bind(&MissionFsmNode::depth_points_callback, this,
                std::placeholders::_1));

  planner_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "waypoints", 10,
      std::bind(&MissionFsmNode::planner_path_callback, this,
                std::placeholders::_1));

  // --- Service Clients ---
  exploration_goal_client_ =
      this->create_client<exploring::srv::GetExplorationGoal>(
          "/exploration/get_goal");

  // --- Publishers ---
  trajectory_pub_ =
      this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
          "/command/trajectory", 10);

  waypoint_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "waypoints", rclcpp::QoS(1).transient_local().reliable());

  cancel_pub_ = this->create_publisher<std_msgs::msg::Empty>("/fsm/cancel", 10);

  state_pub_ = this->create_publisher<std_msgs::msg::String>("/fsm/state", 10);

  planner_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planner/goal", 10);
  // Publisher for A* planner
  planner_goal_pub_a_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planner_a/goal", 10);

  drone_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/fsm/drone_marker", 10);
  checkpoint_markers_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/fsm/checkpoint_markers", 10);

  enable_mapping_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/enable_mapping", rclcpp::QoS(1).transient_local().reliable());

  blacklist_goal_pub_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(
          "/exploration/blacklist_goal", 10);
  priority_target_pub_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(
          "/exploration/priority_target", 10);
  punishment_target_pub_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(
          "/exploration/punishment_target", 10);

  // --- Timer: 10 Hz main loop ---
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&MissionFsmNode::timer_callback, this));

  // --- TF Listener ---
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "Mission FSM Node initialized. State: INIT");
  goal_request_sent_time_ = this->now();
  force_explorer_start_time_ = this->now();
  freeze_stationary_since_ = this->now();
  publish_state();
}

void MissionFsmNode::odometry_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pose_ = msg->pose.pose;

  if (!pose_received_) {
    pose_received_ = true;
    // Store initial position as start position
    start_position_.x = current_pose_.position.x;
    start_position_.y = current_pose_.position.y;
    start_position_.z = current_pose_.position.z;
    RCLCPP_INFO(this->get_logger(), "Initial pose received: [%.2f, %.2f, %.2f]",
                start_position_.x, start_position_.y, start_position_.z);
  }
}

void MissionFsmNode::lantern_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // Only process lanterns during EXPLORE state
  if (current_state_ != MissionState::EXPLORE) {
    return;
  }

  // De-duplication: Is this a new lantern?
  // MUST transform to world frame first, because the detection might be in a
  // local frame (e.g. camera) that moves with the drone.
  geometry_msgs::msg::PoseStamped lantern_world;
  try {
    lantern_world =
        tf_buffer_->transform(*msg, "world", tf2::durationFromSec(1.0));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform lantern to world: %s",
                ex.what());
    return;
  }

  bool is_new = true;
  for (const auto &existing_pose : detected_lantern_poses_) {
    double dist =
        calculate_distance(lantern_world.pose.position, existing_pose.position);

    // Debug logging for distance check
    RCLCPP_INFO(this->get_logger(),
                "Checking lantern candidate [%.2f, %.2f, %.2f] against "
                "existing [%.2f, %.2f, %.2f]: dist=%.2f",
                lantern_world.pose.position.x, lantern_world.pose.position.y,
                lantern_world.pose.position.z, existing_pose.position.x,
                existing_pose.position.y, existing_pose.position.z, dist);

    if (dist <= lantern_dedup_threshold_) {
      is_new = false;
      break;
    }
  }

  if (is_new) {
    RCLCPP_INFO(this->get_logger(),
                "New lantern detected at [%.2f, %.2f, %.2f]! Total found: %zu",
                lantern_world.pose.position.x, lantern_world.pose.position.y,
                lantern_world.pose.position.z, lanterns_found_count_ + 1);

    detected_lantern_poses_.push_back(lantern_world.pose);
    lanterns_found_count_++;

    // Interrupt current movement
    cancel_pub_->publish(std_msgs::msg::Empty());
    goal_active_ = false;

    // Transition to LANTERN_FOUND state
    transition_to(MissionState::LANTERN_FOUND);
  }
}

void MissionFsmNode::timer_callback() {
  update_state();
  monitor_frozen_progress();
  publish_state();
  publish_drone_marker();
  publish_checkpoint_markers();
}

bool MissionFsmNode::is_freeze_watch_state() const {
  return current_state_ == MissionState::EXPLORE ||
         current_state_ == MissionState::REFINE_GOAL;
}

void MissionFsmNode::monitor_frozen_progress() {
  if (!pose_received_) {
    return;
  }

  if (!is_freeze_watch_state()) {
    freeze_reference_position_ = current_pose_.position;
    freeze_stationary_since_ = this->now();
    freeze_reference_initialized_ = true;
    return;
  }

  if (!freeze_reference_initialized_) {
    freeze_reference_position_ = current_pose_.position;
    freeze_stationary_since_ = this->now();
    freeze_reference_initialized_ = true;
    return;
  }

  const double movement =
      calculate_distance(current_pose_.position, freeze_reference_position_);
  if (movement > freeze_reset_distance_threshold_) {
    freeze_reference_position_ = current_pose_.position;
    freeze_stationary_since_ = this->now();
    return;
  }

  const double stationary_time =
      (this->now() - freeze_stationary_since_).seconds();
  if (stationary_time >= freeze_timeout_s_) {
    execute_frozen_recovery();
  }
}

void MissionFsmNode::execute_frozen_recovery() {
  RCLCPP_ERROR(
      this->get_logger(),
      "Drone frozen watchdog triggered (stationary %.1f s within %.2f m). "
      "Resetting mission/exploration state while preserving macroplanning "
      "graph.",
      freeze_timeout_s_, freeze_reset_distance_threshold_);

  cancel_pub_->publish(std_msgs::msg::Empty());

  mission_start_signal_received_ = true;
  goal_active_ = false;
  goal_request_pending_ = false;
  current_goal_ = geometry_msgs::msg::Point();
  strategic_goal_ = geometry_msgs::msg::Point();
  z_retry_altitudes_.clear();
  z_retry_index_ = 0;
  active_goal_source_ = GoalSource::EXPLORER;
  active_goal_anchor_node_id_ = -1;
  potential_resolution_node_id_ = -1;
  potential_objective_timer_active_ = false;
  potential_objective_anchor_node_id_ = -1;
  active_rrt_waypoints_.clear();
  travel_mode_ = false;
  explorer_mode_suspended_for_travel_ = false;
  travel_path_.clear();
  suppress_rule_l_ = false;
  force_explorer_until_new_node_ = false;
  fallback_origin_node_id_ = -1;
  single_edge_travel_target_node_id_ = -1;
  clear_single_edge_priority_target();
  clear_single_edge_punishment_target();
  returning_to_entrance_via_travel_ = false;
  consecutive_too_close_rejections_ = 0;
  consecutive_goal_request_failures_ = 0;
  detected_lantern_poses_.clear();
  lanterns_found_count_ = 0;
  ++explorer_request_epoch_;

  last_pose_at_goal_set_ = current_pose_.position;
  goal_set_time_ = this->now();
  last_successful_exploration_goal_time_ = this->now();
  freeze_reference_position_ = current_pose_.position;
  freeze_stationary_since_ = this->now();
  freeze_reference_initialized_ = true;

  if (current_state_ == MissionState::EXPLORE) {
    RCLCPP_WARN(this->get_logger(),
                "Reinitializing EXPLORE state after frozen fallback.");
    on_state_exit(MissionState::EXPLORE);
    on_state_enter(MissionState::EXPLORE);
    publish_state();
  } else {
    transition_to(MissionState::EXPLORE);
  }
}

void MissionFsmNode::planner_status_callback(
    const std_msgs::msg::String::SharedPtr msg) {
  if (!goal_active_) {
    RCLCPP_DEBUG(
        this->get_logger(),
        "Ignoring planner status '%s' because no goal is currently active.",
        msg->data.c_str());
    return;
  }

  if (msg->data == "GOAL_REACHED") {
    RCLCPP_INFO(this->get_logger(), "Planner: Goal reached!");
    goal_active_ = false;
    // State machine will handle transition in update_state()
  } else if (msg->data == "PLAN_FAILED") {
    RCLCPP_WARN(this->get_logger(),
                "Planner: Planning failed for current goal.");

    if (macroplanning_enabled_) {
      std::vector<int> provisional_failed_nodes;
      for (const auto &entry : graph_nodes_) {
        if (!entry.second.is_provisional) {
          continue;
        }
        if (calculate_distance(entry.second.position, current_goal_) <=
            node_radius_) {
          provisional_failed_nodes.push_back(entry.first);
        }
      }
      for (const int node_id : provisional_failed_nodes) {
        RCLCPP_WARN(this->get_logger(),
                    "Removing provisional node %d after PLAN_FAILED near "
                    "[%.2f, %.2f, %.2f].",
                    node_id, current_goal_.x, current_goal_.y, current_goal_.z);
        remove_checkpoint_node(node_id);
      }
    }

    goal_active_ = false;

    if (current_state_ == MissionState::EXPLORE &&
        active_goal_source_ == GoalSource::POTENTIAL &&
        active_goal_anchor_node_id_ >= 0) {
      geometry_msgs::msg::Point potential_goal;
      if (pop_next_potential_for_node(active_goal_anchor_node_id_,
                                      potential_goal)) {
        RCLCPP_WARN(
            this->get_logger(),
            "Potential goal failed. Trying next potential from anchor %d.",
            active_goal_anchor_node_id_);
        try_activate_exploration_goal(potential_goal, true,
                                      GoalSource::POTENTIAL,
                                      active_goal_anchor_node_id_);
        return;
      }
      RCLCPP_WARN(
          this->get_logger(),
          "All potential goals failed for anchor %d. Resuming travel mode.",
          active_goal_anchor_node_id_);
      potential_resolution_node_id_ = -1;
      active_goal_anchor_node_id_ = -1;
      active_goal_source_ = GoalSource::TRAVEL;
      return;
    }

    if (current_state_ == MissionState::EXPLORE &&
        active_goal_source_ == GoalSource::TRAVEL) {
      int target_node_id = -1;
      if (!travel_path_.empty()) {
        target_node_id = travel_path_.front();
      }

      if (target_node_id >= 0 && graph_nodes_.count(target_node_id) > 0) {
        geometry_msgs::msg::Point alternate_position;
        if (find_alternate_position_for_node(target_node_id,
                                             alternate_position)) {
          graph_nodes_[target_node_id].position = alternate_position;
          RCLCPP_WARN(this->get_logger(),
                      "Travel checkpoint node %d unreachable. Retrying with "
                      "nearby node position [%.2f, %.2f, %.2f].",
                      target_node_id, alternate_position.x,
                      alternate_position.y, alternate_position.z);
          try_activate_exploration_goal(alternate_position, true,
                                        GoalSource::TRAVEL, -1);
          return;
        }

        RCLCPP_ERROR(this->get_logger(),
                     "Travel checkpoint node %d remained unreachable after "
                     "nearby retries. Falling back to explorer mode.",
                     target_node_id);
        clear_node_relocation_state(target_node_id);
      }

      travel_mode_ = false;
      potential_resolution_node_id_ = -1;
      travel_path_.clear();
      resume_explorer_mode_after_travel();
      return;
    }

    // If we're exploring, trigger the Z-retry recovery for the *current* goal.
    // Using start_refine_for_current_goal() keeps strategic_goal_ synchronized
    // with the failing planner target (important for travel-mode goals).
    if (current_state_ == MissionState::EXPLORE ||
        current_state_ == MissionState::REFINE_GOAL) {
      start_refine_for_current_goal("planner reported PLAN_FAILED");
    }
  }
}

void MissionFsmNode::exploration_goal_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // Only accept new exploration goals when in EXPLORE state and not busy
  if (current_state_ != MissionState::EXPLORE) {
    RCLCPP_DEBUG(this->get_logger(),
                 "Ignoring exploration goal - not in EXPLORE state.");
    return;
  }

  if (goal_active_) {
    RCLCPP_DEBUG(this->get_logger(),
                 "Ignoring exploration goal - already have an active goal.");
    return;
  }

  // Store the strategic (x,y) goal
  strategic_goal_.x = msg->pose.position.x;
  strategic_goal_.y = msg->pose.position.y;
  strategic_goal_.z =
      msg->pose.position.z; // Initial Z from exploration manager

  RCLCPP_INFO(this->get_logger(),
              "New exploration goal received: [%.2f, %.2f, %.2f]",
              strategic_goal_.x, strategic_goal_.y, strategic_goal_.z);

  // Reset Z-retry index and start with the first altitude
  z_retry_index_ = 0;

  // Use first Z from retry list (override 2D map's Z=0) and send goal to the
  // planner node, which will generate a collision-free path for the trajectory
  // generator.
  double altitude = z_retry_altitudes_[z_retry_index_];
  z_retry_index_++;

  geometry_msgs::msg::PoseStamped planner_goal;
  planner_goal.header.stamp = this->now();
  planner_goal.header.frame_id = "world";
  planner_goal.pose.position.x = strategic_goal_.x;
  planner_goal.pose.position.y = strategic_goal_.y;
  planner_goal.pose.position.z = altitude;
  planner_goal.pose.orientation.w = 1.0;

  // Check minimum distance - reject goals that are too close
  double goal_dist =
      std::sqrt((planner_goal.pose.position.x - current_pose_.position.x) *
                    (planner_goal.pose.position.x - current_pose_.position.x) +
                (planner_goal.pose.position.y - current_pose_.position.y) *
                    (planner_goal.pose.position.y - current_pose_.position.y));

  if (goal_dist < min_exploration_goal_distance_) {
    ++consecutive_too_close_rejections_;

    // If we've rejected 3 consecutive goals for being too close, accept this
    // one anyway to prevent infinite loop
    if (consecutive_too_close_rejections_ >=
        MAX_CONSECUTIVE_TOO_CLOSE_REJECTIONS) {
      RCLCPP_WARN(
          this->get_logger(),
          "Exploration goal too close (%.2f m < %.2f m), but accepting anyway "
          "after %d consecutive rejections to avoid getting stuck.",
          goal_dist, min_exploration_goal_distance_,
          consecutive_too_close_rejections_);
      consecutive_too_close_rejections_ = 0; // Reset counter
      // Continue to accept the goal below
    } else {
      RCLCPP_WARN(
          this->get_logger(),
          "Exploration goal too close (%.2f m < %.2f m). "
          "Rejecting (consecutive rejections: %d/%d). Requesting new goal.",
          goal_dist, min_exploration_goal_distance_,
          consecutive_too_close_rejections_,
          MAX_CONSECUTIVE_TOO_CLOSE_REJECTIONS);
      // Request a new goal immediately
      request_exploration_goal();
      return;
    }
  } else {
    // Goal is far enough - reset rejection counter
    consecutive_too_close_rejections_ = 0;
  }

  // Track the active exploration goal so distance checks in EXPLORE use
  // the correct target instead of a stale one (e.g. cave entrance).
  current_goal_.x = planner_goal.pose.position.x;
  current_goal_.y = planner_goal.pose.position.y;
  current_goal_.z = planner_goal.pose.position.z;

  // Track when goal was set and drone position for timeout/movement detection
  goal_set_time_ = this->now();
  last_pose_at_goal_set_ = current_pose_.position;

  if (planner_type_ == "RRT") {
    planner_goal_pub_->publish(planner_goal);

  } else {
    planner_goal_pub_a_->publish(planner_goal);
  }
  goal_active_ = true;
  active_goal_source_ = GoalSource::EXPLORER;
  active_goal_anchor_node_id_ = -1;

  RCLCPP_INFO(this->get_logger(),
              "Sent exploration goal to planner: [%.2f, %.2f, %.2f] "
              "(distance=%.2f m)",
              planner_goal.pose.position.x, planner_goal.pose.position.y,
              planner_goal.pose.position.z, goal_dist);
}

void MissionFsmNode::depth_points_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!macroplanning_enabled_ || !pose_received_ || msg->width == 0 ||
      msg->data.empty()) {
    return;
  }

  geometry_msgs::msg::Point best_point;
  bool found = false;
  double best_dist = 0.0;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
  recent_obstacle_points_.clear();
  recent_obstacle_points_.reserve(256);
  size_t sample_counter = 0;

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const float x = *iter_x;
    const float y = *iter_y;
    const float z = *iter_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    geometry_msgs::msg::Point candidate;
    candidate.x = static_cast<double>(x);
    candidate.y = static_cast<double>(y);
    candidate.z = static_cast<double>(z);

    const double d = calculate_distance(candidate, current_pose_.position);
    if (d > max_potential_node_range_) {
      continue;
    }

    // Keep a lightweight, periodically sampled obstacle set for safer
    // checkpoint placement decisions.
    if ((sample_counter++ % 20u) == 0u) {
      recent_obstacle_points_.push_back(candidate);
    }

    if (!found || d > best_dist) {
      best_dist = d;
      best_point = candidate;
      found = true;
    }
  }

  if (found) {
    geometry_msgs::msg::Point backed_off_point = best_point;
    const double line_dist =
        calculate_distance(best_point, current_pose_.position);
    if (line_dist > 1e-6 && potential_node_backoff_distance_ > 0.0) {
      const double applied_backoff = std::min(potential_node_backoff_distance_,
                                              std::max(0.0, line_dist - 0.1));
      const double ux = (current_pose_.position.x - best_point.x) / line_dist;
      const double uy = (current_pose_.position.y - best_point.y) / line_dist;
      const double uz = (current_pose_.position.z - best_point.z) / line_dist;
      backed_off_point.x += ux * applied_backoff;
      backed_off_point.y += uy * applied_backoff;
      backed_off_point.z += uz * applied_backoff;
    }

    latest_seen_point_ = backed_off_point;
    latest_seen_point_valid_ = true;
    latest_seen_point_stamp_ = this->now();
  }
}

void MissionFsmNode::planner_path_callback(
    const nav_msgs::msg::Path::SharedPtr msg) {
  if (msg->poses.empty()) {
    return;
  }

  if (current_state_ != MissionState::EXPLORE || !goal_active_) {
    return;
  }

  if (active_goal_source_ != GoalSource::TRAVEL &&
      active_goal_source_ != GoalSource::POTENTIAL) {
    return;
  }

  // Travel/potential goals always use the RRT topic; cache the latest planned
  // waypoints so we can validate them while flying.
  active_rrt_waypoints_.clear();
  for (const auto &pose : msg->poses) {
    active_rrt_waypoints_.push_back(pose.pose.position);
  }
}

void MissionFsmNode::start_mission_callback(
    const std_msgs::msg::Empty::SharedPtr msg) {
  (void)msg;
  if (!mission_start_signal_received_) {
    mission_start_signal_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Mission Start Signal Received!");
  }
}

void MissionFsmNode::transition_to(MissionState new_state) {
  if (new_state == current_state_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s",
              state_to_string(current_state_).c_str(),
              state_to_string(new_state).c_str());

  on_state_exit(current_state_);
  current_state_ = new_state;
  on_state_enter(new_state);
  publish_state();
}

void MissionFsmNode::on_state_enter(MissionState state) {
  switch (state) {
  case MissionState::TAKEOFF:
    RCLCPP_INFO(this->get_logger(), "Taking off to altitude: %.2f m",
                takeoff_altitude_);

    // Use current_state_est for takeoff position
    {
      double target_x = current_pose_.position.x;
      double target_y = current_pose_.position.y;
      double target_z =
          current_pose_.position.z +
          takeoff_altitude_; // Ascend takeoff_altitude_ m relative to start

      RCLCPP_INFO(this->get_logger(),
                  "Takeoff Target (current_state_est): [%.2f, %.2f, %.2f]",
                  target_x, target_y, target_z);

      geometry_msgs::msg::Point target;
      target.x = target_x;
      target.y = target_y;
      target.z = target_z;
      publish_waypoint_path({target});
    }
    goal_active_ = true;
    break;

  case MissionState::GOTO_ENTRANCE:
    RCLCPP_INFO(this->get_logger(),
                "Navigating to cave entrance: [%.2f, %.2f, %.2f]",
                cave_entrance_.x, cave_entrance_.y, cave_entrance_.z);
    publish_waypoint_path({cave_entrance_});
    goal_active_ = true;
    break;

  case MissionState::EXPLORE: {
    RCLCPP_INFO(this->get_logger(),
                "Entering exploration mode. Lanterns found: %zu/%zu",
                lanterns_found_count_, TARGET_LANTERN_COUNT);
    // Enable mapping (cloud gate will start forwarding point clouds)
    auto enable_msg = std_msgs::msg::Bool();
    enable_msg.data = true;
    enable_mapping_pub_->publish(enable_msg);
    // Reset Z-retry for new exploration goal
    z_retry_index_ = 0;
    goal_active_ = false;
    active_goal_source_ = GoalSource::EXPLORER;
    active_goal_anchor_node_id_ = -1;
    potential_objective_timer_active_ = false;
    goal_request_pending_ = false;
    // Reset goal-selection tracking when (re)entering exploration
    reset_explorer_goal_filters();
    last_successful_exploration_goal_time_ = this->now();
    reset_explorer_filters_on_next_goal_ = false;
    returning_to_entrance_via_travel_ = false;
    if (macroplanning_enabled_ && entrance_node_id_ < 0) {
      entrance_node_id_ = create_checkpoint_node(cave_entrance_, true);
      last_visited_node_id_ = entrance_node_id_;
    }
    // Main loop (update_state) will request the goal automatically
  } break;

  case MissionState::REFINE_GOAL:
    RCLCPP_INFO(this->get_logger(), "Entering goal refinement (Z-retry) mode.");
    // Z-retry logic handled in update_state()
    break;

  case MissionState::LANTERN_FOUND:
    RCLCPP_INFO(this->get_logger(), "Lantern logged! Pausing briefly...");
    // Brief pause handled by state logic
    break;

  case MissionState::RETURN:
    RCLCPP_INFO(this->get_logger(),
                "Returning to start position: [%.2f, %.2f, %.2f] using PLANNER",
                start_position_.x, start_position_.y, takeoff_altitude_);

    // Use the planner to find a safe path back, instead of blind trajectory
    {
      geometry_msgs::msg::PoseStamped return_goal_msg;
      return_goal_msg.header.stamp = this->now();
      return_goal_msg.header.frame_id = "world";
      return_goal_msg.pose.position.x = start_position_.x;
      return_goal_msg.pose.position.y = start_position_.y;
      return_goal_msg.pose.position.z =
          takeoff_altitude_; // Maintain altitude for return
      return_goal_msg.pose.orientation.w = 1.0;

      if (planner_type_ == "A_star") {
        planner_goal_pub_a_->publish(return_goal_msg);
      } else {
        planner_goal_pub_->publish(return_goal_msg);
      }

      current_goal_.x = start_position_.x;
      current_goal_.y = start_position_.y;
      current_goal_.z = takeoff_altitude_;
      goal_set_time_ = this->now();
      last_pose_at_goal_set_ = current_pose_.position;
      goal_active_ = true;
    }
    break;

  case MissionState::LAND:
    RCLCPP_INFO(this->get_logger(), "Landing...");
    // We land at the start position, not the current position
    // We do this using the trajectory goal publisher, so we can get the drone
    // to land at the start position Just to skip all the path planning for
    // going back from entrance to start position
    publish_trajectory_goal(start_position_.x, start_position_.y,
                            start_position_.z);
    goal_active_ = true;
    break;

  default:
    break;
  }
}

void MissionFsmNode::on_state_exit(MissionState state) {
  if (state == MissionState::EXPLORE) {
    auto disable_msg = std_msgs::msg::Bool();
    disable_msg.data = false;
    enable_mapping_pub_->publish(disable_msg);

    // Ensure exploration-side bookkeeping is dropped whenever EXPLORE is
    // left so stale goals/retries cannot leak into subsequent states.
    goal_request_pending_ = false;
    z_retry_altitudes_.clear();
    z_retry_index_ = 0;
    strategic_goal_ = geometry_msgs::msg::Point();
    current_goal_ = geometry_msgs::msg::Point();
    active_goal_source_ = GoalSource::EXPLORER;
    active_goal_anchor_node_id_ = -1;
    single_edge_travel_target_node_id_ = -1;
    potential_objective_timer_active_ = false;
    potential_objective_anchor_node_id_ = -1;
    returning_to_entrance_via_travel_ = false;
    travel_mode_ = false;
    travel_path_.clear();
    active_rrt_waypoints_.clear();
    clear_single_edge_priority_target();
    ++explorer_request_epoch_;
  }
}

void MissionFsmNode::update_state() {
  switch (current_state_) {
  case MissionState::INIT:
    // Wait for pose data and start signal
    if (pose_received_ && mission_start_signal_received_) {
      transition_to(MissionState::TAKEOFF);
    } else {
      // Logging for user feedback
      if (!pose_received_) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Waiting for initial pose from /current_state_est...");
      } else if (!mission_start_signal_received_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Waiting for start signal (publish empty message "
                             "to /mission/start)...");
      }
    }
    break;

  case MissionState::TAKEOFF:
    // Check if takeoff altitude reached
    if (goal_active_) {
      double alt_diff = std::abs(current_pose_.position.z - current_goal_.z);
      if (alt_diff < GOAL_REACHED_THRESHOLD) {
        RCLCPP_INFO(this->get_logger(), "Takeoff complete. Altitude: %.2f m",
                    current_pose_.position.z);
        goal_active_ = false;
        transition_to(MissionState::GOTO_ENTRANCE);
      }
    }
    break;

  case MissionState::GOTO_ENTRANCE:
    // Check if reached cave entrance
    if (goal_active_) {
      double dist = calculate_distance(current_pose_.position, cave_entrance_);
      if (dist < GOAL_REACHED_THRESHOLD) {
        RCLCPP_INFO(this->get_logger(), "Reached cave entrance!");
        goal_active_ = false;
        transition_to(MissionState::EXPLORE);
      }
    }
    break;

  case MissionState::EXPLORE:
    if (returning_to_entrance_via_travel_) {
      // Mission-complete return corridor: keep only travel-to-entrance logic
      // and skip all exploration-node decision machinery.
      if (!goal_active_ && !goal_request_pending_ && macroplanning_enabled_ &&
          travel_mode_ && !travel_path_.empty()) {
        const int next_node_id = travel_path_.front();
        if (graph_nodes_.count(next_node_id) > 0) {
          RCLCPP_INFO(this->get_logger(),
                      "Mission-complete return: traversing checkpoint node %d "
                      "toward entrance.",
                      next_node_id);
          if (!try_activate_exploration_goal(
                  graph_nodes_.at(next_node_id).position, true,
                  GoalSource::TRAVEL, -1)) {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to activate mission-complete return checkpoint "
                        "%d. Skipping node.",
                        next_node_id);
            clear_node_relocation_state(next_node_id);
            travel_path_.pop_front();
            if (travel_path_.empty()) {
              travel_mode_ = false;
            }
          }
        } else {
          travel_path_.pop_front();
          if (travel_path_.empty()) {
            travel_mode_ = false;
          }
        }
      }

      const double entrance_dist =
          calculate_distance(current_pose_.position, cave_entrance_);
      if ((!travel_mode_ || travel_path_.empty()) && !goal_active_ &&
          entrance_dist <= GOAL_REACHED_THRESHOLD) {
        RCLCPP_INFO(this->get_logger(), "Mission-complete return reached cave "
                                        "entrance. Transitioning to RETURN.");
        returning_to_entrance_via_travel_ = false;
        transition_to(MissionState::RETURN);
      }
    }

    if (macroplanning_enabled_) {
      const double potential_prune_radius =
          (active_goal_source_ == GoalSource::POTENTIAL) ? node_radius_ : 10.0;
      prune_potential_nodes_near(current_pose_.position,
                                 potential_prune_radius);
      update_checkpoint_graph();
      update_mode_decision();

      if (single_edge_priority_target_node_id_ >= 0 &&
          graph_nodes_.count(single_edge_priority_target_node_id_) > 0) {
        const int reached_leaf_id = single_edge_priority_target_node_id_;
        const double d_target = calculate_distance(
            current_pose_.position, graph_nodes_.at(reached_leaf_id).position);
        if (d_target <= single_edge_priority_reached_radius_) {
          RCLCPP_INFO(
              this->get_logger(),
              "Reached single-edge priority node radius (%.2f m <= %.2f m).",
              d_target, single_edge_priority_reached_radius_);

          if (!graph_nodes_.at(reached_leaf_id).edges.empty()) {
            const int predecessor_id =
                *graph_nodes_.at(reached_leaf_id).edges.begin();
            set_single_edge_punishment_target(predecessor_id);
          }

          clear_single_edge_priority_target();
        }
      }
    }

    // Check if we've found all lanterns (AND we aren't already returning)
    if (!returning_to_entrance_via_travel_ &&
        lanterns_found_count_ >= TARGET_LANTERN_COUNT) {
      RCLCPP_INFO(this->get_logger(),
                  "All %zu lanterns found! Dropping exploration logic and "
                  "returning to cave entrance via travel mode.",
                  TARGET_LANTERN_COUNT);

      if (goal_active_) {
        cancel_pub_->publish(std_msgs::msg::Empty());
      }
      goal_active_ = false;
      goal_request_pending_ = false;
      ++explorer_request_epoch_;
      active_goal_source_ = GoalSource::TRAVEL;
      active_goal_anchor_node_id_ = -1;
      potential_resolution_node_id_ = -1;
      potential_objective_timer_active_ = false;
      potential_objective_anchor_node_id_ = -1;
      single_edge_travel_target_node_id_ = -1;
      clear_single_edge_priority_target();
      clear_single_edge_punishment_target();

      if (!start_return_to_entrance_via_travel()) {
        RCLCPP_WARN(this->get_logger(),
                    "Unable to build travel corridor to cave entrance. Falling "
                    "back to direct RETURN.");
        returning_to_entrance_via_travel_ = false;
        transition_to(MissionState::RETURN);
      }
      break;
    }

    // Check if we've reached the current exploration goal
    if (goal_active_) {
      if (active_goal_source_ == GoalSource::POTENTIAL &&
          potential_objective_timer_active_) {
        const double potential_objective_age =
            (this->now() - potential_objective_start_time_).seconds();
        if (potential_objective_age > POTENTIAL_OBJECTIVE_TIMEOUT_SECONDS) {
          drop_active_potential_objective(
              "cumulative objective timeout exceeded 60 seconds");
          break;
        }
      }

      if ((active_goal_source_ == GoalSource::TRAVEL ||
           active_goal_source_ == GoalSource::POTENTIAL) &&
          monitor_macroplanning_rrt_waypoints()) {
        break;
      }

      double dist = calculate_distance(current_pose_.position, current_goal_);
      double time_since_goal_set = (this->now() - goal_set_time_).seconds();

      // FIRST: Check if goal reached (most important check - do this first!)
      if (dist < GOAL_REACHED_THRESHOLD) {
        RCLCPP_INFO(this->get_logger(),
                    "Reached exploration goal [%.2f, %.2f, %.2f] "
                    "(distance=%.2f m, took %.1f s). Requesting next goal.",
                    current_goal_.x, current_goal_.y, current_goal_.z, dist,
                    time_since_goal_set);

        // In travel mode, consume the front checkpoint only when we actually
        // reach it (not when activation succeeds). This keeps the queue intact
        // if planning fails and a retry is needed.
        if (macroplanning_enabled_ &&
            active_goal_source_ == GoalSource::TRAVEL &&
            !travel_path_.empty()) {
          const int reached_node_id = travel_path_.front();
          if (graph_nodes_.count(reached_node_id) > 0 &&
              calculate_distance(current_pose_.position,
                                 graph_nodes_.at(reached_node_id).position) <=
                  GOAL_REACHED_THRESHOLD) {
            // Anchor graph state to the reached travel checkpoint immediately.
            // This avoids mode-decision drift when runtime node-containment
            // checks lag or are stricter than GOAL_REACHED_THRESHOLD.
            previous_node_id_ = current_node_id_;
            current_node_id_ = reached_node_id;
            last_visited_node_id_ = reached_node_id;
            clear_node_relocation_state(reached_node_id);
            travel_path_.pop_front();
          }

          if (current_node_id_ >= 0 &&
              graph_nodes_.count(current_node_id_) > 0) {
            const auto degree =
                graph_nodes_[current_node_id_].edges.size() +
                (graph_nodes_[current_node_id_].is_dead_end ? 1 : 0);
            if (degree == 1) {
              travel_mode_ = false;
              travel_path_.clear();
            }
          }

          if (travel_path_.empty()) {
            travel_mode_ = false;
            single_edge_travel_target_node_id_ = -1;
            resume_explorer_mode_after_travel();
          }
        }

        if (macroplanning_enabled_ &&
            active_goal_source_ == GoalSource::POTENTIAL) {
          const int anchor_node = active_goal_anchor_node_id_;
          geometry_msgs::msg::Point next_potential_goal;
          if (anchor_node >= 0 &&
              pop_next_potential_for_node(anchor_node, next_potential_goal)) {
            RCLCPP_INFO(this->get_logger(),
                        "Potential goal reached. Continuing "
                        "potential-resolution queue for anchor %d.",
                        anchor_node);
            if (try_activate_exploration_goal(next_potential_goal, true,
                                              GoalSource::POTENTIAL,
                                              anchor_node)) {
              break;
            }

            RCLCPP_WARN(this->get_logger(),
                        "Failed to activate next potential goal for anchor %d. "
                        "Stopping potential resolution for this anchor.",
                        anchor_node);
          }

          potential_resolution_node_id_ = -1;
          travel_mode_ = false;
        }

        goal_active_ = false;
        active_goal_source_ = GoalSource::EXPLORER;
        active_goal_anchor_node_id_ = -1;
        potential_objective_timer_active_ = false;
        potential_objective_anchor_node_id_ = -1;
        consecutive_too_close_rejections_ =
            0; // Reset counter on successful goal completion
        // Loop will trigger new request in next block
      }
      // SECOND: Check for goal timeout - if goal takes too long, abandon it
      else if (time_since_goal_set > GOAL_TIMEOUT_SECONDS &&
               active_goal_source_ == GoalSource::EXPLORER) {
        RCLCPP_WARN(this->get_logger(),
                    "Exploration goal timeout (%.1f s > %.1f s). "
                    "Switching to recovery for goal [%.2f, %.2f, %.2f] "
                    "(current distance=%.2f m, threshold=%.2f m).",
                    time_since_goal_set, GOAL_TIMEOUT_SECONDS, current_goal_.x,
                    current_goal_.y, current_goal_.z, dist,
                    GOAL_REACHED_THRESHOLD);
        start_refine_for_current_goal("goal timeout");
        break;
      }
      // THIRD: Check if drone is stuck (not making progress toward goal)
      else if (time_since_goal_set > GOAL_TIMEOUT_SECONDS &&
               active_goal_source_ == GoalSource::POTENTIAL) {
        const int anchor_node = active_goal_anchor_node_id_;
        RCLCPP_WARN(this->get_logger(),
                    "Potential goal timeout (%.1f s > %.1f s) for anchor %d. "
                    "Removing timed-out potential and continuing.",
                    time_since_goal_set, GOAL_TIMEOUT_SECONDS, anchor_node);
        goal_active_ = false;

        geometry_msgs::msg::Point next_potential_goal;
        if (anchor_node >= 0 &&
            pop_next_potential_for_node(anchor_node, next_potential_goal) &&
            try_activate_exploration_goal(next_potential_goal, true,
                                          GoalSource::POTENTIAL, anchor_node)) {
          break;
        }

        potential_resolution_node_id_ = -1;
        travel_mode_ = false;
        active_goal_source_ = GoalSource::EXPLORER;
        active_goal_anchor_node_id_ = -1;
        potential_objective_timer_active_ = false;
        potential_objective_anchor_node_id_ = -1;
      }
      // THIRD: Check if drone is stuck (not making progress toward goal)
      else if (time_since_goal_set >
               STUCK_DETECTION_SECONDS) { // After 10 seconds, check movement
        double movement_since_goal =
            calculate_distance(current_pose_.position, last_pose_at_goal_set_);
        double progress_toward_goal =
            calculate_distance(last_pose_at_goal_set_, current_goal_) -
            dist; // How much closer we got

        const bool travel_owned_goal =
            (active_goal_source_ == GoalSource::TRAVEL ||
             active_goal_source_ == GoalSource::POTENTIAL);
        if (travel_owned_goal && time_since_goal_set > GOAL_TIMEOUT_SECONDS &&
            movement_since_goal < MIN_MOVEMENT_THRESHOLD) {
          RCLCPP_WARN(
              this->get_logger(),
              "Travel mode stalled for %.1f s with low movement (%.2f m). "
              "Falling back to explorer mode until a new node is reached.",
              time_since_goal_set, movement_since_goal);
          cancel_pub_->publish(std_msgs::msg::Empty());
          goal_active_ = false;
          active_goal_source_ = GoalSource::EXPLORER;
          active_goal_anchor_node_id_ = -1;
          travel_mode_ = false;
          potential_resolution_node_id_ = -1;
          travel_path_.clear();
          force_explorer_until_new_node_ = true;
          fallback_origin_node_id_ = last_visited_node_id_;
          force_explorer_start_time_ = this->now();
          break;
        }

        if (!travel_owned_goal &&
            movement_since_goal < MIN_MOVEMENT_THRESHOLD) {
          RCLCPP_WARN(this->get_logger(),
                      "Drone appears stuck (moved only %.2f m in %.1f s, "
                      "distance to goal=%.2f m). "
                      "Switching to recovery for goal [%.2f, %.2f, %.2f].",
                      movement_since_goal, time_since_goal_set, dist,
                      current_goal_.x, current_goal_.y, current_goal_.z);
          start_refine_for_current_goal("stuck with low movement");
          break;
        } else if (progress_toward_goal < MIN_PROGRESS_THRESHOLD &&
                   dist > GOAL_REACHED_THRESHOLD * 2.0) {
          // Making movement but not toward goal (might be going around
          // obstacle) Only warn if we're still far from goal
          RCLCPP_DEBUG(this->get_logger(),
                       "Drone moving (%.2f m) but slow progress toward goal "
                       "(%.2f m closer, %.2f m remaining).",
                       movement_since_goal, progress_toward_goal, dist);
        }
      }
    }

    // Recover from request deadlock: if an async goal request remains pending
    // too long, clear the latch so a fresh request can be issued.
    if (goal_request_pending_ && goal_request_timeout_s_ > 0.0) {
      const double pending_age =
          (this->now() - goal_request_sent_time_).seconds();
      if (pending_age > goal_request_timeout_s_) {
        RCLCPP_WARN(this->get_logger(),
                    "Exploration goal request timed out after %.1f s. Clearing "
                    "pending latch.",
                    pending_age);
        goal_request_pending_ = false;
        ++explorer_request_epoch_;
        ++consecutive_goal_request_failures_;
      }
    }

    // Detect goal-selection stuck conditions (no active goal and repeated
    // failures)
    if (!goal_active_) {
      const double time_since_last_success =
          (this->now() - last_successful_exploration_goal_time_).seconds();
      if ((explore_goal_selection_timeout_ > 0.0 &&
           time_since_last_success > explore_goal_selection_timeout_) ||
          (explore_goal_selection_max_failures_ > 0 &&
           consecutive_goal_request_failures_ >=
               explore_goal_selection_max_failures_)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Exploration goal selection appears stuck "
            "(failures=%d, last success=%.1f s ago). "
            "Continuing to request new goals with current parameters.",
            consecutive_goal_request_failures_, time_since_last_success);
      }
    }

    // If no active goal and no pending request -> request one, but only once
    // the exploration map is reported as ready by the exploration_manager.
    // This avoids spamming goal requests while the voxel map is still empty.
    if (!goal_active_ && !goal_request_pending_) {
      if (macroplanning_enabled_ && travel_mode_ &&
          single_edge_travel_target_node_id_ >= 0 &&
          graph_nodes_.count(single_edge_travel_target_node_id_) > 0 &&
          !graph_nodes_.at(single_edge_travel_target_node_id_).edges.empty()) {
        const int predecessor_id =
            *graph_nodes_.at(single_edge_travel_target_node_id_).edges.begin();
        if (graph_nodes_.count(predecessor_id) > 0) {
          const bool at_predecessor =
              (current_node_id_ == predecessor_id) ||
              (last_visited_node_id_ == predecessor_id) ||
              is_inside_node(predecessor_id, current_pose_.position);
          if (at_predecessor) {
            RCLCPP_INFO(
                this->get_logger(),
                "Reached predecessor node %d for single-edge target %d. "
                "Releasing travel mode and handing off to explorer priority.",
                predecessor_id, single_edge_travel_target_node_id_);
            set_single_edge_priority_target(single_edge_travel_target_node_id_);
            travel_mode_ = false;
            travel_path_.clear();
            single_edge_travel_target_node_id_ = -1;
            resume_explorer_mode_after_travel();
          }
        }
      }
      if (macroplanning_enabled_ && travel_mode_ && !travel_path_.empty()) {
        const int next_node_id = travel_path_.front();
        if (graph_nodes_.count(next_node_id) > 0) {
          RCLCPP_INFO(this->get_logger(),
                      "Travel mode: traversing to checkpoint node %d.",
                      next_node_id);
          if (!try_activate_exploration_goal(
                  graph_nodes_.at(next_node_id).position, true,
                  GoalSource::TRAVEL, -1)) {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to activate travel checkpoint node %d. Skipping node.",
                next_node_id);
            clear_node_relocation_state(next_node_id);
            travel_path_.pop_front();
            if (travel_path_.empty()) {
              travel_mode_ = false;
              single_edge_travel_target_node_id_ = -1;
              resume_explorer_mode_after_travel();
            }
          }
        } else {
          travel_path_.pop_front();
          if (travel_path_.empty()) {
            travel_mode_ = false;
            single_edge_travel_target_node_id_ = -1;
            resume_explorer_mode_after_travel();
          }
        }
      } else if (macroplanning_enabled_ && potential_resolution_node_id_ >= 0 &&
                 current_node_id_ == potential_resolution_node_id_) {
        geometry_msgs::msg::Point potential_goal;
        if (pop_next_potential_for_node(potential_resolution_node_id_,
                                        potential_goal)) {
          RCLCPP_INFO(this->get_logger(),
                      "Resolving potential node from anchor %d.",
                      potential_resolution_node_id_);
          if (!try_activate_exploration_goal(potential_goal, true,
                                             GoalSource::POTENTIAL,
                                             potential_resolution_node_id_)) {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to activate potential goal for anchor %d. "
                        "Requesting explorer goal.",
                        potential_resolution_node_id_);
            potential_resolution_node_id_ = -1;
          }
        }
      } else if (exploration_map_ready_) {
        request_exploration_goal();
      } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Waiting for exploration map to be ready before "
                             "requesting goals...");
      }
    }

    // Mid-flight replanning: every replan_interval_s_ seconds while flying
    // toward a goal, ask the planner to recompute from the current position.
    if (goal_active_ && active_goal_source_ == GoalSource::EXPLORER) {
      double time_since_replan = (this->now() - last_replan_time_).seconds();
      if (time_since_replan >= replan_interval_s_) {
        replan_current_goal();
      }
    } else if (goal_active_ && (active_goal_source_ == GoalSource::TRAVEL ||
                                active_goal_source_ == GoalSource::POTENTIAL)) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                            "Macroplanning goal active: keeping single "
                            "published RRT goal (no periodic replan).");
    }
    break;

  case MissionState::LANTERN_FOUND:
    // Brief pause, then return to exploration
    // In a real implementation, add a timer for the pause duration
    transition_to(MissionState::EXPLORE);
    break;

  case MissionState::REFINE_GOAL:
    // Z-Retry Loop: Try different altitudes for the same (x,y) goal
    if (!goal_active_) {
      if (z_retry_index_ < z_retry_altitudes_.size()) {
        double altitude = z_retry_altitudes_[z_retry_index_];
        RCLCPP_INFO(this->get_logger(),
                    "Z-Retry: Attempting altitude %.2f m (attempt %zu/%zu) for "
                    "goal [%.2f, %.2f]",
                    altitude, z_retry_index_ + 1, z_retry_altitudes_.size(),
                    strategic_goal_.x, strategic_goal_.y);

        // Publish goal to planner
        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.stamp = this->now();
        goal_msg.header.frame_id = "world";
        goal_msg.pose.position.x = strategic_goal_.x;
        goal_msg.pose.position.y = strategic_goal_.y;
        goal_msg.pose.position.z = altitude;
        goal_msg.pose.orientation.w = 1.0;

        // Update tracking variables for the new Z-retry goal
        current_goal_.x = goal_msg.pose.position.x;
        current_goal_.y = goal_msg.pose.position.y;
        current_goal_.z = goal_msg.pose.position.z;
        goal_set_time_ = this->now();
        last_pose_at_goal_set_ = current_pose_.position;

        if (planner_type_ == "A_star") {
          planner_goal_pub_a_->publish(goal_msg);
        } else {
          planner_goal_pub_->publish(goal_msg);
        }
        goal_active_ = true;
        z_retry_index_++;
      } else {
        // All Z retries exhausted - blacklist this goal
        RCLCPP_WARN(this->get_logger(),
                    "All Z retries failed for goal [%.2f, %.2f]. Blacklisting.",
                    strategic_goal_.x, strategic_goal_.y);
        if (macroplanning_enabled_) {
          mark_potential_node_unreachable_near(strategic_goal_);
          std::vector<int> provisional_failed_nodes;
          for (const auto &entry : graph_nodes_) {
            if (entry.second.is_provisional &&
                calculate_distance(entry.second.position, strategic_goal_) <=
                    node_radius_) {
              provisional_failed_nodes.push_back(entry.first);
            }
          }
          for (const int node_id : provisional_failed_nodes) {
            remove_checkpoint_node(node_id);
          }
        }

        // Publish to ExplorationManager so it doesn't suggest this again
        geometry_msgs::msg::PointStamped blacklist_msg;
        blacklist_msg.header.stamp = this->now();
        blacklist_msg.header.frame_id = "world";
        blacklist_msg.point = strategic_goal_;
        blacklist_goal_pub_->publish(blacklist_msg);

        // Reset and return to exploration for a new goal
        z_retry_index_ = 0;
        goal_active_ = false;
        transition_to(MissionState::EXPLORE);
      }
    } else {
      // Monitor progress during Z-retry
      double dist = calculate_distance(current_pose_.position, current_goal_);
      double time_since_goal_set = (this->now() - goal_set_time_).seconds();

      // Check if reached
      if (dist < GOAL_REACHED_THRESHOLD) {
        RCLCPP_INFO(this->get_logger(),
                    "Reached refined goal [%.2f, %.2f, %.2f] "
                    "(distance=%.2f m). Resuming exploration.",
                    current_goal_.x, current_goal_.y, current_goal_.z, dist);

        goal_active_ = false;
        // Success! Return to EXPLORE state to request next goal
        transition_to(MissionState::EXPLORE);
      }
      // Check for timeout
      else if (time_since_goal_set > GOAL_TIMEOUT_SECONDS) {
        RCLCPP_WARN(this->get_logger(),
                    "Refined goal timeout (%.1f s). "
                    "Abandoning altitude %.2f m. Trying next option.",
                    time_since_goal_set, current_goal_.z);
        cancel_pub_->publish(std_msgs::msg::Empty());
        goal_active_ = false;
        // Next loop iteration will trigger next Z-retry
      } else if (time_since_goal_set > STUCK_DETECTION_SECONDS) {
        double movement_since_goal =
            calculate_distance(current_pose_.position, last_pose_at_goal_set_);
        if (movement_since_goal < MIN_MOVEMENT_THRESHOLD) {
          RCLCPP_WARN(
              this->get_logger(),
              "Refine goal stalled at altitude %.2f m (moved only %.2f m "
              "in %.1f s). Trying next altitude.",
              current_goal_.z, movement_since_goal, time_since_goal_set);
          cancel_pub_->publish(std_msgs::msg::Empty());
          goal_active_ = false;
        }
      }
    }

    // Mid-flight replanning for REFINE_GOAL state
    if (goal_active_) {
      double time_since_replan = (this->now() - last_replan_time_).seconds();
      if (time_since_replan >= replan_interval_s_) {
        replan_current_goal();
      }
    }
    break;

  case MissionState::RETURN:
    // If goal is no longer active, it means planner either reached it or failed
    // (handled in planner_status_callback)
    if (!goal_active_) {
      // Check if we are actually close to start position
      geometry_msgs::msg::Point return_goal;
      return_goal.x = start_position_.x;
      return_goal.y = start_position_.y;
      return_goal.z = takeoff_altitude_;

      double dist = calculate_distance(current_pose_.position, return_goal);
      if (dist < 2.0) { // Slightly larger threshold for return
        RCLCPP_INFO(this->get_logger(),
                    "Returned to start position (dist=%.2f)! Landing.", dist);
        transition_to(MissionState::LAND);
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Planner finished but drone is %.2f m from start. Landing "
                    "anyway (emergency).",
                    dist);
        transition_to(MissionState::LAND);
      }
    }
    // Also monitor timeout for return
    else if ((this->now() - goal_set_time_).seconds() >
             120.0) { // 2 minutes max for return
      RCLCPP_WARN(this->get_logger(), "Return home timed out!");
      goal_active_ = false;
      transition_to(MissionState::LAND); // Land where we are? Or try again?
                                         // Landing is safer.
    }
    break;

  case MissionState::LAND:
    // Check if landed
    if (goal_active_) {
      if (current_pose_.position.z < 0.2) {
        RCLCPP_INFO(this->get_logger(), "Landed! Mission complete.");
        goal_active_ = false;
        // Could transition to a COMPLETE state or just stay in LAND
      }
    }
    break;
  }
}

double
MissionFsmNode::calculate_distance(const geometry_msgs::msg::Point &p1,
                                   const geometry_msgs::msg::Point &p2) const {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void MissionFsmNode::publish_trajectory_goal(double x, double y, double z,
                                             double yaw) {
  trajectory_msgs::msg::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = this->now();
  trajectory_msg.header.frame_id = "world";

  trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;

  // Set position via transform
  geometry_msgs::msg::Transform transform;
  transform.translation.x = x;
  transform.translation.y = y;
  transform.translation.z = z;

  // Convert yaw to quaternion (simplified: only yaw rotation around Z)
  double half_yaw = yaw / 2.0;
  transform.rotation.x = 0.0;
  transform.rotation.y = 0.0;
  transform.rotation.z = std::sin(half_yaw);
  transform.rotation.w = std::cos(half_yaw);

  point.transforms.push_back(transform);

  // Zero velocity and acceleration
  geometry_msgs::msg::Twist zero_twist;
  point.velocities.push_back(zero_twist);
  point.accelerations.push_back(zero_twist);

  trajectory_msg.points.push_back(point);

  trajectory_pub_->publish(trajectory_msg);

  current_goal_.x = x;
  current_goal_.y = y;
  current_goal_.z = z;

  RCLCPP_DEBUG(this->get_logger(),
               "Published trajectory goal: [%.2f, %.2f, %.2f]", x, y, z);
}

void MissionFsmNode::publish_waypoint_path(
    const std::vector<geometry_msgs::msg::Point> &waypoints) {
  if (waypoints.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "Requested to publish empty waypoint path.");
    return;
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = "world";
  path_msg.poses.reserve(waypoints.size());

  for (const auto &waypoint : waypoints) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position = waypoint;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  waypoint_path_pub_->publish(path_msg);

  current_goal_ = waypoints.back();

  RCLCPP_INFO(
      this->get_logger(),
      "Published waypoint path with %zu poses. Goal: [%.2f, %.2f, %.2f]",
      waypoints.size(), current_goal_.x, current_goal_.y, current_goal_.z);
}

void MissionFsmNode::replan_current_goal() {
  // Republish the current goal to the planner so it recomputes the path
  // from the drone's present position. The trajectory generator will
  // seamlessly replace the old trajectory with the new one.
  geometry_msgs::msg::PoseStamped replan_goal;
  replan_goal.header.stamp = this->now();
  replan_goal.header.frame_id = "world";
  replan_goal.pose.position = current_goal_;
  replan_goal.pose.orientation.w = 1.0;

  if (planner_type_ == "RRT") {
    planner_goal_pub_->publish(replan_goal);
  } else {
    planner_goal_pub_a_->publish(replan_goal);
  }

  last_replan_time_ = this->now();

  RCLCPP_DEBUG(this->get_logger(),
               "Replanned path to [%.2f, %.2f, %.2f] via %s", current_goal_.x,
               current_goal_.y, current_goal_.z, planner_type_.c_str());
}

bool MissionFsmNode::waypoint_appears_unreachable(
    const geometry_msgs::msg::Point &waypoint) const {
  for (const auto &obstacle : recent_obstacle_points_) {
    if (calculate_distance(waypoint, obstacle) <=
        waypoint_obstacle_clearance_) {
      return true;
    }
  }
  return false;
}

bool MissionFsmNode::monitor_macroplanning_rrt_waypoints() {
  if (planner_type_ != "RRT" || active_rrt_waypoints_.empty() ||
      recent_obstacle_points_.empty()) {
    return false;
  }

  while (!active_rrt_waypoints_.empty() &&
         calculate_distance(current_pose_.position,
                            active_rrt_waypoints_.front()) <=
             GOAL_REACHED_THRESHOLD) {
    active_rrt_waypoints_.pop_front();
  }

  if (active_rrt_waypoints_.empty()) {
    return false;
  }

  const geometry_msgs::msg::Point &next_waypoint =
      active_rrt_waypoints_.front();
  if (!waypoint_appears_unreachable(next_waypoint)) {
    return false;
  }

  RCLCPP_WARN(
      this->get_logger(),
      "Detected blocked RRT waypoint [%.2f, %.2f, %.2f] while in %s mode. "
      "Stopping and replanning current goal.",
      next_waypoint.x, next_waypoint.y, next_waypoint.z,
      active_goal_source_ == GoalSource::TRAVEL ? "travel" : "potential");

  cancel_pub_->publish(std_msgs::msg::Empty());
  goal_active_ = false;
  active_rrt_waypoints_.clear();

  const GoalSource source = active_goal_source_;
  const int anchor_node_id = active_goal_anchor_node_id_;

  if (try_activate_exploration_goal(current_goal_, true, source,
                                    anchor_node_id)) {
    return true;
  }

  RCLCPP_WARN(this->get_logger(),
              "Failed to reactivate blocked %s goal. Falling back to existing "
              "failure handling.",
              source == GoalSource::TRAVEL ? "travel" : "potential");

  if (source == GoalSource::POTENTIAL) {
    geometry_msgs::msg::Point next_potential_goal;
    if (anchor_node_id >= 0 &&
        pop_next_potential_for_node(anchor_node_id, next_potential_goal) &&
        try_activate_exploration_goal(next_potential_goal, true,
                                      GoalSource::POTENTIAL, anchor_node_id)) {
      return true;
    }

    potential_resolution_node_id_ = -1;
    travel_mode_ = false;
    active_goal_source_ = GoalSource::EXPLORER;
    active_goal_anchor_node_id_ = -1;
    return false;
  }

  travel_mode_ = false;
  potential_resolution_node_id_ = -1;
  travel_path_.clear();
  resume_explorer_mode_after_travel();
  active_goal_source_ = GoalSource::EXPLORER;
  active_goal_anchor_node_id_ = -1;
  return false;
}

void MissionFsmNode::drop_active_potential_objective(
    const std::string &reason) {
  if (active_goal_source_ != GoalSource::POTENTIAL) {
    return;
  }

  const int anchor_node = active_goal_anchor_node_id_;
  const geometry_msgs::msg::Point dropped_goal = current_goal_;

  RCLCPP_WARN(this->get_logger(),
              "Dropping potential objective [%.2f, %.2f, %.2f] (%s).",
              dropped_goal.x, dropped_goal.y, dropped_goal.z, reason.c_str());

  cancel_pub_->publish(std_msgs::msg::Empty());
  goal_active_ = false;
  active_rrt_waypoints_.clear();
  mark_potential_node_unreachable_near(dropped_goal);

  geometry_msgs::msg::Point next_potential_goal;
  if (anchor_node >= 0 &&
      pop_next_potential_for_node(anchor_node, next_potential_goal) &&
      try_activate_exploration_goal(next_potential_goal, true,
                                    GoalSource::POTENTIAL, anchor_node)) {
    return;
  }

  potential_resolution_node_id_ = -1;
  travel_mode_ = false;
  active_goal_source_ = GoalSource::EXPLORER;
  active_goal_anchor_node_id_ = -1;
  potential_objective_timer_active_ = false;
  potential_objective_anchor_node_id_ = -1;
}

void MissionFsmNode::publish_state() {
  std_msgs::msg::String state_msg;
  state_msg.data = state_to_string(current_state_);
  state_pub_->publish(state_msg);
}

void MissionFsmNode::publish_drone_marker() {
  if (!pose_received_) {
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.stamp = this->now();
  marker.header.frame_id = "world";
  marker.ns = "drone_position";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Position from current_state_est
  marker.pose = current_pose_;

  // Scale (size of the sphere)
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Color (green sphere for drone)
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime =
      rclcpp::Duration::from_seconds(0); // Persistent until updated

  drone_marker_pub_->publish(marker);
}

void MissionFsmNode::publish_checkpoint_markers() {
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker single_edge_nodes_marker;
  single_edge_nodes_marker.header.stamp = this->now();
  single_edge_nodes_marker.header.frame_id = "world";
  single_edge_nodes_marker.ns = "checkpoint_single_edge_nodes";
  single_edge_nodes_marker.id = 0;
  single_edge_nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  single_edge_nodes_marker.action = visualization_msgs::msg::Marker::ADD;
  single_edge_nodes_marker.scale.x = 1.6;
  single_edge_nodes_marker.scale.y = 1.6;
  single_edge_nodes_marker.scale.z = 1.6;
  single_edge_nodes_marker.color.r = 1.0;
  single_edge_nodes_marker.color.g = 0.0;
  single_edge_nodes_marker.color.b = 0.0;
  single_edge_nodes_marker.color.a = 1.0;

  visualization_msgs::msg::Marker multi_edge_nodes_marker;
  multi_edge_nodes_marker.header = single_edge_nodes_marker.header;
  multi_edge_nodes_marker.ns = "checkpoint_multi_edge_nodes";
  multi_edge_nodes_marker.id = 1;
  multi_edge_nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  multi_edge_nodes_marker.action = visualization_msgs::msg::Marker::ADD;
  multi_edge_nodes_marker.scale.x = 1.6;
  multi_edge_nodes_marker.scale.y = 1.6;
  multi_edge_nodes_marker.scale.z = 1.6;
  multi_edge_nodes_marker.color.r = 0.0;
  multi_edge_nodes_marker.color.g = 0.0;
  multi_edge_nodes_marker.color.b = 1.0;
  multi_edge_nodes_marker.color.a = 1.0;

  visualization_msgs::msg::Marker edges_marker;
  edges_marker.header = single_edge_nodes_marker.header;
  edges_marker.ns = "checkpoint_edges";
  edges_marker.id = 2;
  edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges_marker.action = visualization_msgs::msg::Marker::ADD;
  edges_marker.scale.x = 0.35;
  edges_marker.color.r = 0.0;
  edges_marker.color.g = 0.0;
  edges_marker.color.b = 1.0;
  edges_marker.color.a = 1.0;

  visualization_msgs::msg::Marker potential_marker;
  potential_marker.header = single_edge_nodes_marker.header;
  potential_marker.ns = "checkpoint_potential_nodes";
  potential_marker.id = 3;
  potential_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  potential_marker.action = visualization_msgs::msg::Marker::ADD;
  potential_marker.scale.x = 1.2;
  potential_marker.scale.y = 1.2;
  potential_marker.scale.z = 1.2;
  potential_marker.color.r = 1.0;
  potential_marker.color.g = 0.55;
  potential_marker.color.b = 0.0;
  potential_marker.color.a = 1.0;

  visualization_msgs::msg::Marker provisional_marker;
  provisional_marker.header = single_edge_nodes_marker.header;
  provisional_marker.ns = "checkpoint_provisional_nodes";
  provisional_marker.id = 4;
  provisional_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  provisional_marker.action = visualization_msgs::msg::Marker::ADD;
  provisional_marker.scale.x = 1.4;
  provisional_marker.scale.y = 1.4;
  provisional_marker.scale.z = 1.4;
  provisional_marker.color.r = 1.0;
  provisional_marker.color.g = 0.55;
  provisional_marker.color.b = 0.0;
  provisional_marker.color.a = 1.0;

  std::set<std::pair<int, int>> drawn_edges;
  for (const auto &entry : graph_nodes_) {
    const auto &node = entry.second;
    const size_t degree = node.edges.size() + (node.is_dead_end ? 1u : 0u);
    if (node.is_provisional) {
      provisional_marker.points.push_back(node.position);
    } else if (degree <= 1) {
      single_edge_nodes_marker.points.push_back(node.position);
    } else {
      multi_edge_nodes_marker.points.push_back(node.position);
    }

    for (const auto &pot : node.potentials) {
      if (!pot.unreachable) {
        potential_marker.points.push_back(pot.position);
      }
    }

    for (const int neigh : node.edges) {
      if (graph_nodes_.count(neigh) == 0) {
        continue;
      }
      const int a = std::min(node.id, neigh);
      const int b = std::max(node.id, neigh);
      if (!drawn_edges.insert({a, b}).second) {
        continue;
      }
      edges_marker.points.push_back(node.position);
      edges_marker.points.push_back(graph_nodes_.at(neigh).position);
    }
  }

  marker_array.markers.push_back(single_edge_nodes_marker);
  marker_array.markers.push_back(multi_edge_nodes_marker);
  marker_array.markers.push_back(edges_marker);
  marker_array.markers.push_back(potential_marker);
  marker_array.markers.push_back(provisional_marker);
  checkpoint_markers_pub_->publish(marker_array);
}

void MissionFsmNode::start_refine_for_current_goal(const std::string &reason) {
  // If we have no active goal to refine, fallback to normal exploration loop.
  if (!goal_active_) {
    RCLCPP_WARN(this->get_logger(),
                "Requested refine transition without active goal (%s).",
                reason.c_str());
    return;
  }

  // Keep XY fixed and try nearby altitudes. This gives the planner/controller
  // a structured recovery path before we eventually blacklist this goal.
  strategic_goal_ = current_goal_;
  build_z_retry_altitudes(current_goal_.z);
  z_retry_index_ = 0;

  cancel_pub_->publish(std_msgs::msg::Empty());
  goal_active_ = false;

  if (reason.find("stuck") != std::string::npos) {
    suppress_rule_l_ = true;
  }

  RCLCPP_WARN(
      this->get_logger(), "Triggering REFINE_GOAL for [%.2f, %.2f, %.2f] (%s).",
      strategic_goal_.x, strategic_goal_.y, strategic_goal_.z, reason.c_str());
  transition_to(MissionState::REFINE_GOAL);
}

void MissionFsmNode::build_z_retry_altitudes(double center_z) {
  z_retry_altitudes_.clear();
  z_retry_altitudes_.push_back(center_z);

  const int max_attempts = std::max(1, z_retry_max_attempts_);
  int level = 1;
  while (static_cast<int>(z_retry_altitudes_.size()) < max_attempts) {
    z_retry_altitudes_.push_back(center_z - level * z_retry_step_);
    if (static_cast<int>(z_retry_altitudes_.size()) >= max_attempts) {
      break;
    }
    z_retry_altitudes_.push_back(center_z + level * z_retry_step_);
    ++level;
  }
}

void MissionFsmNode::reset_explorer_goal_filters() {
  consecutive_too_close_rejections_ = 0;
  consecutive_goal_request_failures_ = 0;
}

void MissionFsmNode::suspend_explorer_mode_for_travel() {
  if (explorer_mode_suspended_for_travel_) {
    return;
  }

  // Always publish cancel when explorer gets suspended. A planner goal may
  // already have been consumed and converted into a trajectory, so we must
  // explicitly flush downstream execution regardless of local source flags.
  if (goal_active_ && active_goal_source_ == GoalSource::EXPLORER) {
    RCLCPP_INFO(this->get_logger(),
                "Dismissing active explorer goal [%.2f, %.2f, %.2f] while "
                "entering travel mode.",
                current_goal_.x, current_goal_.y, current_goal_.z);
    // Treat as intentionally completed/closed so watchdog bookkeeping does not
    // consider this an unfinished explorer goal.
    last_successful_exploration_goal_time_ = this->now();
    consecutive_too_close_rejections_ = 0;
  }

  cancel_pub_->publish(std_msgs::msg::Empty());
  goal_active_ = false;

  auto disable_msg = std_msgs::msg::Bool();
  disable_msg.data = false;
  enable_mapping_pub_->publish(disable_msg);

  // Any asynchronous service response received after this point is ignored
  // while travel_mode_ is active; also clear local explorer state to prevent
  // stale goals/filters from leaking back when returning to explorer mode.
  goal_request_pending_ = false;
  z_retry_altitudes_.clear();
  z_retry_index_ = 0;
  strategic_goal_ = geometry_msgs::msg::Point();
  current_goal_ = geometry_msgs::msg::Point();
  active_goal_source_ = GoalSource::TRAVEL;
  active_goal_anchor_node_id_ = -1;
  last_pose_at_goal_set_ = current_pose_.position;
  reset_explorer_goal_filters();
  reset_explorer_filters_on_next_goal_ = false;
  ++explorer_request_epoch_;
  explorer_mode_suspended_for_travel_ = true;

  RCLCPP_INFO(
      this->get_logger(),
      "Macroplanning travel mode engaged: explorer goaling state fully reset.");
}

void MissionFsmNode::resume_explorer_mode_after_travel() {
  if (!explorer_mode_suspended_for_travel_) {
    return;
  }

  const bool travel_still_leading =
      travel_mode_ ||
      (goal_active_ && (active_goal_source_ == GoalSource::TRAVEL ||
                        active_goal_source_ == GoalSource::POTENTIAL));
  if (travel_still_leading) {
    RCLCPP_DEBUG(this->get_logger(), "Keeping explorer mode suspended while "
                                     "macroplanning travel still leads.");
    return;
  }

  auto enable_msg = std_msgs::msg::Bool();
  enable_msg.data = true;
  enable_mapping_pub_->publish(enable_msg);

  z_retry_altitudes_.clear();
  z_retry_index_ = 0;
  strategic_goal_ = geometry_msgs::msg::Point();
  current_goal_ = geometry_msgs::msg::Point();
  active_goal_source_ = GoalSource::EXPLORER;
  active_goal_anchor_node_id_ = -1;

  reset_explorer_goal_filters();
  last_successful_exploration_goal_time_ = this->now();
  explorer_mode_suspended_for_travel_ = false;
}

bool MissionFsmNode::try_activate_exploration_goal(
    const geometry_msgs::msg::Point &goal, bool allow_close_goal,
    GoalSource source, int anchor_node_id) {
  if (source != GoalSource::EXPLORER) {
    reset_explorer_filters_on_next_goal_ = true;
  } else if (reset_explorer_filters_on_next_goal_) {
    RCLCPP_INFO(this->get_logger(),
                "Returning to explorer goaling after macroplanning mode. "
                "Resetting exploration-goal filters.");
    reset_explorer_goal_filters();
    reset_explorer_filters_on_next_goal_ = false;
  }

  build_z_retry_altitudes(goal.z);
  z_retry_index_ = 0;
  if (z_retry_altitudes_.empty()) {
    return false;
  }

  geometry_msgs::msg::PoseStamped planner_goal;
  planner_goal.header.stamp = this->now();
  planner_goal.header.frame_id = "world";
  planner_goal.pose.position.x = goal.x;
  planner_goal.pose.position.y = goal.y;
  planner_goal.pose.position.z = z_retry_altitudes_[z_retry_index_++];
  planner_goal.pose.orientation.w = 1.0;

  double goal_dist =
      std::hypot(planner_goal.pose.position.x - current_pose_.position.x,
                 planner_goal.pose.position.y - current_pose_.position.y);

  if (!allow_close_goal && goal_dist < min_exploration_goal_distance_) {
    ++consecutive_too_close_rejections_;
    if (consecutive_too_close_rejections_ <
        MAX_CONSECUTIVE_TOO_CLOSE_REJECTIONS) {
      RCLCPP_WARN(this->get_logger(),
                  "Exploration goal too close (%.2f m < %.2f m). "
                  "Rejecting (consecutive rejections: %d/%d).",
                  goal_dist, min_exploration_goal_distance_,
                  consecutive_too_close_rejections_,
                  MAX_CONSECUTIVE_TOO_CLOSE_REJECTIONS);
      return false;
    }

    RCLCPP_WARN(
        this->get_logger(),
        "Exploration goal too close (%.2f m < %.2f m), but accepting anyway "
        "after %d consecutive rejections to avoid deadlock.",
        goal_dist, min_exploration_goal_distance_,
        consecutive_too_close_rejections_);
    consecutive_too_close_rejections_ = 0;
  } else {
    consecutive_too_close_rejections_ = 0;
  }

  active_rrt_waypoints_.clear();
  current_goal_ = planner_goal.pose.position;
  goal_set_time_ = this->now();
  last_pose_at_goal_set_ = current_pose_.position;

  if (travel_mode_ || source == GoalSource::POTENTIAL ||
      source == GoalSource::TRAVEL) {
    planner_goal_pub_->publish(planner_goal);
  } else if (planner_type_ == "A_star") {
    planner_goal_pub_a_->publish(planner_goal);
  } else {
    planner_goal_pub_->publish(planner_goal);
  }
  goal_active_ = true;
  active_goal_source_ = source;
  active_goal_anchor_node_id_ = anchor_node_id;

  if (source == GoalSource::POTENTIAL) {
    const bool same_anchor =
        potential_objective_timer_active_ &&
        potential_objective_anchor_node_id_ == anchor_node_id;
    const bool same_goal =
        same_anchor && calculate_distance(potential_objective_goal_,
                                          planner_goal.pose.position) <=
                           GOAL_REACHED_THRESHOLD;
    if (!same_goal) {
      potential_objective_start_time_ = this->now();
      potential_objective_goal_ = planner_goal.pose.position;
      potential_objective_anchor_node_id_ = anchor_node_id;
      potential_objective_timer_active_ = true;
    }
  } else {
    potential_objective_timer_active_ = false;
    potential_objective_anchor_node_id_ = -1;
  }

  RCLCPP_INFO(this->get_logger(),
              "Navigating to exploration goal via planner: [%.2f, %.2f, %.2f] "
              "(distance=%.2f m)",
              planner_goal.pose.position.x, planner_goal.pose.position.y,
              planner_goal.pose.position.z, goal_dist);

  return true;
}

void MissionFsmNode::request_exploration_goal() {
  if (explorer_mode_suspended_for_travel_) {
    return;
  }

  // Check if service is ready (non-blocking)
  if (!exploration_goal_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Exploration service not available yet... Retrying soon.");
    ++consecutive_goal_request_failures_;
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "Requesting exploration goal from service...");

  auto request =
      std::make_shared<exploring::srv::GetExplorationGoal::Request>();

  // Set flag to prevent duplicate requests
  goal_request_pending_ = true;
  goal_request_sent_time_ = this->now();
  const uint64_t request_epoch = explorer_request_epoch_;

  // Async call with callback
  exploration_goal_client_->async_send_request(
      request,
      [this, request_epoch](
          rclcpp::Client<exploring::srv::GetExplorationGoal>::SharedFuture
              future) {
        // Clear flag when response received
        goal_request_pending_ = false;

        if (request_epoch != explorer_request_epoch_ ||
            explorer_mode_suspended_for_travel_) {
          RCLCPP_DEBUG(this->get_logger(),
                       "Discarding stale exploration goal response (epoch "
                       "mismatch or explorer suspended).");
          return;
        }

        std::shared_ptr<exploring::srv::GetExplorationGoal::Response> response;
        try {
          response = future.get();
        } catch (const std::exception &ex) {
          RCLCPP_ERROR(this->get_logger(),
                       "Exploration goal service call threw exception: %s",
                       ex.what());
          ++consecutive_goal_request_failures_;
          return;
        }

        if (!response->success) {
          RCLCPP_WARN(this->get_logger(), "Exploration goal request failed: %s",
                      response->message.c_str());
          // Track consecutive failures so EXPLORE can detect goal-selection
          // issues
          ++consecutive_goal_request_failures_;
          return;
        }

        // Check if still in EXPLORE state (might have transitioned)
        if (current_state_ != MissionState::EXPLORE || travel_mode_ ||
            potential_resolution_node_id_ >= 0 ||
            active_goal_source_ == GoalSource::TRAVEL ||
            active_goal_source_ == GoalSource::POTENTIAL) {
          RCLCPP_DEBUG(this->get_logger(),
                       "Ignoring exploration goal - EXPLORE no longer owns "
                       "command chain.");
          return;
        }

        // Store the strategic goal
        strategic_goal_.x = response->goal.pose.position.x;
        strategic_goal_.y = response->goal.pose.position.y;
        strategic_goal_.z = response->goal.pose.position.z;

        RCLCPP_INFO(this->get_logger(),
                    "Received exploration goal: [%.2f, %.2f, %.2f]",
                    strategic_goal_.x, strategic_goal_.y, strategic_goal_.z);

        if (macroplanning_enabled_) {
          register_potential_node_for_anchor(strategic_goal_);
        }

        if (!try_activate_exploration_goal(strategic_goal_)) {
          goal_request_pending_ = false;
          request_exploration_goal();
          return;
        }

        // Successful goal activation: reset failure tracking
        consecutive_goal_request_failures_ = 0;
        last_successful_exploration_goal_time_ = this->now();
      });
}

bool MissionFsmNode::is_inside_node(
    int node_id, const geometry_msgs::msg::Point &pos) const {
  const auto it = graph_nodes_.find(node_id);
  if (it == graph_nodes_.end()) {
    return false;
  }
  return std::hypot(it->second.position.x - pos.x,
                    it->second.position.y - pos.y) <= node_radius_;
}

std::optional<int> MissionFsmNode::find_node_containing_position(
    const geometry_msgs::msg::Point &pos) const {
  int closest_id = -1;
  double closest_dist = std::numeric_limits<double>::max();
  for (const auto &entry : graph_nodes_) {
    // Node containment is evaluated on the horizontal plane so regular
    // altitude corrections do not make us miss node-entry events.
    const double d = std::hypot(entry.second.position.x - pos.x,
                                entry.second.position.y - pos.y);
    if (d <= node_radius_ && d < closest_dist) {
      closest_dist = d;
      closest_id = entry.first;
    }
  }
  if (closest_id < 0) {
    return std::nullopt;
  }
  return closest_id;
}

int MissionFsmNode::create_checkpoint_node(const geometry_msgs::msg::Point &pos,
                                           bool is_entrance,
                                           bool is_provisional) {
  int nearest_existing_id = -1;
  double nearest_existing_dist = std::numeric_limits<double>::max();
  for (const auto &entry : graph_nodes_) {
    const double d = calculate_distance(entry.second.position, pos);
    if (d < nodes_distance_ && d < nearest_existing_dist) {
      nearest_existing_dist = d;
      nearest_existing_id = entry.first;
    }
  }

  if (nearest_existing_id >= 0) {
    // Hard rule: never create a node within nodes_distance_ of another node.
    // For entrance bootstrap, we still allow reusing the closest existing node.
    if (!is_entrance) {
      return -1;
    }

    auto &existing = graph_nodes_[nearest_existing_id];
    existing.is_dead_end = true;
    return nearest_existing_id;
  }

  const int node_id = next_node_id_++;
  CheckpointNode node;
  node.id = node_id;
  node.position = pos;
  node.is_dead_end = is_entrance;
  node.is_provisional = is_provisional;
  node.visit_count = 1;
  graph_nodes_[node_id] = node;

  // Event-based cleanup: whenever a new node is generated, re-validate all
  // potentials against the current graph (nodes + edges).
  prune_potentials_within_node_distance_recursive();
  prune_potential_nodes_near(pos, nodes_distance_);

  if (!is_provisional) {
    std::vector<int> provisional_to_remove;
    for (const auto &entry : graph_nodes_) {
      if (entry.first == node_id) {
        continue;
      }
      if (entry.second.is_provisional &&
          calculate_distance(entry.second.position, pos) <= nodes_distance_) {
        provisional_to_remove.push_back(entry.first);
      }
    }
    for (const int provisional_id : provisional_to_remove) {
      remove_checkpoint_node(provisional_id);
    }
  }
  return node_id;
}

void MissionFsmNode::add_edge_between_nodes(int from_node, int to_node) {
  if (from_node < 0 || to_node < 0 || from_node == to_node ||
      graph_nodes_.count(from_node) == 0 || graph_nodes_.count(to_node) == 0) {
    return;
  }
  graph_nodes_[from_node].edges.insert(to_node);
  graph_nodes_[to_node].edges.insert(from_node);
}

void MissionFsmNode::remove_checkpoint_node(int node_id) {
  const auto it = graph_nodes_.find(node_id);
  if (it == graph_nodes_.end()) {
    return;
  }

  const auto neighbors = it->second.edges;
  for (const int neighbor : neighbors) {
    if (graph_nodes_.count(neighbor) > 0) {
      graph_nodes_[neighbor].edges.erase(node_id);
    }
  }

  if (current_node_id_ == node_id) {
    current_node_id_ = -1;
  }
  if (previous_node_id_ == node_id) {
    previous_node_id_ = -1;
  }
  if (last_visited_node_id_ == node_id) {
    last_visited_node_id_ = entrance_node_id_;
  }
  if (potential_resolution_node_id_ == node_id) {
    potential_resolution_node_id_ = -1;
  }

  travel_path_.erase(
      std::remove(travel_path_.begin(), travel_path_.end(), node_id),
      travel_path_.end());
  clear_node_relocation_state(node_id);
  graph_nodes_.erase(node_id);
}

void MissionFsmNode::prune_potential_nodes_near(
    const geometry_msgs::msg::Point &pos, double radius) {
  for (auto &entry : graph_nodes_) {
    auto &pots = entry.second.potentials;
    pots.erase(std::remove_if(pots.begin(), pots.end(),
                              [&](const PotentialNode &pot) {
                                return !pot.unreachable &&
                                       calculate_distance(pos, pot.position) <=
                                           radius;
                              }),
               pots.end());
  }
}

double MissionFsmNode::point_to_segment_distance(
    const geometry_msgs::msg::Point &p, const geometry_msgs::msg::Point &a,
    const geometry_msgs::msg::Point &b) const {
  const double abx = b.x - a.x;
  const double aby = b.y - a.y;
  const double abz = b.z - a.z;
  const double apx = p.x - a.x;
  const double apy = p.y - a.y;
  const double apz = p.z - a.z;

  const double ab_len_sq = abx * abx + aby * aby + abz * abz;
  if (ab_len_sq < 1e-9) {
    return calculate_distance(p, a);
  }

  const double t =
      std::clamp((apx * abx + apy * aby + apz * abz) / ab_len_sq, 0.0, 1.0);
  geometry_msgs::msg::Point projection;
  projection.x = a.x + t * abx;
  projection.y = a.y + t * aby;
  projection.z = a.z + t * abz;
  return calculate_distance(p, projection);
}

double MissionFsmNode::projection_parameter_on_segment(
    const geometry_msgs::msg::Point &p, const geometry_msgs::msg::Point &a,
    const geometry_msgs::msg::Point &b) const {
  const double abx = b.x - a.x;
  const double aby = b.y - a.y;
  const double abz = b.z - a.z;
  const double apx = p.x - a.x;
  const double apy = p.y - a.y;
  const double apz = p.z - a.z;
  const double ab_len_sq = abx * abx + aby * aby + abz * abz;
  if (ab_len_sq < 1e-9) {
    return 0.0;
  }
  return std::clamp((apx * abx + apy * aby + apz * abz) / ab_len_sq, 0.0, 1.0);
}

void MissionFsmNode::fuse_nearby_checkpoint_nodes() {
  if (graph_nodes_.size() < 2) {
    return;
  }

  std::vector<int> node_ids;
  node_ids.reserve(graph_nodes_.size());
  for (const auto &entry : graph_nodes_) {
    node_ids.push_back(entry.first);
  }

  std::unordered_map<int, int> parent;
  for (const int id : node_ids) {
    parent[id] = id;
  }

  std::function<int(int)> find_root = [&](int x) {
    while (parent[x] != x) {
      parent[x] = parent[parent[x]];
      x = parent[x];
    }
    return x;
  };

  auto unite = [&](int a, int b) {
    const int ra = find_root(a);
    const int rb = find_root(b);
    if (ra != rb) {
      parent[rb] = ra;
    }
  };

  for (size_t i = 0; i < node_ids.size(); ++i) {
    for (size_t j = i + 1; j < node_ids.size(); ++j) {
      const int a = node_ids[i];
      const int b = node_ids[j];
      if (calculate_distance(graph_nodes_.at(a).position,
                             graph_nodes_.at(b).position) <= node_radius_) {
        unite(a, b);
      }
    }
  }

  std::unordered_map<int, std::vector<int>> groups;
  for (const int id : node_ids) {
    groups[find_root(id)].push_back(id);
  }

  std::unordered_map<int, int> remap;
  std::unordered_map<int, CheckpointNode> rebuilt;

  for (const auto &group_entry : groups) {
    const auto &members = group_entry.second;
    if (members.empty()) {
      continue;
    }

    int representative = members.front();
    if (std::find(members.begin(), members.end(), entrance_node_id_) !=
        members.end()) {
      representative = entrance_node_id_;
    }

    CheckpointNode merged;
    merged.id = representative;
    merged.position = graph_nodes_.at(representative).position;

    double sx = 0.0, sy = 0.0, sz = 0.0;
    bool any_dead_end = false;
    bool any_non_provisional = false;
    std::vector<PotentialNode> merged_potentials;

    for (const int id : members) {
      remap[id] = representative;
      const auto &node = graph_nodes_.at(id);
      sx += node.position.x;
      sy += node.position.y;
      sz += node.position.z;
      any_dead_end = any_dead_end || node.is_dead_end;
      any_non_provisional = any_non_provisional || !node.is_provisional;
      merged_potentials.insert(merged_potentials.end(), node.potentials.begin(),
                               node.potentials.end());
    }

    const double inv = 1.0 / static_cast<double>(members.size());
    merged.position.x = sx * inv;
    merged.position.y = sy * inv;
    merged.position.z = sz * inv;
    merged.is_dead_end = any_dead_end;
    merged.is_provisional = !any_non_provisional;

    for (const auto &pot : merged_potentials) {
      bool duplicate = false;
      for (const auto &existing : merged.potentials) {
        if (calculate_distance(existing.position, pot.position) <=
            node_radius_) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        merged.potentials.push_back(pot);
      }
    }

    rebuilt[representative] = merged;
  }

  for (const auto &entry : graph_nodes_) {
    const int from_rep = remap[entry.first];
    for (const int neigh : entry.second.edges) {
      if (remap.count(neigh) == 0) {
        continue;
      }
      const int to_rep = remap[neigh];
      if (from_rep == to_rep) {
        continue;
      }
      rebuilt[from_rep].edges.insert(to_rep);
      rebuilt[to_rep].edges.insert(from_rep);
    }
  }

  auto remap_id = [&](int &id) {
    if (remap.count(id) > 0) {
      id = remap[id];
    }
  };
  remap_id(entrance_node_id_);
  remap_id(last_visited_node_id_);
  remap_id(current_node_id_);
  remap_id(previous_node_id_);
  remap_id(potential_resolution_node_id_);
  remap_id(active_goal_anchor_node_id_);
  remap_id(fallback_origin_node_id_);

  for (auto &id : travel_path_) {
    remap_id(id);
  }
  travel_path_.erase(std::unique(travel_path_.begin(), travel_path_.end()),
                     travel_path_.end());

  graph_nodes_ = std::move(rebuilt);
}

void MissionFsmNode::redistribute_edges_through_nearby_nodes() {
  if (graph_nodes_.size() < 3) {
    return;
  }

  std::set<std::pair<int, int>> unique_edges;
  for (const auto &entry : graph_nodes_) {
    for (const int neigh : entry.second.edges) {
      unique_edges.insert(
          {std::min(entry.first, neigh), std::max(entry.first, neigh)});
    }
  }

  for (const auto &edge : unique_edges) {
    const int a = edge.first;
    const int b = edge.second;
    if (graph_nodes_.count(a) == 0 || graph_nodes_.count(b) == 0) {
      continue;
    }

    std::vector<std::pair<double, int>> split_nodes;
    for (const auto &entry : graph_nodes_) {
      const int c = entry.first;
      if (c == a || c == b) {
        continue;
      }
      const double dist = point_to_segment_distance(
          entry.second.position, graph_nodes_.at(a).position,
          graph_nodes_.at(b).position);
      if (dist <= node_radius_) {
        const double t = projection_parameter_on_segment(
            entry.second.position, graph_nodes_.at(a).position,
            graph_nodes_.at(b).position);
        split_nodes.push_back({t, c});
      }
    }

    if (split_nodes.empty()) {
      continue;
    }

    std::sort(
        split_nodes.begin(), split_nodes.end(),
        [](const auto &lhs, const auto &rhs) { return lhs.first < rhs.first; });

    graph_nodes_[a].edges.erase(b);
    graph_nodes_[b].edges.erase(a);

    int previous = a;
    for (const auto &split_entry : split_nodes) {
      const int node_id = split_entry.second;
      add_edge_between_nodes(previous, node_id);
      previous = node_id;
    }
    add_edge_between_nodes(previous, b);
  }
}

void MissionFsmNode::simplify_checkpoint_graph() {
  if (graph_nodes_.size() < 2) {
    return;
  }
  fuse_nearby_checkpoint_nodes();
  redistribute_edges_through_nearby_nodes();
  prune_potentials_within_node_distance_recursive();
}

bool MissionFsmNode::is_potential_valid_global(
    const geometry_msgs::msg::Point &candidate) const {
  for (const auto &entry : graph_nodes_) {
    // "Within node_distance" must include points exactly on the threshold.
    if (xy_distance(entry.second.position, candidate) <= nodes_distance_) {
      return false;
    }
  }

  std::set<std::pair<int, int>> unique_edges;
  for (const auto &entry : graph_nodes_) {
    for (const int neigh : entry.second.edges) {
      unique_edges.insert(
          {std::min(entry.first, neigh), std::max(entry.first, neigh)});
    }
  }

  for (const auto &edge : unique_edges) {
    if (graph_nodes_.count(edge.first) == 0 ||
        graph_nodes_.count(edge.second) == 0) {
      continue;
    }
    const auto &from = graph_nodes_.at(edge.first).position;
    const auto &to = graph_nodes_.at(edge.second).position;
    if (point_to_segment_distance_xy(candidate, from, to) <= nodes_distance_) {
      return false;
    }
  }

  return true;
}

void MissionFsmNode::prune_potentials_within_node_distance_recursive() {
  if (graph_nodes_.empty()) {
    return;
  }

  // Snapshot current nodes and edges once, then filter every potential using
  // strict XY rules against both node points and full graph edges.
  std::vector<geometry_msgs::msg::Point> node_positions;
  node_positions.reserve(graph_nodes_.size());
  for (const auto &entry : graph_nodes_) {
    node_positions.push_back(entry.second.position);
  }

  std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>>
      edge_segments;
  std::set<std::pair<int, int>> unique_edges;
  for (const auto &entry : graph_nodes_) {
    for (const int neigh : entry.second.edges) {
      unique_edges.insert(
          {std::min(entry.first, neigh), std::max(entry.first, neigh)});
    }
  }
  edge_segments.reserve(unique_edges.size());
  for (const auto &edge : unique_edges) {
    if (graph_nodes_.count(edge.first) == 0 ||
        graph_nodes_.count(edge.second) == 0) {
      continue;
    }
    edge_segments.push_back({graph_nodes_.at(edge.first).position,
                             graph_nodes_.at(edge.second).position});
  }

  for (auto &anchor_entry : graph_nodes_) {
    auto &potentials = anchor_entry.second.potentials;
    potentials.erase(
        std::remove_if(potentials.begin(), potentials.end(),
                       [&](const PotentialNode &potential) {
                         for (const auto &node_position : node_positions) {
                           if (xy_distance(potential.position, node_position) <=
                               nodes_distance_) {
                             return true;
                           }
                         }

                         for (const auto &edge : edge_segments) {
                           if (point_to_segment_distance_xy(
                                   potential.position, edge.first,
                                   edge.second) <= nodes_distance_) {
                             return true;
                           }
                         }
                         return false;
                       }),
        potentials.end());
  }
}

bool MissionFsmNode::is_within_node_distance_of_any_node(
    const geometry_msgs::msg::Point &pos) const {
  for (const auto &entry : graph_nodes_) {
    if (calculate_distance(pos, entry.second.position) <= nodes_distance_) {
      return true;
    }
  }
  return false;
}

bool MissionFsmNode::find_safer_node_position(
    const geometry_msgs::msg::Point &center,
    geometry_msgs::msg::Point &safe_out) const {
  if (recent_obstacle_points_.empty()) {
    return false;
  }

  auto clearance_at = [&](const geometry_msgs::msg::Point &candidate) {
    double min_dist = std::numeric_limits<double>::max();
    for (const auto &obs : recent_obstacle_points_) {
      min_dist = std::min(min_dist, calculate_distance(candidate, obs));
    }
    return min_dist;
  };

  geometry_msgs::msg::Point best = center;
  double best_clearance = clearance_at(center);
  bool improved = false;

  static constexpr double kMaxSearchRadius = 5.0;
  static constexpr double kRadiusStep = 1.0;
  static constexpr int kAngleSamples = 16;
  const double kPi = std::acos(-1.0);

  for (double r = kRadiusStep; r <= kMaxSearchRadius + 1e-6; r += kRadiusStep) {
    for (int i = 0; i < kAngleSamples; ++i) {
      const double theta = 2.0 * kPi * static_cast<double>(i) /
                           static_cast<double>(kAngleSamples);
      geometry_msgs::msg::Point candidate = center;
      candidate.x += r * std::cos(theta);
      candidate.y += r * std::sin(theta);

      if (is_within_node_distance_of_any_node(candidate)) {
        continue;
      }

      const double clearance = clearance_at(candidate);
      if (clearance > best_clearance) {
        best = candidate;
        best_clearance = clearance;
        improved = true;
      }
    }
  }

  if (improved) {
    safe_out = best;
  }
  return improved;
}

bool MissionFsmNode::find_alternate_position_for_node(
    int node_id, geometry_msgs::msg::Point &candidate_out) {
  const auto node_it = graph_nodes_.find(node_id);
  if (node_it == graph_nodes_.end()) {
    return false;
  }

  auto origin_it = node_relocation_origin_.find(node_id);
  if (origin_it == node_relocation_origin_.end()) {
    node_relocation_origin_[node_id] = node_it->second.position;
    origin_it = node_relocation_origin_.find(node_id);
  }
  const auto &origin = origin_it->second;

  static constexpr std::array<double, 5> kSearchRadii{
      {2.0, 4.0, 6.0, 8.0, 10.0}};
  static constexpr int kAngleSamples = 16;
  const double kPi = std::acos(-1.0);

  auto &attempt_counter = node_relocation_attempts_[node_id];
  const int total_candidates =
      static_cast<int>(kSearchRadii.size()) * kAngleSamples;

  for (int idx = attempt_counter; idx < total_candidates; ++idx) {
    const int radius_idx = idx / kAngleSamples;
    const int angle_idx = idx % kAngleSamples;
    const double theta = 2.0 * kPi * static_cast<double>(angle_idx) /
                         static_cast<double>(kAngleSamples);

    geometry_msgs::msg::Point candidate = origin;
    candidate.x += kSearchRadii[radius_idx] * std::cos(theta);
    candidate.y += kSearchRadii[radius_idx] * std::sin(theta);

    bool too_close_to_other_node = false;
    const double node_separation = std::max(1.0, nodes_distance_ * 0.5);
    for (const auto &entry : graph_nodes_) {
      if (entry.first == node_id) {
        continue;
      }
      if (calculate_distance(candidate, entry.second.position) <
          node_separation) {
        too_close_to_other_node = true;
        break;
      }
    }

    if (too_close_to_other_node) {
      continue;
    }

    attempt_counter = idx + 1;
    candidate_out = candidate;
    return true;
  }

  attempt_counter = total_candidates;
  return false;
}

void MissionFsmNode::clear_node_relocation_state(int node_id) {
  node_relocation_origin_.erase(node_id);
  node_relocation_attempts_.erase(node_id);
}

void MissionFsmNode::register_potential_node_for_anchor(
    const geometry_msgs::msg::Point &candidate) {
  if (current_state_ != MissionState::EXPLORE) {
    return;
  }

  const int anchor_id =
      (current_node_id_ >= 0 && graph_nodes_.count(current_node_id_) > 0)
          ? current_node_id_
          : last_visited_node_id_;
  // Keep harvesting potential nodes while traveling or resolving existing
  // potentials so the graph can keep expanding branch hypotheses online.
  if (anchor_id < 0 || graph_nodes_.count(anchor_id) == 0) {
    return;
  }

  auto &anchor = graph_nodes_[anchor_id];
  if (anchor.visit_count > 3) {
    RCLCPP_DEBUG(
        this->get_logger(),
        "Skipping potential registration for node %d: visit_count=%d (>3).",
        anchor_id, anchor.visit_count);
    return;
  }

  if (!is_potential_valid_global(candidate)) {
    return;
  }

  const double candidate_angle = std::atan2(candidate.y - anchor.position.y,
                                            candidate.x - anchor.position.x);
  const double candidate_radius =
      calculate_distance(anchor.position, candidate);
  const double kPi = std::acos(-1.0);
  auto angle_delta = [kPi](double a, double b) {
    double d = std::fabs(a - b);
    while (d > kPi) {
      d = std::fabs(d - 2.0 * kPi);
    }
    return d;
  };

  bool has_distinct_angle = anchor.potentials.empty();
  for (auto &pot : anchor.potentials) {
    if (pot.unreachable) {
      continue;
    }
    const double existing_angle = std::atan2(
        pot.position.y - anchor.position.y, pot.position.x - anchor.position.x);
    const double existing_radius =
        calculate_distance(anchor.position, pot.position);
    const double delta = angle_delta(candidate_angle, existing_angle);
    const double angle_threshold =
        std::clamp(potential_angle_threshold_deg_, 1.0, 179.0) * (kPi / 180.0);
    if (delta < angle_threshold) {
      if (candidate_radius > existing_radius) {
        pot.position = candidate;
        pot.unreachable = false;
      }
      return;
    }
    has_distinct_angle = true;
  }

  if (has_distinct_angle) {
    PotentialNode new_potential;
    new_potential.position = candidate;
    new_potential.valid = true;
    new_potential.unreachable = false;
    anchor.potentials.push_back(new_potential);
    prune_potentials_within_node_distance_recursive();
  }
}

bool MissionFsmNode::node_has_resolvable_potential(int node_id) const {
  const auto it = graph_nodes_.find(node_id);
  if (it == graph_nodes_.end()) {
    return false;
  }
  for (const auto &pot : it->second.potentials) {
    if (!pot.unreachable) {
      return true;
    }
  }
  return false;
}

bool MissionFsmNode::pop_next_potential_for_node(
    int node_id, geometry_msgs::msg::Point &goal_out) {
  auto it = graph_nodes_.find(node_id);
  if (it == graph_nodes_.end()) {
    return false;
  }

  auto &pots = it->second.potentials;
  for (auto pot_it = pots.begin(); pot_it != pots.end(); ++pot_it) {
    if (pot_it->unreachable) {
      continue;
    }
    goal_out = pot_it->position;
    pots.erase(pot_it);
    return true;
  }
  return false;
}

std::vector<int>
MissionFsmNode::compute_shortest_path_nodes(int start_node,
                                            int goal_node) const {
  std::queue<int> q;
  std::unordered_map<int, int> parent;
  q.push(start_node);
  parent[start_node] = -1;

  while (!q.empty()) {
    const int current = q.front();
    q.pop();
    if (current == goal_node) {
      break;
    }
    const auto it = graph_nodes_.find(current);
    if (it == graph_nodes_.end()) {
      continue;
    }
    for (const int neigh : it->second.edges) {
      if (parent.count(neigh) > 0) {
        continue;
      }
      parent[neigh] = current;
      q.push(neigh);
    }
  }

  if (parent.count(goal_node) == 0) {
    return {};
  }

  std::vector<int> path;
  for (int at = goal_node; at >= 0; at = parent[at]) {
    path.push_back(at);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

void MissionFsmNode::reset_graph_to_entrance() {
  if (entrance_node_id_ < 0 || graph_nodes_.count(entrance_node_id_) == 0) {
    return;
  }

  CheckpointNode entrance = graph_nodes_[entrance_node_id_];
  entrance.edges.clear();
  entrance.potentials.clear();
  entrance.is_dead_end = true;

  graph_nodes_.clear();
  graph_nodes_[entrance_node_id_] = entrance;
  last_visited_node_id_ = entrance_node_id_;
  current_node_id_ = entrance_node_id_;
  previous_node_id_ = entrance_node_id_;
  travel_mode_ = false;
  potential_resolution_node_id_ = -1;
  travel_path_.clear();
}

void MissionFsmNode::update_checkpoint_graph() {
  if (graph_nodes_.empty()) {
    return;
  }

  previous_node_id_ = current_node_id_;
  const auto containing = find_node_containing_position(current_pose_.position);
  current_node_id_ = containing.value_or(-1);

  // Bootstrap phase: keep graph logic disabled until we have entrance + one
  // in-cave node.
  if (graph_nodes_.size() < 2) {
    travel_mode_ = false;
    potential_resolution_node_id_ = -1;
    travel_path_.clear();
    suppress_rule_l_ = false;

    if (entrance_node_id_ >= 0 && graph_nodes_.count(entrance_node_id_) > 0) {
      if (current_node_id_ == entrance_node_id_) {
        last_visited_node_id_ = entrance_node_id_;
      }
      if (last_visited_node_id_ < 0) {
        last_visited_node_id_ = entrance_node_id_;
      }

      if (last_visited_node_id_ == entrance_node_id_ && current_node_id_ < 0) {
        const double d = calculate_distance(
            graph_nodes_[entrance_node_id_].position, current_pose_.position);
        if (d >= nodes_distance_ &&
            !is_within_node_distance_of_any_node(current_pose_.position)) {
          geometry_msgs::msg::Point node_candidate = current_pose_.position;
          find_safer_node_position(current_pose_.position, node_candidate);

          const int new_node = create_checkpoint_node(node_candidate, false);
          if (new_node >= 0 && new_node != entrance_node_id_) {
            add_edge_between_nodes(entrance_node_id_, new_node);
            last_visited_node_id_ = new_node;
            current_node_id_ = new_node;
            previous_node_id_ = new_node;
          }
        }
      }
    }
    return;
  }

  const bool entered_node =
      (current_node_id_ >= 0 && current_node_id_ != previous_node_id_);
  const bool inside_any_node = (current_node_id_ >= 0);

  // Always run core graph processing while inside a node. Restrict
  // edge-creation and rule-L entry semantics to true node transitions.
  if (inside_any_node) {
    if (last_visited_node_id_ >= 0 &&
        last_visited_node_id_ != current_node_id_) {
      if (entered_node) {
        add_edge_between_nodes(last_visited_node_id_, current_node_id_);
      }
    }

    // Mandatory re-filter each time we reach a checkpoint node so stale
    // potentials too close to any node are removed immediately.
    prune_potentials_within_node_distance_recursive();
    simplify_checkpoint_graph();

    const auto remapped_containing =
        find_node_containing_position(current_pose_.position);
    current_node_id_ = remapped_containing.value_or(current_node_id_);
    if (last_visited_node_id_ >= 0 &&
        graph_nodes_.count(last_visited_node_id_) == 0) {
      last_visited_node_id_ = current_node_id_;
    }

    if (entered_node && last_visited_node_id_ == current_node_id_) {
      if (!suppress_rule_l_) {
        auto &node = graph_nodes_[current_node_id_];
        const size_t node_degree =
            node.edges.size() + (node.is_dead_end ? 1u : 0u);

        // Rule L revisit behavior:
        // Re-entering a single-edge node from outside while the last visited
        // checkpoint is that same node means we reached a potential dead end.
        // - If the node still has potential targets, keep explorer behavior
        //   unchanged (no forced potential/travel switch).
        // - If no potentials remain, classify as dead-end so its effective
        //   degree becomes 2 (incoming edge + dead-end self edge).
        if (current_node_id_ != entrance_node_id_ && node_degree == 1) {
          if (!node_has_resolvable_potential(current_node_id_)) {
            node.is_dead_end = true;
            if (potential_resolution_node_id_ == current_node_id_) {
              potential_resolution_node_id_ = -1;
            }
            RCLCPP_INFO(this->get_logger(),
                        "Node %d re-entered without potentials. Reclassifying "
                        "as dead-end.",
                        current_node_id_);
          } else {
            RCLCPP_DEBUG(this->get_logger(),
                         "Node %d re-entered with available potentials. "
                         "Keeping explorer behavior unchanged.",
                         current_node_id_);
          }
        }
      }
    } else if (entered_node) {
      suppress_rule_l_ = false;
    }

    if (entered_node && graph_nodes_.count(current_node_id_) > 0) {
      graph_nodes_[current_node_id_].visit_count += 1;
    }

    if (entered_node || last_visited_node_id_ < 0) {
      last_visited_node_id_ = current_node_id_;
    }
  }

  if (current_node_id_ < 0 && last_visited_node_id_ >= 0 &&
      graph_nodes_.count(last_visited_node_id_) > 0) {
    const double d = calculate_distance(
        graph_nodes_[last_visited_node_id_].position, current_pose_.position);
    if (d >= nodes_distance_ &&
        !is_within_node_distance_of_any_node(current_pose_.position)) {
      geometry_msgs::msg::Point node_candidate = current_pose_.position;
      find_safer_node_position(current_pose_.position, node_candidate);

      const int new_node = create_checkpoint_node(node_candidate, false);
      if (new_node < 0) {
        return;
      }
      add_edge_between_nodes(last_visited_node_id_, new_node);
      last_visited_node_id_ = new_node;
      current_node_id_ = new_node;
      previous_node_id_ = new_node;
    }
  }

  if (last_visited_node_id_ >= 0 &&
      graph_nodes_.count(last_visited_node_id_) > 0 &&
      latest_seen_point_valid_ &&
      (this->now() - latest_seen_point_stamp_).seconds() <=
          seen_point_timeout_s_) {
    register_potential_node_for_anchor(latest_seen_point_);
  }
}

bool MissionFsmNode::start_return_to_entrance_via_travel() {
  returning_to_entrance_via_travel_ = true;

  if (!macroplanning_enabled_) {
    return false;
  }

  if (entrance_node_id_ < 0 || graph_nodes_.count(entrance_node_id_) == 0) {
    entrance_node_id_ = create_checkpoint_node(cave_entrance_, true);
  }

  if (entrance_node_id_ < 0 || graph_nodes_.count(entrance_node_id_) == 0) {
    return false;
  }

  int start_node = -1;
  if (current_node_id_ >= 0 && graph_nodes_.count(current_node_id_) > 0) {
    start_node = current_node_id_;
  } else if (last_visited_node_id_ >= 0 &&
             graph_nodes_.count(last_visited_node_id_) > 0) {
    start_node = last_visited_node_id_;
  }

  if (start_node < 0) {
    if (is_inside_node(entrance_node_id_, current_pose_.position)) {
      travel_mode_ = false;
      travel_path_.clear();
      return true;
    }
    return false;
  }

  if (start_node == entrance_node_id_) {
    travel_mode_ = false;
    travel_path_.clear();
    return true;
  }

  auto path = compute_shortest_path_nodes(start_node, entrance_node_id_);
  if (path.empty()) {
    return false;
  }

  travel_mode_ = true;
  travel_path_.clear();
  for (size_t i = 1; i < path.size(); ++i) {
    travel_path_.push_back(path[i]);
  }

  if (!explorer_mode_suspended_for_travel_) {
    suspend_explorer_mode_for_travel();
  }

  RCLCPP_INFO(this->get_logger(),
              "Mission-complete return corridor activated: node %d -> entrance "
              "node %d (%zu checkpoints).",
              start_node, entrance_node_id_, travel_path_.size());
  return true;
}

void MissionFsmNode::update_mode_decision() {
  const bool was_travel_mode = travel_mode_;

  if (returning_to_entrance_via_travel_) {
    // Mission complete: keep the precomputed return corridor latched.
    return;
  }

  if (force_explorer_until_new_node_) {
    const bool reached_new_node =
        (current_node_id_ >= 0 && current_node_id_ != fallback_origin_node_id_);
    const bool fallback_timed_out =
        (force_explorer_timeout_s_ > 0.0) &&
        ((this->now() - force_explorer_start_time_).seconds() >
         force_explorer_timeout_s_);
    if (!reached_new_node && !fallback_timed_out) {
      travel_mode_ = false;
      potential_resolution_node_id_ = -1;
      travel_path_.clear();
      resume_explorer_mode_after_travel();
      return;
    }
    if (fallback_timed_out && !reached_new_node) {
      RCLCPP_WARN(this->get_logger(),
                  "Forced explorer fallback timed out after %.1f s. "
                  "Re-enabling macroplanning travel decisions.",
                  force_explorer_timeout_s_);
    }
    force_explorer_until_new_node_ = false;
    fallback_origin_node_id_ = -1;
  }

  if (last_visited_node_id_ < 0 ||
      graph_nodes_.count(last_visited_node_id_) == 0 ||
      graph_nodes_.size() < 2) {
    travel_mode_ = false;
    potential_resolution_node_id_ = -1;
    travel_path_.clear();
    if (!travel_mode_) {
      resume_explorer_mode_after_travel();
    }
    return;
  }

  std::vector<int> single_edge_nodes;
  for (const auto &entry : graph_nodes_) {
    if (entry.first == entrance_node_id_) {
      continue;
    }
    const auto degree =
        entry.second.edges.size() + (entry.second.is_dead_end ? 1 : 0);
    if (degree == 1) {
      single_edge_nodes.push_back(entry.first);
    }
  }

  if (single_edge_nodes.empty()) {
    clear_single_edge_priority_target();
    const int start_node =
        (current_node_id_ >= 0) ? current_node_id_ : last_visited_node_id_;
    if (start_node < 0 || graph_nodes_.count(start_node) == 0) {
      travel_mode_ = false;
      potential_resolution_node_id_ = -1;
      travel_path_.clear();
      resume_explorer_mode_after_travel();
      return;
    }

    auto compute_path_metric = [this](const std::vector<int> &path) {
      if (path.empty()) {
        return std::numeric_limits<double>::max();
      }
      double metric = 0.0;
      for (size_t i = 1; i < path.size(); ++i) {
        if (graph_nodes_.count(path[i - 1]) == 0 ||
            graph_nodes_.count(path[i]) == 0) {
          return std::numeric_limits<double>::max();
        }
        metric += calculate_distance(graph_nodes_.at(path[i - 1]).position,
                                     graph_nodes_.at(path[i]).position);
      }
      return metric;
    };

    // Keep an already-selected target latched while it's still valid.
    // This prevents oscillations where the "closest" candidate flips every
    // timer tick as the drone moves between checkpoints.
    int selected_potential_node = -1;
    std::vector<int> selected_path;

    if (potential_resolution_node_id_ >= 0 &&
        graph_nodes_.count(potential_resolution_node_id_) > 0 &&
        node_has_resolvable_potential(potential_resolution_node_id_)) {
      if (potential_resolution_node_id_ == start_node) {
        selected_potential_node = start_node;
        selected_path = {start_node};
      } else {
        auto candidate_path = compute_shortest_path_nodes(
            start_node, potential_resolution_node_id_);
        if (!candidate_path.empty()) {
          selected_potential_node = potential_resolution_node_id_;
          selected_path = std::move(candidate_path);
        }
      }
    }

    if (selected_potential_node < 0) {
      double best_metric = std::numeric_limits<double>::max();

      for (const auto &entry : graph_nodes_) {
        if (!node_has_resolvable_potential(entry.first)) {
          continue;
        }

        std::vector<int> candidate_path;
        if (entry.first == start_node) {
          candidate_path = {start_node};
        } else {
          candidate_path = compute_shortest_path_nodes(start_node, entry.first);
        }
        if (candidate_path.empty()) {
          continue;
        }

        const double metric = compute_path_metric(candidate_path);
        if (metric < best_metric) {
          best_metric = metric;
          selected_potential_node = entry.first;
          selected_path = std::move(candidate_path);
        }
      }
    }

    if (selected_potential_node < 0) {
      reset_graph_to_entrance();
      travel_mode_ = false;
      potential_resolution_node_id_ = -1;
      travel_path_.clear();
      if (!travel_mode_) {
        resume_explorer_mode_after_travel();
      }
      return;
    }

    potential_resolution_node_id_ = selected_potential_node;
    if (start_node != selected_potential_node) {
      if (selected_path.size() > 1) {
        travel_mode_ = true;
        std::deque<int> next_travel_path;
        for (size_t i = 1; i < selected_path.size(); ++i) {
          next_travel_path.push_back(selected_path[i]);
        }
        if (next_travel_path != travel_path_) {
          travel_path_ = std::move(next_travel_path);
        }
        if (!was_travel_mode) {
          suspend_explorer_mode_for_travel();
        }
        return;
      }
    }

    travel_mode_ = false;
    travel_path_.clear();
    resume_explorer_mode_after_travel();
    return;
  }

  potential_resolution_node_id_ = -1;
  const bool in_node_with_one_edge =
      (current_node_id_ >= 0 && current_node_id_ != entrance_node_id_ &&
       graph_nodes_.count(current_node_id_) > 0 &&
       (graph_nodes_[current_node_id_].edges.size() +
            (graph_nodes_[current_node_id_].is_dead_end ? 1 : 0) ==
        1));

  // Rule M (single-edge revisit -> potential resolution):
  // If we are currently in a single-edge node for at least the second visit,
  // force potential resolution from that node instead of directional
  // single-edge traversal.
  if (in_node_with_one_edge &&
      graph_nodes_[current_node_id_].visit_count >= 2 &&
      node_has_resolvable_potential(current_node_id_)) {
    if (goal_active_ && active_goal_source_ == GoalSource::TRAVEL) {
      cancel_pub_->publish(std_msgs::msg::Empty());
      goal_active_ = false;
    }
    potential_resolution_node_id_ = current_node_id_;
    single_edge_travel_target_node_id_ = -1;
    clear_single_edge_priority_target();
    travel_mode_ = false;
    travel_path_.clear();
    resume_explorer_mode_after_travel();
    return;
  }

  const bool outside_node = (current_node_id_ < 0);
  const bool last_has_one_edge =
      (last_visited_node_id_ != entrance_node_id_ &&
       (graph_nodes_[last_visited_node_id_].edges.size() +
            (graph_nodes_[last_visited_node_id_].is_dead_end ? 1 : 0) ==
        1));

  if (outside_node && last_has_one_edge && !was_travel_mode) {
    travel_mode_ = false;
    travel_path_.clear();
    resume_explorer_mode_after_travel();
    return;
  }

  const int direction_anchor_node_id =
      (in_node_with_one_edge) ? current_node_id_ : last_visited_node_id_;
  geometry_msgs::msg::Point preferred_direction;
  preferred_direction.x = current_pose_.position.x -
                          graph_nodes_[direction_anchor_node_id].position.x;
  preferred_direction.y = current_pose_.position.y -
                          graph_nodes_[direction_anchor_node_id].position.y;
  preferred_direction.z = current_pose_.position.z -
                          graph_nodes_[direction_anchor_node_id].position.z;

  const auto anchor_degree =
      graph_nodes_[direction_anchor_node_id].edges.size() +
      (graph_nodes_[direction_anchor_node_id].is_dead_end ? 1 : 0);
  if (anchor_degree == 1 &&
      !graph_nodes_[direction_anchor_node_id].edges.empty()) {
    const int predecessor_id =
        *graph_nodes_[direction_anchor_node_id].edges.begin();
    if (graph_nodes_.count(predecessor_id) > 0) {
      preferred_direction.x =
          graph_nodes_[direction_anchor_node_id].position.x -
          graph_nodes_[predecessor_id].position.x;
      preferred_direction.y =
          graph_nodes_[direction_anchor_node_id].position.y -
          graph_nodes_[predecessor_id].position.y;
      preferred_direction.z =
          graph_nodes_[direction_anchor_node_id].position.z -
          graph_nodes_[predecessor_id].position.z;
    }
  }

  const double preferred_norm =
      std::hypot(preferred_direction.x, preferred_direction.y);

  int target_leaf = -1;
  double best_alignment = -std::numeric_limits<double>::max();
  double best_distance = std::numeric_limits<double>::max();
  for (const int leaf : single_edge_nodes) {
    if (leaf == direction_anchor_node_id) {
      continue;
    }
    const auto &leaf_pos = graph_nodes_[leaf].position;
    const double d = calculate_distance(current_pose_.position, leaf_pos);

    double alignment = 0.0;
    if (preferred_norm > 1e-3) {
      const double vx =
          leaf_pos.x - graph_nodes_[direction_anchor_node_id].position.x;
      const double vy =
          leaf_pos.y - graph_nodes_[direction_anchor_node_id].position.y;
      const double vnorm = std::hypot(vx, vy);
      if (vnorm > 1e-3) {
        alignment = (vx * preferred_direction.x + vy * preferred_direction.y) /
                    (vnorm * preferred_norm);
      }
    }

    if (alignment > best_alignment + 1e-6 ||
        (std::abs(alignment - best_alignment) <= 1e-6 && d < best_distance)) {
      best_alignment = alignment;
      best_distance = d;
      target_leaf = leaf;
    }
  }

  const int start_node =
      (current_node_id_ >= 0) ? current_node_id_ : last_visited_node_id_;
  if (start_node < 0 || graph_nodes_.count(start_node) == 0 ||
      target_leaf < 0) {
    single_edge_travel_target_node_id_ = -1;
    clear_single_edge_priority_target();
    travel_mode_ = false;
    travel_path_.clear();
    resume_explorer_mode_after_travel();
    return;
  }

  // Defensive handoff guard:
  // If we're already at the predecessor of the selected single-edge leaf,
  // never keep/enter travel mode for that leaf. Immediately switch to
  // explorer-goal mode with priority reward toward the leaf.
  if (graph_nodes_.count(target_leaf) > 0 &&
      !graph_nodes_.at(target_leaf).edges.empty()) {
    const int predecessor_id = *graph_nodes_.at(target_leaf).edges.begin();
    if (graph_nodes_.count(predecessor_id) > 0) {
      const bool at_predecessor =
          (current_node_id_ == predecessor_id) ||
          (last_visited_node_id_ == predecessor_id) ||
          is_inside_node(predecessor_id, current_pose_.position);
      if (at_predecessor) {
        if (goal_active_ && active_goal_source_ == GoalSource::TRAVEL) {
          cancel_pub_->publish(std_msgs::msg::Empty());
          goal_active_ = false;
        }
        single_edge_travel_target_node_id_ = -1;
        set_single_edge_priority_target(target_leaf);
        travel_mode_ = false;
        travel_path_.clear();
        resume_explorer_mode_after_travel();
        return;
      }
    }
  }

  auto path = compute_shortest_path_nodes(start_node, target_leaf);
  if (path.empty()) {
    single_edge_travel_target_node_id_ = -1;
    clear_single_edge_priority_target();
    travel_mode_ = false;
    travel_path_.clear();
    resume_explorer_mode_after_travel();
    return;
  }

  if (path.size() <= 2) {
    if (goal_active_ && active_goal_source_ == GoalSource::TRAVEL) {
      cancel_pub_->publish(std_msgs::msg::Empty());
      goal_active_ = false;
    }
    single_edge_travel_target_node_id_ = -1;
    set_single_edge_priority_target(target_leaf);
    travel_mode_ = false;
    travel_path_.clear();
    resume_explorer_mode_after_travel();
    return;
  }

  // Keep priority reward disabled while we are still transiting through
  // intermediate checkpoints. It must be activated only once we reach the
  // predecessor node of the selected single-edge target.
  clear_single_edge_priority_target();
  single_edge_travel_target_node_id_ = target_leaf;

  travel_mode_ = true;
  travel_path_.clear();
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    travel_path_.push_back(path[i]);
  }
  if (!was_travel_mode) {
    suspend_explorer_mode_for_travel();
  }
}

void MissionFsmNode::set_single_edge_priority_target(int target_node_id) {
  if (target_node_id < 0 || graph_nodes_.count(target_node_id) == 0) {
    return;
  }
  if (single_edge_priority_target_node_id_ == target_node_id) {
    return;
  }

  single_edge_priority_target_node_id_ = target_node_id;
  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "world";
  msg.point = graph_nodes_.at(target_node_id).position;
  priority_target_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(),
              "Activated single-edge exploration priority toward node %d at "
              "[%.2f, %.2f, %.2f].",
              target_node_id, msg.point.x, msg.point.y, msg.point.z);
}

void MissionFsmNode::clear_single_edge_priority_target() {
  if (single_edge_priority_target_node_id_ < 0) {
    return;
  }

  single_edge_priority_target_node_id_ = -1;
  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "world";
  msg.point.x = std::numeric_limits<double>::quiet_NaN();
  msg.point.y = std::numeric_limits<double>::quiet_NaN();
  msg.point.z = std::numeric_limits<double>::quiet_NaN();
  priority_target_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(),
              "Cleared single-edge exploration priority target.");
}

void MissionFsmNode::set_single_edge_punishment_target(int target_node_id) {
  if (target_node_id < 0 || graph_nodes_.count(target_node_id) == 0) {
    return;
  }

  single_edge_punishment_target_node_id_ = target_node_id;
  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "world";
  msg.point = graph_nodes_.at(target_node_id).position;
  punishment_target_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(),
              "Activated single-edge return punishment toward predecessor node "
              "%d at [%.2f, %.2f, %.2f].",
              target_node_id, msg.point.x, msg.point.y, msg.point.z);
}

void MissionFsmNode::clear_single_edge_punishment_target() {
  if (single_edge_punishment_target_node_id_ < 0) {
    return;
  }

  single_edge_punishment_target_node_id_ = -1;
  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "world";
  msg.point.x = std::numeric_limits<double>::quiet_NaN();
  msg.point.y = std::numeric_limits<double>::quiet_NaN();
  msg.point.z = std::numeric_limits<double>::quiet_NaN();
  punishment_target_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(),
              "Cleared single-edge return punishment target.");
}

void MissionFsmNode::mark_potential_node_unreachable_near(
    const geometry_msgs::msg::Point &pos) {
  for (auto &entry : graph_nodes_) {
    auto &pots = entry.second.potentials;
    pots.erase(std::remove_if(pots.begin(), pots.end(),
                              [&](const PotentialNode &pot) {
                                return calculate_distance(pot.position, pos) <=
                                       node_radius_;
                              }),
               pots.end());
  }
}

} // namespace control
