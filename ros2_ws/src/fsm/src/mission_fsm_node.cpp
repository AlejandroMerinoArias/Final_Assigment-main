#include "fsm/mission_fsm_node.hpp"

#include <limits>
#include <queue>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace control {

MissionFsmNode::MissionFsmNode()
    : Node("mission_fsm_node"), current_state_(MissionState::INIT),
      pose_received_(false), mission_start_signal_received_(false),
      goal_active_(false), goal_request_pending_(false),
      lanterns_found_count_(0), lantern_dedup_threshold_(2.5),
      takeoff_altitude_(2.0), z_retry_index_(0),
      goal_set_time_(this->now()), min_exploration_goal_distance_(2.0),
      consecutive_too_close_rejections_(0),
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
  lantern_dedup_threshold_ = this->get_parameter("lantern_dedup_threshold").as_double();
  this->declare_parameter("min_exploration_goal_distance", min_exploration_goal_distance_);
  min_exploration_goal_distance_ = this->get_parameter("min_exploration_goal_distance").as_double();
  this->declare_parameter("explore_goal_selection_timeout", explore_goal_selection_timeout_);
  explore_goal_selection_timeout_ =
      this->get_parameter("explore_goal_selection_timeout").as_double();
  this->declare_parameter("explore_goal_selection_max_failures", explore_goal_selection_max_failures_);
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
  macroplanning_enabled_ = this->get_parameter("macroplanning_enabled").as_bool();
  this->declare_parameter("nodes_distance", nodes_distance_);
  nodes_distance_ = this->get_parameter("nodes_distance").as_double();
  this->declare_parameter("node_radius", node_radius_);
  node_radius_ = this->get_parameter("node_radius").as_double();
  this->declare_parameter("max_potential_node_range", max_potential_node_range_);
  max_potential_node_range_ = this->get_parameter("max_potential_node_range").as_double();
  this->declare_parameter("seen_point_timeout_s", seen_point_timeout_s_);
  seen_point_timeout_s_ = this->get_parameter("seen_point_timeout_s").as_double();

  // Cave entrance - Main entrance
  cave_entrance_.x = -320.0;
  cave_entrance_.y = 10.0;
  cave_entrance_.z = 18.0;

  // Z-retry altitudes: start with default, then try lower, then higher
  // z_retry_altitudes_ is now built dynamically per goal in request_exploration_goal()
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
      std::bind(&MissionFsmNode::depth_points_callback, this, std::placeholders::_1));

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
  checkpoint_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/fsm/checkpoint_markers", 10);

  enable_mapping_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/enable_mapping", rclcpp::QoS(1).transient_local().reliable());

  blacklist_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/exploration/blacklist_goal", 10);

  // --- Timer: 10 Hz main loop ---
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&MissionFsmNode::timer_callback, this));

  // --- TF Listener ---
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "Mission FSM Node initialized. State: INIT");
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
    lantern_world = tf_buffer_->transform(*msg, "world", tf2::durationFromSec(1.0));
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
                "Checking lantern candidate [%.2f, %.2f, %.2f] against existing [%.2f, %.2f, %.2f]: dist=%.2f",
                lantern_world.pose.position.x, lantern_world.pose.position.y, lantern_world.pose.position.z,
                existing_pose.position.x, existing_pose.position.y, existing_pose.position.z,
                dist);

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
  publish_state();
  publish_drone_marker();
  publish_checkpoint_markers();
}

void MissionFsmNode::planner_status_callback(
    const std_msgs::msg::String::SharedPtr msg) {
  if (msg->data == "GOAL_REACHED") {
    RCLCPP_INFO(this->get_logger(), "Planner: Goal reached!");
    goal_active_ = false;
    // State machine will handle transition in update_state()
  } else if (msg->data == "PLAN_FAILED") {
    RCLCPP_WARN(this->get_logger(),
                "Planner: Planning failed for current goal.");
    goal_active_ = false;

    // If we're exploring, trigger the Z-retry recovery
    if (current_state_ == MissionState::EXPLORE ||
        current_state_ == MissionState::REFINE_GOAL) {
      transition_to(MissionState::REFINE_GOAL);
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
  double goal_dist = std::sqrt(
      (planner_goal.pose.position.x - current_pose_.position.x) *
      (planner_goal.pose.position.x - current_pose_.position.x) +
      (planner_goal.pose.position.y - current_pose_.position.y) *
      (planner_goal.pose.position.y - current_pose_.position.y));
  
  if (goal_dist < min_exploration_goal_distance_) {
    ++consecutive_too_close_rejections_;
    
    // If we've rejected 3 consecutive goals for being too close, accept this one anyway
    // to prevent infinite loop
    if (consecutive_too_close_rejections_ >= MAX_CONSECUTIVE_TOO_CLOSE_REJECTIONS) {
      RCLCPP_WARN(this->get_logger(),
                  "Exploration goal too close (%.2f m < %.2f m), but accepting anyway "
                  "after %d consecutive rejections to avoid getting stuck.",
                  goal_dist, min_exploration_goal_distance_,
                  consecutive_too_close_rejections_);
      consecutive_too_close_rejections_ = 0;  // Reset counter
      // Continue to accept the goal below
    } else {
      RCLCPP_WARN(this->get_logger(),
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

  RCLCPP_INFO(this->get_logger(),
              "Sent exploration goal to planner: [%.2f, %.2f, %.2f] "
              "(distance=%.2f m)",
              planner_goal.pose.position.x, planner_goal.pose.position.y,
              planner_goal.pose.position.z, goal_dist);
}


void MissionFsmNode::depth_points_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!macroplanning_enabled_ || !pose_received_ || msg->width == 0 || msg->data.empty()) {
    return;
  }

  geometry_msgs::msg::Point best_point;
  bool found = false;
  double best_dist = 0.0;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

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

    if (!found || d > best_dist) {
      best_dist = d;
      best_point = candidate;
      found = true;
    }
  }

  if (found) {
    latest_seen_point_ = best_point;
    latest_seen_point_valid_ = true;
    latest_seen_point_stamp_ = this->now();
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
          current_pose_.position.z + takeoff_altitude_; // Ascend takeoff_altitude_ m relative to start

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

  case MissionState::EXPLORE:
    {
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
      goal_request_pending_ = false;
      // Reset goal-selection tracking when (re)entering exploration
      consecutive_goal_request_failures_ = 0;
      last_successful_exploration_goal_time_ = this->now();
      if (macroplanning_enabled_ && entrance_node_id_ < 0) {
        entrance_node_id_ = create_checkpoint_node(cave_entrance_, true);
        last_visited_node_id_ = entrance_node_id_;
      }
      // Main loop (update_state) will request the goal automatically
    }
    break;

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
        return_goal_msg.pose.position.z = takeoff_altitude_; // Maintain altitude for return
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
    publish_trajectory_goal(current_pose_.position.x, current_pose_.position.y,
                            0.0);
    goal_active_ = true;
    break;

  default:
    break;
  }
}

void MissionFsmNode::on_state_exit(MissionState state) {
  (void)state; // Currently no exit actions needed
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
    if (macroplanning_enabled_) {
      update_checkpoint_graph();
      update_mode_decision();
    }

    // Check if we've found all lanterns
    if (lanterns_found_count_ >= TARGET_LANTERN_COUNT) {
      RCLCPP_INFO(this->get_logger(),
                  "All %zu lanterns found! Mission complete.",
                  TARGET_LANTERN_COUNT);
      transition_to(MissionState::RETURN);
      break;
    }

    // Check if we've reached the current exploration goal
    if (goal_active_) {
      double dist = calculate_distance(current_pose_.position, current_goal_);
      double time_since_goal_set = (this->now() - goal_set_time_).seconds();
      
      // FIRST: Check if goal reached (most important check - do this first!)
      if (dist < GOAL_REACHED_THRESHOLD) {
        RCLCPP_INFO(this->get_logger(),
                    "Reached exploration goal [%.2f, %.2f, %.2f] "
                    "(distance=%.2f m, took %.1f s). Requesting next goal.",
                    current_goal_.x, current_goal_.y, current_goal_.z,
                    dist, time_since_goal_set);
        goal_active_ = false;
        consecutive_too_close_rejections_ = 0;  // Reset counter on successful goal completion
        // Loop will trigger new request in next block
      }
      // SECOND: Check for goal timeout - if goal takes too long, abandon it
      else if (time_since_goal_set > GOAL_TIMEOUT_SECONDS) {
        RCLCPP_WARN(this->get_logger(),
                    "Exploration goal timeout (%.1f s > %.1f s). "
                    "Switching to recovery for goal [%.2f, %.2f, %.2f] "
                    "(current distance=%.2f m, threshold=%.2f m).",
                    time_since_goal_set, GOAL_TIMEOUT_SECONDS,
                    current_goal_.x, current_goal_.y, current_goal_.z,
                    dist, GOAL_REACHED_THRESHOLD);
        start_refine_for_current_goal("goal timeout");
        break;
      }
      // THIRD: Check if drone is stuck (not making progress toward goal)
      else if (time_since_goal_set > STUCK_DETECTION_SECONDS) {  // After 10 seconds, check movement
        double movement_since_goal = calculate_distance(
            current_pose_.position, last_pose_at_goal_set_);
        double progress_toward_goal = calculate_distance(
            last_pose_at_goal_set_, current_goal_) -
            dist;  // How much closer we got
        
        if (movement_since_goal < MIN_MOVEMENT_THRESHOLD) {
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
          // Making movement but not toward goal (might be going around obstacle)
          // Only warn if we're still far from goal
          RCLCPP_DEBUG(this->get_logger(),
                       "Drone moving (%.2f m) but slow progress toward goal "
                       "(%.2f m closer, %.2f m remaining).",
                       movement_since_goal, progress_toward_goal, dist);
        }
      }
    }

    // Detect goal-selection stuck conditions (no active goal and repeated failures)
    if (!goal_active_) {
      const double time_since_last_success =
          (this->now() - last_successful_exploration_goal_time_).seconds();
      if ((explore_goal_selection_timeout_ > 0.0 &&
           time_since_last_success > explore_goal_selection_timeout_) ||
          (explore_goal_selection_max_failures_ > 0 &&
           consecutive_goal_request_failures_ >= explore_goal_selection_max_failures_)) {
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
      if (macroplanning_enabled_ && travel_mode_ && !travel_path_.empty()) {
        const int next_node_id = travel_path_.front();
        travel_path_.pop_front();
        if (graph_nodes_.count(next_node_id) > 0) {
          RCLCPP_INFO(this->get_logger(),
                      "Travel mode: traversing to checkpoint node %d.",
                      next_node_id);
          try_activate_exploration_goal(graph_nodes_.at(next_node_id).position);
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
    if (goal_active_) {
      double time_since_replan = (this->now() - last_replan_time_).seconds();
      if (time_since_replan >= replan_interval_s_) {
        replan_current_goal();
      }
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
          RCLCPP_WARN(this->get_logger(),
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
        RCLCPP_INFO(this->get_logger(), "Returned to start position (dist=%.2f)! Landing.", dist);
        transition_to(MissionState::LAND);
      } else {
         RCLCPP_WARN(this->get_logger(), "Planner finished but drone is %.2f m from start. Landing anyway (emergency).", dist);
         transition_to(MissionState::LAND);
      }
    }
    // Also monitor timeout for return
    else if ((this->now() - goal_set_time_).seconds() > 120.0) { // 2 minutes max for return
        RCLCPP_WARN(this->get_logger(), "Return home timed out!");
        goal_active_ = false; 
        transition_to(MissionState::LAND); // Land where we are? Or try again? Landing is safer.
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
               "Replanned path to [%.2f, %.2f, %.2f] via %s",
               current_goal_.x, current_goal_.y, current_goal_.z,
               planner_type_.c_str());
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

  std::set<std::pair<int, int>> drawn_edges;
  for (const auto &entry : graph_nodes_) {
    const auto &node = entry.second;
    const size_t degree = node.edges.size() + (node.is_dead_end ? 1u : 0u);
    if (degree <= 1) {
      single_edge_nodes_marker.points.push_back(node.position);
    } else {
      multi_edge_nodes_marker.points.push_back(node.position);
    }

    if (node.potential.valid && !node.potential.unreachable) {
      potential_marker.points.push_back(node.potential.position);
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

  RCLCPP_WARN(this->get_logger(),
              "Triggering REFINE_GOAL for [%.2f, %.2f, %.2f] (%s).",
              strategic_goal_.x, strategic_goal_.y, strategic_goal_.z,
              reason.c_str());
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

bool MissionFsmNode::try_activate_exploration_goal(const geometry_msgs::msg::Point &goal) {
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

  double goal_dist = std::hypot(planner_goal.pose.position.x - current_pose_.position.x,
                                planner_goal.pose.position.y - current_pose_.position.y);

  if (goal_dist < min_exploration_goal_distance_) {
    ++consecutive_too_close_rejections_;
    if (consecutive_too_close_rejections_ < MAX_CONSECUTIVE_TOO_CLOSE_REJECTIONS) {
      RCLCPP_WARN(this->get_logger(),
                  "Exploration goal too close (%.2f m < %.2f m). "
                  "Rejecting (consecutive rejections: %d/%d).",
                  goal_dist, min_exploration_goal_distance_,
                  consecutive_too_close_rejections_,
                  MAX_CONSECUTIVE_TOO_CLOSE_REJECTIONS);
      return false;
    }

    RCLCPP_WARN(this->get_logger(),
                "Exploration goal too close (%.2f m < %.2f m), but accepting anyway "
                "after %d consecutive rejections to avoid deadlock.",
                goal_dist, min_exploration_goal_distance_,
                consecutive_too_close_rejections_);
    consecutive_too_close_rejections_ = 0;
  } else {
    consecutive_too_close_rejections_ = 0;
  }

  current_goal_ = planner_goal.pose.position;
  goal_set_time_ = this->now();
  last_pose_at_goal_set_ = current_pose_.position;

  if (travel_mode_) {
    planner_goal_pub_->publish(planner_goal);
  } else if (planner_type_ == "A_star") {
    planner_goal_pub_a_->publish(planner_goal);
  } else {
    planner_goal_pub_->publish(planner_goal);
  }
  goal_active_ = true;

  RCLCPP_INFO(
      this->get_logger(),
      "Navigating to exploration goal via planner: [%.2f, %.2f, %.2f] (distance=%.2f m)",
      planner_goal.pose.position.x, planner_goal.pose.position.y,
      planner_goal.pose.position.z, goal_dist);

  return true;
}

void MissionFsmNode::request_exploration_goal() {
  // Check if service is ready (non-blocking)
  if (!exploration_goal_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Exploration service not available yet... Retrying soon.");
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "Requesting exploration goal from service...");

  auto request =
      std::make_shared<exploring::srv::GetExplorationGoal::Request>();

  // Set flag to prevent duplicate requests
  goal_request_pending_ = true;

  // Async call with callback
  exploration_goal_client_->async_send_request(
      request,
      [this](rclcpp::Client<exploring::srv::GetExplorationGoal>::SharedFuture
                 future) {
        // Clear flag when response received
        goal_request_pending_ = false;

        auto response = future.get();

        if (!response->success) {
          RCLCPP_WARN(this->get_logger(), "Exploration goal request failed: %s",
                      response->message.c_str());
          // Track consecutive failures so EXPLORE can detect goal-selection issues
          ++consecutive_goal_request_failures_;
          return;
        }

        // Check if still in EXPLORE state (might have transitioned)
        if (current_state_ != MissionState::EXPLORE) {
          RCLCPP_DEBUG(
              this->get_logger(),
              "Ignoring exploration goal - no longer in EXPLORE state.");
          return;
        }

        // Store the strategic goal
        strategic_goal_.x = response->goal.pose.position.x;
        strategic_goal_.y = response->goal.pose.position.y;
        strategic_goal_.z = response->goal.pose.position.z;

        RCLCPP_INFO(this->get_logger(),
                    "Received exploration goal: [%.2f, %.2f, %.2f]",
                    strategic_goal_.x, strategic_goal_.y, strategic_goal_.z);

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


bool MissionFsmNode::is_inside_node(int node_id, const geometry_msgs::msg::Point &pos) const {
  const auto it = graph_nodes_.find(node_id);
  if (it == graph_nodes_.end()) {
    return false;
  }
  return calculate_distance(it->second.position, pos) <= node_radius_;
}

std::optional<int> MissionFsmNode::find_node_containing_position(const geometry_msgs::msg::Point &pos) const {
  int closest_id = -1;
  double closest_dist = std::numeric_limits<double>::max();
  for (const auto &entry : graph_nodes_) {
    const double d = calculate_distance(entry.second.position, pos);
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

int MissionFsmNode::create_checkpoint_node(const geometry_msgs::msg::Point &pos, bool is_entrance) {
  const int node_id = next_node_id_++;
  CheckpointNode node;
  node.id = node_id;
  node.position = pos;
  node.is_dead_end = is_entrance;
  graph_nodes_[node_id] = node;
  prune_potential_nodes_near(pos, nodes_distance_);
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

void MissionFsmNode::prune_potential_nodes_near(const geometry_msgs::msg::Point &pos, double radius) {
  for (auto &entry : graph_nodes_) {
    auto &pot = entry.second.potential;
    if (pot.valid && calculate_distance(pos, pot.position) <= radius) {
      pot = PotentialNode{};
    }
  }
}

void MissionFsmNode::register_potential_node_for_anchor(const geometry_msgs::msg::Point &candidate) {
  if (last_visited_node_id_ < 0 || graph_nodes_.count(last_visited_node_id_) == 0) {
    return;
  }

  auto &anchor = graph_nodes_[last_visited_node_id_];
  if (calculate_distance(anchor.position, candidate) < nodes_distance_) {
    return;
  }

  for (const int neigh : anchor.edges) {
    if (graph_nodes_.count(neigh) > 0 &&
        calculate_distance(graph_nodes_[neigh].position, candidate) < nodes_distance_) {
      return;
    }
  }

  anchor.potential.position = candidate;
  anchor.potential.valid = true;
  anchor.potential.unreachable = false;
}

void MissionFsmNode::promote_potential_node(int anchor_node_id) {
  if (graph_nodes_.count(anchor_node_id) == 0) {
    return;
  }
  auto &anchor = graph_nodes_[anchor_node_id];
  if (!anchor.potential.valid || anchor.potential.unreachable) {
    return;
  }

  const geometry_msgs::msg::Point new_pos = anchor.potential.position;
  anchor.potential = PotentialNode{};

  int maybe_existing = -1;
  for (const auto &entry : graph_nodes_) {
    if (calculate_distance(entry.second.position, new_pos) <= node_radius_) {
      maybe_existing = entry.first;
      break;
    }
  }

  const int new_node_id = (maybe_existing >= 0) ? maybe_existing : create_checkpoint_node(new_pos, false);
  add_edge_between_nodes(anchor_node_id, new_node_id);
}

bool MissionFsmNode::promote_closest_potential_node() {
  if (last_visited_node_id_ < 0 || graph_nodes_.count(last_visited_node_id_) == 0) {
    return false;
  }

  int best_anchor = -1;
  double best_dist = std::numeric_limits<double>::max();
  for (const auto &entry : graph_nodes_) {
    if (!entry.second.potential.valid || entry.second.potential.unreachable) {
      continue;
    }
    const double d = calculate_distance(current_pose_.position, entry.second.potential.position);
    if (d < best_dist) {
      best_dist = d;
      best_anchor = entry.first;
    }
  }

  if (best_anchor < 0) {
    return false;
  }
  promote_potential_node(best_anchor);
  return true;
}

std::vector<int> MissionFsmNode::compute_shortest_path_nodes(int start_node, int goal_node) const {
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
  entrance.potential = PotentialNode{};
  entrance.is_dead_end = true;

  graph_nodes_.clear();
  graph_nodes_[entrance_node_id_] = entrance;
  last_visited_node_id_ = entrance_node_id_;
  current_node_id_ = entrance_node_id_;
  previous_node_id_ = entrance_node_id_;
  travel_mode_ = false;
  travel_path_.clear();
}

void MissionFsmNode::update_checkpoint_graph() {
  if (graph_nodes_.empty()) {
    return;
  }

  previous_node_id_ = current_node_id_;
  const auto containing = find_node_containing_position(current_pose_.position);
  current_node_id_ = containing.value_or(-1);

  const bool entered_node = (current_node_id_ >= 0 && current_node_id_ != previous_node_id_);

  if (entered_node) {
    if (last_visited_node_id_ >= 0 && last_visited_node_id_ != current_node_id_) {
      add_edge_between_nodes(last_visited_node_id_, current_node_id_);
    }

    if (last_visited_node_id_ == current_node_id_) {
      auto &node = graph_nodes_[current_node_id_];
      if (node.potential.valid && !node.potential.unreachable) {
        promote_potential_node(current_node_id_);
      } else if (node.edges.size() <= 1 && !node.is_dead_end) {
        // Backtracking into the same node with no potential means a dead-end.
        node.is_dead_end = true;
      }
    }

    last_visited_node_id_ = current_node_id_;
  }

  if (current_node_id_ < 0 && last_visited_node_id_ >= 0 && graph_nodes_.count(last_visited_node_id_) > 0) {
    const double d = calculate_distance(graph_nodes_[last_visited_node_id_].position,
                                        current_pose_.position);
    if (d >= nodes_distance_) {
      const int new_node = create_checkpoint_node(current_pose_.position, false);
      add_edge_between_nodes(last_visited_node_id_, new_node);
      last_visited_node_id_ = new_node;
      current_node_id_ = new_node;
      previous_node_id_ = new_node;
    }
  }

  if (last_visited_node_id_ >= 0 && graph_nodes_.count(last_visited_node_id_) > 0 &&
      latest_seen_point_valid_ &&
      (this->now() - latest_seen_point_stamp_).seconds() <= seen_point_timeout_s_) {
    register_potential_node_for_anchor(latest_seen_point_);
  }
}

void MissionFsmNode::update_mode_decision() {
  if (last_visited_node_id_ < 0 || graph_nodes_.count(last_visited_node_id_) == 0) {
    travel_mode_ = false;
    return;
  }

  std::vector<int> single_edge_nodes;
  for (const auto &entry : graph_nodes_) {
    const auto degree = entry.second.edges.size() + (entry.second.is_dead_end ? 1 : 0);
    if (degree == 1) {
      single_edge_nodes.push_back(entry.first);
    }
  }

  if (single_edge_nodes.empty()) {
    if (!promote_closest_potential_node()) {
      bool has_any_potential = false;
      for (const auto &entry : graph_nodes_) {
        has_any_potential = has_any_potential || entry.second.potential.valid;
      }
      if (!has_any_potential) {
        reset_graph_to_entrance();
      }
    }
    travel_mode_ = false;
    travel_path_.clear();
    return;
  }

  const bool in_node_with_one_edge = (current_node_id_ >= 0 && graph_nodes_.count(current_node_id_) > 0 &&
                                      (graph_nodes_[current_node_id_].edges.size() + (graph_nodes_[current_node_id_].is_dead_end ? 1 : 0) == 1));
  const bool outside_node = (current_node_id_ < 0);
  const bool last_has_one_edge = (graph_nodes_[last_visited_node_id_].edges.size() + (graph_nodes_[last_visited_node_id_].is_dead_end ? 1 : 0) == 1);

  if (in_node_with_one_edge || (outside_node && last_has_one_edge)) {
    travel_mode_ = false;
    travel_path_.clear();
    return;
  }

  int target_leaf = -1;
  double best = std::numeric_limits<double>::max();
  for (const int leaf : single_edge_nodes) {
    double d = calculate_distance(current_pose_.position, graph_nodes_[leaf].position);
    if (d < best) {
      best = d;
      target_leaf = leaf;
    }
  }

  const bool current_has_multi = current_node_id_ >= 0 && graph_nodes_[current_node_id_].edges.size() > 1;
  const bool last_has_multi = graph_nodes_[last_visited_node_id_].edges.size() > 1;
  if (!(current_has_multi && last_has_multi) || target_leaf < 0) {
    travel_mode_ = false;
    travel_path_.clear();
    return;
  }

  auto path = compute_shortest_path_nodes(current_node_id_, target_leaf);
  if (path.size() <= 1) {
    travel_mode_ = false;
    travel_path_.clear();
    return;
  }

  travel_mode_ = true;
  travel_path_.clear();
  for (size_t i = 1; i < path.size(); ++i) {
    travel_path_.push_back(path[i]);
  }
}

void MissionFsmNode::mark_potential_node_unreachable_near(const geometry_msgs::msg::Point &pos) {
  for (auto &entry : graph_nodes_) {
    auto &pot = entry.second.potential;
    if (pot.valid && calculate_distance(pot.position, pos) <= node_radius_) {
      pot = PotentialNode{};
    }
  }
}

} // namespace control
