#include "FSM/mission_fsm_node.hpp"

namespace control
{

MissionFsmNode::MissionFsmNode()
: Node("mission_fsm_node"),
  current_state_(MissionState::INIT),
  pose_received_(false),
  mission_start_signal_received_(false),
  goal_active_(false),
  lanterns_found_count_(0),
  takeoff_altitude_(2.0),
  z_retry_index_(0)
{
  // Initialize positions
  start_position_.x = 0.0;
  start_position_.y = 0.0;
  start_position_.z = 0.0;
  
  // Cave entrance - TODO: Set actual coordinates
  cave_entrance_.x = 10.0;
  cave_entrance_.y = 0.0;
  cave_entrance_.z = 2.0;

  // Z-retry altitudes: start with default, then try lower, then higher
  z_retry_altitudes_ = {1.5, 1.0, 2.0, 0.75, 2.5};

  // --- Subscribers ---
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/current_state_est", 10,
    std::bind(&MissionFsmNode::odometry_callback, this, std::placeholders::_1));

  lantern_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/detected_lanterns", 10,
    std::bind(&MissionFsmNode::lantern_callback, this, std::placeholders::_1));

  planner_status_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/planner/status", 10,
    std::bind(&MissionFsmNode::planner_status_callback, this, std::placeholders::_1));

  exploration_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/exploration/goal", 10,
    std::bind(&MissionFsmNode::exploration_goal_callback, this, std::placeholders::_1));

  start_mission_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/mission/start", 10,
    std::bind(&MissionFsmNode::start_mission_callback, this, std::placeholders::_1));

  // --- Publishers ---
  trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
    "/command/trajectory", 10);

  cancel_pub_ = this->create_publisher<std_msgs::msg::Empty>(
    "/fsm/cancel", 10);

  state_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/fsm/state", 10);

  planner_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/planner/goal", 10);

  // --- Timer: 10 Hz main loop ---
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MissionFsmNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Mission FSM Node initialized. State: INIT");
  publish_state();
}

void MissionFsmNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
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

void MissionFsmNode::lantern_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Only process lanterns during EXPLORE state
  if (current_state_ != MissionState::EXPLORE) {
    return;
  }

  // De-duplication: Is this a new lantern?
  bool is_new = true;
  for (const auto& existing_pose : detected_lantern_poses_) {
    double dist = calculate_distance(msg->pose.position, existing_pose.position);
    if (dist < LANTERN_DEDUP_THRESHOLD) {
      is_new = false;
      break;
    }
  }

  if (is_new) {
    RCLCPP_INFO(this->get_logger(), 
                "New lantern detected at [%.2f, %.2f, %.2f]! Total found: %zu",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                lanterns_found_count_ + 1);
    
    detected_lantern_poses_.push_back(msg->pose);
    lanterns_found_count_++;

    // Interrupt current movement
    cancel_pub_->publish(std_msgs::msg::Empty());
    goal_active_ = false;

    // Transition to LANTERN_FOUND state
    transition_to(MissionState::LANTERN_FOUND);
  }
}

void MissionFsmNode::timer_callback()
{
  update_state();
  publish_state();
}

void MissionFsmNode::planner_status_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "GOAL_REACHED") {
    RCLCPP_INFO(this->get_logger(), "Planner: Goal reached!");
    goal_active_ = false;
    // State machine will handle transition in update_state()
  }
  else if (msg->data == "PLAN_FAILED") {
    RCLCPP_WARN(this->get_logger(), "Planner: Planning failed for current goal.");
    goal_active_ = false;
    
    // If we're exploring, trigger the Z-retry recovery
    if (current_state_ == MissionState::EXPLORE || 
        current_state_ == MissionState::REFINE_GOAL) {
      transition_to(MissionState::REFINE_GOAL);
    }
  }
}

void MissionFsmNode::exploration_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Only accept new exploration goals when in EXPLORE state and not busy
  if (current_state_ != MissionState::EXPLORE) {
    RCLCPP_DEBUG(this->get_logger(), "Ignoring exploration goal - not in EXPLORE state.");
    return;
  }

  if (goal_active_) {
    RCLCPP_DEBUG(this->get_logger(), "Ignoring exploration goal - already have an active goal.");
    return;
  }

  // Check if this goal is blacklisted
  if (is_goal_blacklisted(msg->pose.position)) {
    RCLCPP_WARN(this->get_logger(), 
                "Received blacklisted goal [%.2f, %.2f]. Requesting new goal.",
                msg->pose.position.x, msg->pose.position.y);
    return;
  }

  // Store the strategic (x,y) goal
  strategic_goal_.x = msg->pose.position.x;
  strategic_goal_.y = msg->pose.position.y;
  strategic_goal_.z = msg->pose.position.z;  // Initial Z from exploration manager

  RCLCPP_INFO(this->get_logger(), 
              "New exploration goal received: [%.2f, %.2f, %.2f]",
              strategic_goal_.x, strategic_goal_.y, strategic_goal_.z);

  // Reset Z-retry index and start with the first altitude
  z_retry_index_ = 0;

  // Publish goal to planner with first Z from retry list
  double altitude = z_retry_altitudes_[z_retry_index_];
  geometry_msgs::msg::PoseStamped goal_msg;
  goal_msg.header.stamp = this->now();
  goal_msg.header.frame_id = "world";
  goal_msg.pose.position.x = strategic_goal_.x;
  goal_msg.pose.position.y = strategic_goal_.y;
  goal_msg.pose.position.z = altitude;
  goal_msg.pose.orientation.w = 1.0;

  planner_goal_pub_->publish(goal_msg);
  goal_active_ = true;
  z_retry_index_++;

  RCLCPP_INFO(this->get_logger(), 
              "Sent goal to planner: [%.2f, %.2f, %.2f]",
              goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z);
}

void MissionFsmNode::start_mission_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  if (!mission_start_signal_received_) {
    mission_start_signal_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Mission Start Signal Received!");
  }
}


void MissionFsmNode::transition_to(MissionState new_state)
{
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

void MissionFsmNode::on_state_enter(MissionState state)
{
  switch (state) {
    case MissionState::TAKEOFF:
      RCLCPP_INFO(this->get_logger(), "Taking off to altitude: %.2f m", takeoff_altitude_);
      publish_trajectory_goal(
        current_pose_.position.x,
        current_pose_.position.y,
        takeoff_altitude_);
      goal_active_ = true;
      break;

    case MissionState::GOTO_ENTRANCE:
      RCLCPP_INFO(this->get_logger(), "Navigating to cave entrance: [%.2f, %.2f, %.2f]",
                  cave_entrance_.x, cave_entrance_.y, cave_entrance_.z);
      publish_trajectory_goal(cave_entrance_.x, cave_entrance_.y, cave_entrance_.z);
      goal_active_ = true;
      break;

    case MissionState::EXPLORE:
      RCLCPP_INFO(this->get_logger(), "Entering exploration mode. Lanterns found: %zu/%zu",
                  lanterns_found_count_, TARGET_LANTERN_COUNT);
      // Reset Z-retry for new exploration goal
      z_retry_index_ = 0;
      goal_active_ = false;
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
      RCLCPP_INFO(this->get_logger(), "Returning to start position: [%.2f, %.2f, %.2f]",
                  start_position_.x, start_position_.y, takeoff_altitude_);
      publish_trajectory_goal(start_position_.x, start_position_.y, takeoff_altitude_);
      goal_active_ = true;
      break;

    case MissionState::LAND:
      RCLCPP_INFO(this->get_logger(), "Landing...");
      publish_trajectory_goal(
        current_pose_.position.x,
        current_pose_.position.y,
        0.0);
      goal_active_ = true;
      break;

    default:
      break;
  }
}

void MissionFsmNode::on_state_exit(MissionState state)
{
  (void)state;  // Currently no exit actions needed
}

void MissionFsmNode::update_state()
{
  switch (current_state_) {
    case MissionState::INIT:
      // Wait for pose data and start signal
      if (pose_received_ && mission_start_signal_received_) {
        transition_to(MissionState::TAKEOFF);
      } else {
        // Logging for user feedback
        if (!pose_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                 "Waiting for initial pose...");
        } else if (!mission_start_signal_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                 "Waiting for start signal (publish empty message to /mission/start)...");
        }
      }
      break;

    case MissionState::TAKEOFF:
      // Check if takeoff altitude reached
      if (goal_active_) {
        double alt_diff = std::abs(current_pose_.position.z - takeoff_altitude_);
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
      // Check if we've found all lanterns or exploration is complete
      if (lanterns_found_count_ >= TARGET_LANTERN_COUNT) {
        RCLCPP_INFO(this->get_logger(), "All %zu lanterns found! Mission complete.",
                    TARGET_LANTERN_COUNT);
        transition_to(MissionState::RETURN);
      }
      // Otherwise, exploration_manager will provide goals via action/topic
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
                      "Z-Retry: Attempting altitude %.2f m (attempt %zu/%zu) for goal [%.2f, %.2f]",
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
          
          planner_goal_pub_->publish(goal_msg);
          goal_active_ = true;
          z_retry_index_++;
        } else {
          // All Z retries exhausted - blacklist this goal
          RCLCPP_WARN(this->get_logger(), 
                      "All Z retries failed for goal [%.2f, %.2f]. Blacklisting.",
                      strategic_goal_.x, strategic_goal_.y);
          blacklisted_goals_.push_back(strategic_goal_);
          
          // Reset and return to exploration for a new goal
          z_retry_index_ = 0;
          goal_active_ = false;
          transition_to(MissionState::EXPLORE);
        }
      }
      break;

    case MissionState::RETURN:
      // Check if returned to start
      if (goal_active_) {
        geometry_msgs::msg::Point return_goal;
        return_goal.x = start_position_.x;
        return_goal.y = start_position_.y;
        return_goal.z = takeoff_altitude_;
        double dist = calculate_distance(current_pose_.position, return_goal);
        if (dist < GOAL_REACHED_THRESHOLD) {
          RCLCPP_INFO(this->get_logger(), "Returned to start position!");
          goal_active_ = false;
          transition_to(MissionState::LAND);
        }
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

double MissionFsmNode::calculate_distance(const geometry_msgs::msg::Point& p1,
                                          const geometry_msgs::msg::Point& p2) const
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void MissionFsmNode::publish_trajectory_goal(double x, double y, double z, double yaw)
{
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

  RCLCPP_DEBUG(this->get_logger(), "Published trajectory goal: [%.2f, %.2f, %.2f]", x, y, z);
}

void MissionFsmNode::publish_state()
{
  std_msgs::msg::String state_msg;
  state_msg.data = state_to_string(current_state_);
  state_pub_->publish(state_msg);
}

bool MissionFsmNode::is_goal_blacklisted(const geometry_msgs::msg::Point& goal) const
{
  for (const auto& blacklisted : blacklisted_goals_) {
    // Only check x,y distance (ignoring z since we retry different altitudes)
    double dx = goal.x - blacklisted.x;
    double dy = goal.y - blacklisted.y;
    double dist_2d = std::sqrt(dx * dx + dy * dy);
    if (dist_2d < BLACKLIST_THRESHOLD) {
      return true;
    }
  }
  return false;
}

}  // namespace control
