#ifndef FSM__MISSION_FSM_NODE_HPP_
#define FSM__MISSION_FSM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <vector>
#include <string>
#include <cmath>

namespace control
{

/**
 * @brief Mission states for the FSM
 */
enum class MissionState
{
  INIT,
  TAKEOFF,
  GOTO_ENTRANCE,
  EXPLORE,
  REFINE_GOAL,  // Recovery state for Z-altitude retry
  LANTERN_FOUND,
  RETURN,
  LAND
};

/**
 * @brief Convert MissionState enum to string for logging/publishing
 */
inline std::string state_to_string(MissionState state)
{
  switch (state) {
    case MissionState::INIT: return "INIT";
    case MissionState::TAKEOFF: return "TAKEOFF";
    case MissionState::GOTO_ENTRANCE: return "GOTO_ENTRANCE";
    case MissionState::EXPLORE: return "EXPLORE";
    case MissionState::REFINE_GOAL: return "REFINE_GOAL";
    case MissionState::LANTERN_FOUND: return "LANTERN_FOUND";
    case MissionState::RETURN: return "RETURN";
    case MissionState::LAND: return "LAND";
    default: return "UNKNOWN";
  }
}

/**
 * @brief Mission FSM Node - Orchestrates the drone's autonomous mission
 */
class MissionFsmNode : public rclcpp::Node
{
public:
  MissionFsmNode();

private:
  // --- Callbacks ---
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void lantern_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void planner_status_callback(const std_msgs::msg::String::SharedPtr msg);
  void exploration_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void start_mission_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void timer_callback();

  // --- State machine logic ---
  void transition_to(MissionState new_state);
  void on_state_enter(MissionState state);
  void on_state_exit(MissionState state);
  void update_state();

  // --- Helper functions ---
  double calculate_distance(const geometry_msgs::msg::Point& p1, 
                            const geometry_msgs::msg::Point& p2) const;
  void publish_trajectory_goal(double x, double y, double z, double yaw = 0.0);
  void publish_state();
  bool is_goal_blacklisted(const geometry_msgs::msg::Point& goal) const;

  // --- Subscribers ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr lantern_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr exploration_goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_mission_sub_;

  // --- Publishers ---
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cancel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr planner_goal_pub_;

  // --- Timer ---
  rclcpp::TimerBase::SharedPtr timer_;

  // --- State variables ---
  MissionState current_state_;
  geometry_msgs::msg::Pose current_pose_;
  bool pose_received_;
  bool mission_start_signal_received_;
  bool goal_active_;

  // --- Lantern tracking ---
  std::vector<geometry_msgs::msg::Pose> detected_lantern_poses_;
  size_t lanterns_found_count_;
  static constexpr double LANTERN_DEDUP_THRESHOLD = 2.0;  // meters
  static constexpr size_t TARGET_LANTERN_COUNT = 4;

  // --- Mission parameters ---
  double takeoff_altitude_;
  geometry_msgs::msg::Point start_position_;
  geometry_msgs::msg::Point cave_entrance_;
  
  // --- Goal tracking ---
  geometry_msgs::msg::Point current_goal_;
  static constexpr double GOAL_REACHED_THRESHOLD = 0.5;  // meters

  // --- Z-Retry (Goal Refinement) ---
  std::vector<double> z_retry_altitudes_;  // List of altitudes to try
  size_t z_retry_index_;                    // Current index in retry list
  geometry_msgs::msg::Point strategic_goal_; // The (x,y) we're trying to reach
  std::vector<geometry_msgs::msg::Point> blacklisted_goals_;  // Goals that failed all retries
  static constexpr double BLACKLIST_THRESHOLD = 1.0;  // meters
};

}  // namespace control

#endif  // FSM__MISSION_FSM_NODE_HPP_
