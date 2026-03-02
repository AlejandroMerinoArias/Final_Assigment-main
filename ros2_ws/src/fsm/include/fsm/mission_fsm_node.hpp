#ifndef FSM__MISSION_FSM_NODE_HPP_
#define FSM__MISSION_FSM_NODE_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/time.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "exploring/srv/get_exploration_goal.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace control {

/**
 * @brief Mission states for the FSM
 */
enum class MissionState {
  INIT,
  TAKEOFF,
  GOTO_ENTRANCE,
  EXPLORE,
  REFINE_GOAL, // Recovery state for Z-altitude retry
  LANTERN_FOUND,
  RETURN,
  LAND
};

/**
 * @brief Convert MissionState enum to string for logging/publishing
 */
inline std::string state_to_string(MissionState state) {
  switch (state) {
  case MissionState::INIT:
    return "INIT";
  case MissionState::TAKEOFF:
    return "TAKEOFF";
  case MissionState::GOTO_ENTRANCE:
    return "GOTO_ENTRANCE";
  case MissionState::EXPLORE:
    return "EXPLORE";
  case MissionState::REFINE_GOAL:
    return "REFINE_GOAL";
  case MissionState::LANTERN_FOUND:
    return "LANTERN_FOUND";
  case MissionState::RETURN:
    return "RETURN";
  case MissionState::LAND:
    return "LAND";
  default:
    return "UNKNOWN";
  }
}

/**
 * @brief Mission FSM Node - Orchestrates the drone's autonomous mission
 */
class MissionFsmNode : public rclcpp::Node {
public:
  MissionFsmNode();

private:
  // --- Callbacks ---
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void lantern_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void planner_status_callback(const std_msgs::msg::String::SharedPtr msg);
  void exploration_goal_callback(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void start_mission_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void depth_points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void timer_callback();
  void request_exploration_goal();

  // --- State machine logic ---
  void transition_to(MissionState new_state);
  void on_state_enter(MissionState state);
  void on_state_exit(MissionState state);
  void update_state();

  // --- Helper functions ---
  double calculate_distance(const geometry_msgs::msg::Point &p1,
                            const geometry_msgs::msg::Point &p2) const;
  void publish_trajectory_goal(double x, double y, double z, double yaw = 0.0);
  void publish_waypoint_path(
      const std::vector<geometry_msgs::msg::Point> &waypoints);
  void publish_state();
  void publish_drone_marker();
  void publish_checkpoint_markers();
  /// Re-send the current active goal to the planner so it recomputes the path
  /// from the drone's current position (mid-flight replanning).
  void replan_current_goal();
  void start_refine_for_current_goal(const std::string &reason);
  void build_z_retry_altitudes(double center_z);
  enum class GoalSource { EXPLORER, TRAVEL, POTENTIAL };
  bool try_activate_exploration_goal(const geometry_msgs::msg::Point &goal,
                                     bool allow_close_goal = false,
                                     GoalSource source = GoalSource::EXPLORER,
                                     int anchor_node_id = -1);
  void reset_explorer_goal_filters();
  bool is_inside_node(int node_id, const geometry_msgs::msg::Point &pos) const;
  std::optional<int> find_node_containing_position(const geometry_msgs::msg::Point &pos) const;
  int create_checkpoint_node(const geometry_msgs::msg::Point &pos, bool is_entrance = false,
                             bool is_provisional = false);
  void add_edge_between_nodes(int from_node, int to_node);
  void remove_checkpoint_node(int node_id);
  void update_checkpoint_graph();
  void update_mode_decision();
  void suspend_explorer_mode_for_travel();
  void resume_explorer_mode_after_travel();
  void register_potential_node_for_anchor(const geometry_msgs::msg::Point &candidate);
  bool is_potential_valid_global(const geometry_msgs::msg::Point &candidate) const;
  void prune_potentials_within_node_distance_recursive();
  bool node_has_resolvable_potential(int node_id) const;
  bool pop_next_potential_for_node(int node_id, geometry_msgs::msg::Point &goal_out);
  std::vector<int> compute_shortest_path_nodes(int start_node, int goal_node) const;
  void reset_graph_to_entrance();
  void prune_potential_nodes_near(const geometry_msgs::msg::Point &pos, double radius);
  double point_to_segment_distance(const geometry_msgs::msg::Point &p,
                                   const geometry_msgs::msg::Point &a,
                                   const geometry_msgs::msg::Point &b) const;
  bool is_within_node_distance_of_any_node(const geometry_msgs::msg::Point &pos) const;
  bool find_safer_node_position(const geometry_msgs::msg::Point &center,
                                geometry_msgs::msg::Point &safe_out) const;
  bool find_alternate_position_for_node(int node_id,
                                        geometry_msgs::msg::Point &candidate_out);
  void clear_node_relocation_state(int node_id);
  void mark_potential_node_unreachable_near(const geometry_msgs::msg::Point &pos);

  struct PotentialNode {
    geometry_msgs::msg::Point position;
    bool valid = false;
    bool unreachable = false;
  };

  struct CheckpointNode {
    int id = -1;
    geometry_msgs::msg::Point position;
    std::set<int> edges;
    bool is_dead_end = false;
    bool is_provisional = false;
    std::vector<PotentialNode> potentials;
  };

  // --- Subscribers ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr lantern_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      exploration_map_ready_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      depth_points_sub_;

  // --- Service Clients ---
  rclcpp::Client<exploring::srv::GetExplorationGoal>::SharedPtr
      exploration_goal_client_;

  // --- Publishers ---
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr
      trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoint_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cancel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      planner_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      planner_goal_pub_a_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      drone_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      checkpoint_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_mapping_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr blacklist_goal_pub_;
  // --- Timer ---
  rclcpp::TimerBase::SharedPtr timer_;

  // --- State variables ---
  MissionState current_state_;
  geometry_msgs::msg::Pose current_pose_;
  bool pose_received_;
  bool mission_start_signal_received_;
  bool goal_active_;
  bool goal_request_pending_;
  bool exploration_map_ready_ = false;

  // --- Lantern tracking ---
  std::vector<geometry_msgs::msg::Pose> detected_lantern_poses_;
  size_t lanterns_found_count_;
  double lantern_dedup_threshold_; // meters
  static constexpr size_t TARGET_LANTERN_COUNT = 4;

  // --- Mission parameters ---
  double takeoff_altitude_;
  geometry_msgs::msg::Point start_position_;
  geometry_msgs::msg::Point cave_entrance_;
  std::string planner_type_;  // "RRT" or "A_star"

  // --- Goal tracking ---
  geometry_msgs::msg::Point current_goal_;
  rclcpp::Time goal_set_time_;  // When the current goal was set
  geometry_msgs::msg::Point last_pose_at_goal_set_;  // Drone position when goal was set
  static constexpr double GOAL_REACHED_THRESHOLD = 1.5; // meters - increased from 0.5m for more lenient goal completion
  double min_exploration_goal_distance_; // meters - minimum distance for new goals (configurable)
  static constexpr double GOAL_TIMEOUT_SECONDS = 30.0; // seconds - timeout for goal completion
  static constexpr double STUCK_DETECTION_SECONDS = 10.0; // seconds before checking for stuck behavior
  static constexpr double MIN_MOVEMENT_THRESHOLD = 0.5; // meters - minimum movement to consider progress
  static constexpr double MIN_PROGRESS_THRESHOLD = 0.5; // meters - minimum progress toward goal before warning
  static constexpr int MAX_CONSECUTIVE_TOO_CLOSE_REJECTIONS = 3; // Accept goal after this many rejections
  int consecutive_too_close_rejections_; // Counter for consecutive "too close" rejections

  // --- Mid-flight replanning ---
  rclcpp::Time last_replan_time_;   ///< When the last replan was issued
  double replan_interval_s_ = 3.0;  ///< How often to replan the same goal (seconds)

  // --- Exploration goal-selection tracking ---
  int consecutive_goal_request_failures_ = 0;              // How many goal service calls failed in a row
  rclcpp::Time last_successful_exploration_goal_time_;     // When we last activated a new exploration goal

  // --- Exploration goal-selection watchdog configuration ---
  double explore_goal_selection_timeout_ = 60.0;           // Seconds since last successful goal before considering selection stuck
  int explore_goal_selection_max_failures_ = 50;           // Max consecutive failed requests before considering selection stuck

  // --- TF ---
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- Z-Retry (Goal Refinement) ---
  std::vector<double> z_retry_altitudes_;    // List of altitudes to try
  size_t z_retry_index_;                     // Current index in retry list
  geometry_msgs::msg::Point strategic_goal_; // The (x,y) we're trying to reach
  int z_retry_max_attempts_ = 3;             // Max altitudes to try per strategic goal
  double z_retry_step_ = 1.0;                // Altitude spacing between retries

  // --- Macroplanning graph state ---
  std::unordered_map<int, CheckpointNode> graph_nodes_;
  int next_node_id_ = 0;
  int entrance_node_id_ = -1;
  int last_visited_node_id_ = -1;
  int current_node_id_ = -1;
  int previous_node_id_ = -1;
  bool travel_mode_ = false;
  bool explorer_mode_suspended_for_travel_ = false;
  int potential_resolution_node_id_ = -1;
  bool suppress_rule_l_ = false;
  std::deque<int> travel_path_;

  double nodes_distance_ = 30.0;
  double node_radius_ = 15.0;
  bool macroplanning_enabled_ = true;
  double max_potential_node_range_ = 60.0;
  double potential_angle_threshold_deg_ = 30.0;
  geometry_msgs::msg::Point latest_seen_point_;
  bool latest_seen_point_valid_ = false;
  rclcpp::Time latest_seen_point_stamp_;
  double seen_point_timeout_s_ = 1.0;
  std::vector<geometry_msgs::msg::Point> recent_obstacle_points_;
  GoalSource active_goal_source_ = GoalSource::EXPLORER;
  int active_goal_anchor_node_id_ = -1;
  bool reset_explorer_filters_on_next_goal_ = false;
  bool force_explorer_until_new_node_ = false;
  int fallback_origin_node_id_ = -1;
  rclcpp::Time force_explorer_start_time_;
  double force_explorer_timeout_s_ = 20.0;
  uint64_t explorer_request_epoch_ = 0;
  rclcpp::Time goal_request_sent_time_;
  double goal_request_timeout_s_ = 4.0;
  std::unordered_map<int, geometry_msgs::msg::Point> node_relocation_origin_;
  std::unordered_map<int, int> node_relocation_attempts_;
};

} // namespace control

#endif // FSM__MISSION_FSM_NODE_HPP_
