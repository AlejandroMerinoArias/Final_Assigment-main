#ifndef EXPLORING__EXPLORATION_MANAGER_HPP_
#define EXPLORING__EXPLORATION_MANAGER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include "exploring/srv/get_exploration_goal.hpp"

#include "octomap/OcTree.h"
#include "octomap/octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace planning {

// ---------------------------------------------------------------------------
// Spatial hashing for 3D voxel clustering
// ---------------------------------------------------------------------------

// --- Dai-Lite Sampling Pipeline Structures ---

/// A single 3D frontier candidate point with its utility score
struct FrontierCandidate {
  octomap::point3d position;
  double utility = 0.0;
};

/// Stuck mode classification for adaptive filter relaxation
enum class StuckMode {
  NONE,       ///< Normal operation, no stuck detected
  OBSTACLE,   ///< Stuck due to obstacle filter dominating
  LOS,        ///< Stuck due to line-of-sight filter dominating
  DISTANCE,   ///< Stuck due to min-distance filter dominating
  BLACKLIST   ///< Stuck due to blacklist filter dominating
};

// ---------------------------------------------------------------------------
// ExplorationManager – True 3D frontier-based exploration
// ---------------------------------------------------------------------------

/**
 * @brief 3D Frontier-Based Exploration Manager.
 *
 * Detects frontiers directly in the OctoMap (no 2D slicing), clusters them
 * in 3D with spatial hashing, estimates information gain via bounding-box
 * queries, and selects the best exploration goal using a composite score.
 *
 * Provides a SERVICE interface (`/exploration/get_goal`) that returns a
 * full 3D goal (x, y, z) to the FSM.
 *
 * Optionally listens on `/exploration/blacklist_goal` so the FSM can
 * communicate goals that should be avoided.
 */
class ExplorationManager : public rclcpp::Node {
public:
  ExplorationManager();

private:
  // --- Callbacks --------------------------------------------------------

  void map_callback(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void
  blacklist_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  // --- Service handler --------------------------------------------------

  void handle_get_goal(
      const std::shared_ptr<exploring::srv::GetExplorationGoal::Request>
          request,
      std::shared_ptr<exploring::srv::GetExplorationGoal::Response> response);

  // --- Dai-Lite Sampling Pipeline (fast) --------------------------------

  /// Detect all frontier voxels (free with ≥1 unknown neighbor) in the OctoMap
  /// Returns raw 3D points.
  std::vector<octomap::point3d> detect_frontier_voxels(const octomap::OcTree &tree);

  /// Downsample frontier points to a coarser grid to spread candidates spatially
  /// and reduce the number of points.
  std::vector<octomap::point3d> voxel_grid_downsample(
      const std::vector<octomap::point3d> &points, double grid_size);

  /// Randomly sample N candidates from the downsampled frontier set
  std::vector<FrontierCandidate> sample_candidates(
      const std::vector<octomap::point3d> &frontier_points, int num_candidates);

  /// Evaluate a single candidate's utility = 1 / effective_time
  double evaluate_candidate(const octomap::point3d &candidate,
                            const octomap::point3d &drone_pos);

  /// Check if a point is too close to obstacles (within inflation radius).
  /// Uses the 3D OctoMap to check for occupied voxels within the inflation radius.
  bool is_too_close_to_obstacle(double x, double y, double z) const;
  
  /// Check if a point is too close to obstacles with a custom clearance radius.
  /// Used for stuck-mode relaxation.
  bool is_too_close_to_obstacle(double x, double y, double z, double clearance_radius) const;

  // --- Goal Memory / Blacklisting ---------------------------------------

  /// Returns true if a goal is within blacklist_radius_ of any blacklisted
  /// point.
  bool is_blacklisted(const octomap::point3d &goal) const;

  // --- Utilities --------------------------------------------------------

  bool get_drone_position(double &x, double &y, double &z);

  /// Check if a vertical column at (x, y) is already well observed in 3D.
  /// Uses the underlying OctoMap across a range of z values to see whether
  /// multiple voxels along that column have been observed (free or occupied).
  bool is_column_explored(double x, double y) const;

  void publish_visualization(const std::vector<FrontierCandidate> &candidates,
                             const FrontierCandidate *best, double drone_x,
                             double drone_y, double drone_z);
  
  // --- Stuck-mode detection and adaptive filter relaxation --------------
  
  /// Update failure statistics with the latest filter counts
  void updateFailureStats(int filt_blacklist, int filt_obstacle, int filt_distance,
                         int filt_cave, int filt_los, int total_candidates);
  
  /// Compute current stuck mode and severity factor based on failure stats
  /// Returns (mode, severity) where severity is in [0, 1]
  std::pair<StuckMode, double> computeStuckMode() const;
  
  /// Reset stuck-mode statistics (called on successful goal selection)
  void resetStuckStats();

  // --- Test access ------------------------------------------------------
  friend class ExplorationManagerTest;

  // --- ROS interfaces ---------------------------------------------------

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      blacklist_sub_;
  rclcpp::Service<exploring::srv::GetExplorationGoal>::SharedPtr goal_service_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr slice_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr map_ready_pub_;
  
  // Performance logging
  std::ofstream perf_log_file_;
  int request_count_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- State ------------------------------------------------------------

  std::shared_ptr<octomap::OcTree> current_octomap_;
  nav_msgs::msg::OccupancyGrid current_sliced_map_;
  std::vector<octomap::point3d> blacklisted_goals_;

  // Entrance / global reference for generic "forward into cave" bias.
  // Set to the drone pose when we receive the FIRST exploration goal request.
  octomap::point3d entrance_pos_;
  bool has_entrance_pos_ = false;
  bool map_ready_ = false;

  // --- Parameters -------------------------------------------------------

  int min_frontier_size_;   ///< Min neighbors/points filter (if used)
  double blacklist_radius_; ///< Radius around blacklisted goals to avoid (m)
  double min_z_;            ///< Minimum navigable altitude (m)
  double max_z_;            ///< Maximum navigable altitude (m)
  double max_step_distance_; ///< Max step length from drone to goal (m)
  double cave_entrance_x_;  ///< X threshold for cave entrance (points with x > this are outside)
  double exploration_inflation_radius_; ///< Minimum distance from obstacles for exploration goals (m)
  double min_goal_distance_; ///< Minimum distance from drone for a frontier to be considered (m)
  
  // Dai-Lite specific parameters
  int num_candidates_;         ///< Number of random samples (default 20)
  double downsample_grid_;     ///< Voxel grid size for spatial spreading (m)
  double drone_speed_;         ///< Assumed drone speed for time estimation (m/s)
  double vertical_penalty_weight_; ///< Penalty weight for altitude changes
  double frontier_search_radius_; ///< Radius around drone to search for frontiers (m)
  
  int consecutive_failures_;   ///< Track consecutive goal generation failures for adaptive scaling
  
  // --- Stuck-mode state and parameters ----------------------------------
  
  /// Exponentially weighted moving average of filter failure fractions
  double fail_stat_blacklist_;
  double fail_stat_obstacle_;
  double fail_stat_distance_;
  double fail_stat_cave_;
  double fail_stat_los_;
  
  /// Base thresholds for computing relaxed values
  double base_min_goal_distance_;
  double base_blacklist_radius_;
  double base_obstacle_clearance_;
  
  /// Stuck-mode tuning parameters
  int min_stuck_failures_;        ///< Min consecutive failures to consider stuck (default 10)
  double stuck_fraction_threshold_; ///< Min fraction for a filter to dominate (default 0.5)
  double stuck_alpha_;            ///< EMA alpha for failure stats (default 0.25)
  double los_short_range_threshold_; ///< Distance below which LOS is relaxed in stuck mode (default 6.0m)
};

} // namespace planning

#endif // EXPLORING__EXPLORATION_MANAGER_HPP_
