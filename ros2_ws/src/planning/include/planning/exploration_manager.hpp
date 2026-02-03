#ifndef PLANNING__EXPLORATION_MANAGER_HPP_
#define PLANNING__EXPLORATION_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vector>
#include <string>
#include <utility>
#include <memory>

namespace planning
{

/**
 * @brief Represents a cluster of frontier cells.
 */
struct FrontierCluster
{
  std::vector<std::pair<int, int>> cells;  ///< (row, col) indices in the grid
  geometry_msgs::msg::Point centroid;       ///< Centroid in map frame (x, y)
  double score;                             ///< Higher is better (1 / distance)
};

/**
 * @brief ExplorationManager node implementing Yamauchi's frontier-based exploration.
 *
 * This node detects frontiers (boundaries between known-free and unknown space)
 * on a 2D OccupancyGrid and publishes the best frontier as a strategic goal.
 */
class ExplorationManager : public rclcpp::Node
{
public:
  ExplorationManager();

private:
  // Callbacks
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void update_timer_callback();

  // Core algorithm
  std::vector<std::pair<int, int>> detect_frontier_cells(
    const nav_msgs::msg::OccupancyGrid & map);
  std::vector<FrontierCluster> cluster_frontiers(
    const std::vector<std::pair<int, int>> & frontier_cells,
    const nav_msgs::msg::OccupancyGrid & map);
  void calculate_centroid(
    FrontierCluster & cluster,
    const nav_msgs::msg::OccupancyGrid & map);
  void score_frontiers(
    std::vector<FrontierCluster> & clusters,
    double drone_x, double drone_y);
  FrontierCluster * select_best_frontier(std::vector<FrontierCluster> & clusters);

  friend class ExplorationManagerTest;

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;

  // Utilities
  bool get_drone_position(double & x, double & y);
  void publish_visualization(
    const std::vector<FrontierCluster> & clusters,
    const FrontierCluster * best,
    double drone_x, double drone_y);

  // Timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // State
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  std::vector<std::pair<int, int>> blacklisted_centroids_;  ///< Failed goals

  // Parameters
  int min_frontier_size_;
  double update_rate_hz_;
};

}  // namespace planning

#endif  // PLANNING__EXPLORATION_MANAGER_HPP_
