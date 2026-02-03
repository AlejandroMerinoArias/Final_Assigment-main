#include "exploring/exploration_manager.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

namespace planning
{

ExplorationManager::ExplorationManager()
: Node("exploration_manager"),
  min_frontier_size_(5),
  update_rate_hz_(2.0)
{
  // Declare parameters
  this->declare_parameter("min_frontier_size", min_frontier_size_);
  this->declare_parameter("update_rate_hz", update_rate_hz_);
  min_frontier_size_ = this->get_parameter("min_frontier_size").as_int();
  update_rate_hz_ = this->get_parameter("update_rate_hz").as_double();

  RCLCPP_INFO(this->get_logger(), "Exploration Manager started.");
  RCLCPP_INFO(this->get_logger(), "  min_frontier_size: %d", min_frontier_size_);
  RCLCPP_INFO(this->get_logger(), "  update_rate_hz: %.2f", update_rate_hz_);

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscriptions
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    std::bind(&ExplorationManager::map_callback, this, std::placeholders::_1));

  // Publishers
  goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/exploration/strategic_goal", 10);
  status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/exploration/status", 10);
  viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/exploration/frontiers_viz", 10);

  // Timer
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_hz_));
  update_timer_ = this->create_wall_timer(
    period, std::bind(&ExplorationManager::update_timer_callback, this));
}

void ExplorationManager::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_map_ = msg;
}

void ExplorationManager::update_timer_callback()
{
  if (!latest_map_) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Waiting for map...");
    return;
  }

  // Get drone position
  double drone_x, drone_y;
  if (!get_drone_position(drone_x, drone_y)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Cannot get drone position from TF.");
    return;
  }

  // Step 1: Detect frontier cells
  auto frontier_cells = detect_frontier_cells(*latest_map_);
  if (frontier_cells.empty()) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "No frontier cells found.");
    std_msgs::msg::String status;
    status.data = "NO_FRONTIERS";
    status_pub_->publish(status);
    return;
  }

  // Step 2: Cluster frontiers
  auto clusters = cluster_frontiers(frontier_cells, *latest_map_);
  if (clusters.empty()) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "No valid frontier clusters (all too small).");
    std_msgs::msg::String status;
    status.data = "NO_FRONTIERS";
    status_pub_->publish(status);
    return;
  }

  // Step 3: Calculate centroids
  for (auto & cluster : clusters) {
    calculate_centroid(cluster, *latest_map_);
  }

  // Step 4: Score frontiers
  score_frontiers(clusters, drone_x, drone_y);

  // Step 5: Select best frontier
  FrontierCluster * best = select_best_frontier(clusters);

  // Step 6: Publish goal and visualization
  if (best) {
    geometry_msgs::msg::PointStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    goal.point = best->centroid;
    goal_pub_->publish(goal);

    std_msgs::msg::String status;
    status.data = "EXPLORING";
    status_pub_->publish(status);

    RCLCPP_INFO(this->get_logger(), "Publishing goal: (%.2f, %.2f)",
                best->centroid.x, best->centroid.y);
  }

  publish_visualization(clusters, best, drone_x, drone_y);
}

std::vector<std::pair<int, int>> ExplorationManager::detect_frontier_cells(
  const nav_msgs::msg::OccupancyGrid & map)
{
  std::vector<std::pair<int, int>> frontiers;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);

  // Neighbor offsets (4-connected)
  const int dx[] = {0, 0, 1, -1};
  const int dy[] = {1, -1, 0, 0};

  for (int row = 1; row < height - 1; ++row) {
    for (int col = 1; col < width - 1; ++col) {
      int idx = row * width + col;
      int8_t cell = map.data[idx];

      // Cell must be FREE (value 0)
      if (cell != 0) {
        continue;
      }

      // Check if any neighbor is UNKNOWN (-1)
      bool is_frontier = false;
      for (int i = 0; i < 4; ++i) {
        int nr = row + dy[i];
        int nc = col + dx[i];
        int nidx = nr * width + nc;
        if (map.data[nidx] == -1) {
          is_frontier = true;
          break;
        }
      }

      if (is_frontier) {
        frontiers.emplace_back(row, col);
      }
    }
  }

  return frontiers;
}

std::vector<FrontierCluster> ExplorationManager::cluster_frontiers(
  const std::vector<std::pair<int, int>> & frontier_cells,
  const nav_msgs::msg::OccupancyGrid & map)
{
  std::vector<FrontierCluster> clusters;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);

  // Create a visited grid
  std::vector<bool> visited(width * height, false);

  // Mark all frontier cells
  std::vector<bool> is_frontier(width * height, false);
  for (const auto & cell : frontier_cells) {
    int idx = cell.first * width + cell.second;
    is_frontier[idx] = true;
  }

  // BFS to cluster adjacent frontier cells
  const int dx[] = {0, 0, 1, -1, 1, 1, -1, -1};
  const int dy[] = {1, -1, 0, 0, 1, -1, 1, -1};

  for (const auto & start : frontier_cells) {
    int start_idx = start.first * width + start.second;
    if (visited[start_idx]) {
      continue;
    }

    // BFS from this cell
    FrontierCluster cluster;
    std::queue<std::pair<int, int>> q;
    q.push(start);
    visited[start_idx] = true;

    while (!q.empty()) {
      auto current = q.front();
      q.pop();
      cluster.cells.push_back(current);

      // Check 8-connected neighbors
      for (int i = 0; i < 8; ++i) {
        int nr = current.first + dy[i];
        int nc = current.second + dx[i];
        if (nr < 0 || nr >= height || nc < 0 || nc >= width) {
          continue;
        }
        int nidx = nr * width + nc;
        if (!visited[nidx] && is_frontier[nidx]) {
          visited[nidx] = true;
          q.emplace(nr, nc);
        }
      }
    }

    // Filter by minimum size
    if (static_cast<int>(cluster.cells.size()) >= min_frontier_size_) {
      clusters.push_back(cluster);
    }
  }

  return clusters;
}

void ExplorationManager::calculate_centroid(
  FrontierCluster & cluster,
  const nav_msgs::msg::OccupancyGrid & map)
{
  double sum_x = 0.0, sum_y = 0.0;
  const double resolution = map.info.resolution;
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;

  for (const auto & cell : cluster.cells) {
    // Convert grid (row, col) to world (x, y)
    double x = origin_x + (cell.second + 0.5) * resolution;
    double y = origin_y + (cell.first + 0.5) * resolution;
    sum_x += x;
    sum_y += y;
  }

  cluster.centroid.x = sum_x / cluster.cells.size();
  cluster.centroid.y = sum_y / cluster.cells.size();
  cluster.centroid.z = 0.0;  // 2D strategic goal
}

void ExplorationManager::score_frontiers(
  std::vector<FrontierCluster> & clusters,
  double drone_x, double drone_y)
{
  for (auto & cluster : clusters) {
    double dx = cluster.centroid.x - drone_x;
    double dy = cluster.centroid.y - drone_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Avoid division by zero
    if (distance < 0.1) {
      distance = 0.1;
    }

    // Greedy heuristic: closer is better
    cluster.score = 1.0 / distance;
  }
}

FrontierCluster * ExplorationManager::select_best_frontier(
  std::vector<FrontierCluster> & clusters)
{
  if (clusters.empty()) {
    return nullptr;
  }

  FrontierCluster * best = nullptr;
  double best_score = -1.0;

  for (auto & cluster : clusters) {
    if (cluster.score > best_score) {
      best_score = cluster.score;
      best = &cluster;
    }
  }

  return best;
}

bool ExplorationManager::get_drone_position(double & x, double & y)
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      "map", "base_link", tf2::TimePointZero);
    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(this->get_logger(), "TF exception: %s", ex.what());
    return false;
  }
}

void ExplorationManager::publish_visualization(
  const std::vector<FrontierCluster> & clusters,
  const FrontierCluster * best,
  double drone_x, double drone_y)
{
  visualization_msgs::msg::MarkerArray markers;
  int id = 0;

  // Clear previous markers
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  // Draw all frontier clusters
  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto & cluster = clusters[i];
    bool is_best = (best != nullptr && &cluster == best);

    // Centroid sphere
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = "map";
    sphere.header.stamp = this->now();
    sphere.ns = "frontier_centroids";
    sphere.id = id++;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position = cluster.centroid;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = is_best ? 0.6 : 0.3;  // Slightly bigger for best
    sphere.scale.y = is_best ? 0.6 : 0.3;
    sphere.scale.z = is_best ? 0.6 : 0.3;
    
    // Green if best, Yellow if candidate
    if (is_best) {
        sphere.color.r = 0.0f; sphere.color.g = 1.0f; sphere.color.b = 0.0f;
    } else {
        sphere.color.r = 1.0f; sphere.color.g = 0.8f; sphere.color.b = 0.0f;
    }
    sphere.color.a = 1.0f;
    
    sphere.lifetime = rclcpp::Duration::from_seconds(1.0);
    markers.markers.push_back(sphere);
  }

  // Draw line to the best frontier
  if (best != nullptr) {
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = this->now();
    line.ns = "choice_line";
    line.id = id++;
    line.type = visualization_msgs::msg::Marker::ARROW;
    line.action = visualization_msgs::msg::Marker::ADD;
    
    // Arrow start (drone) and end (goal)
    geometry_msgs::msg::Point p_start;
    p_start.x = drone_x; p_start.y = drone_y; p_start.z = 1.5; // Assume flight height
    line.points.push_back(p_start);
    line.points.push_back(best->centroid);

    line.scale.x = 0.1; // Shaft diameter
    line.scale.y = 0.2; // Head diameter
    line.scale.z = 0.0;
    
    line.color.r = 0.0f;
    line.color.g = 1.0f;
    line.color.b = 0.0f;
    line.color.a = 0.8f;
    
    line.lifetime = rclcpp::Duration::from_seconds(1.0);
    markers.markers.push_back(line);
  }

  viz_pub_->publish(markers);
}

}  // namespace planning
