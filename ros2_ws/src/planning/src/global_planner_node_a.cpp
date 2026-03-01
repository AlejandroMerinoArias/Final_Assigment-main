#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>

#include <Eigen/Dense>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "octomap/OcTree.h"

using namespace std::chrono_literals;

class GlobalPlannerNode : public rclcpp::Node
{
public:
  GlobalPlannerNode()
  : rclcpp::Node("global_planner_node_a"),
    has_octomap_(false),
    has_odom_(false),
    robot_radius_(0.7),
    collision_check_resolution_(0.1),
    local_box_size_(40.0),
    z_band_(1.5),
    step_size_(2.0), 
    goal_radius_(3.0),
    max_iterations_(30000), // High iteration count for complex caves
    max_segment_length_(2.0) // Short segments to help trajectory optimizer
  {
    robot_radius_ = this->declare_parameter<double>("robot_radius", robot_radius_);
    collision_check_resolution_ = this->declare_parameter<double>("collision_check_resolution", collision_check_resolution_);
    local_box_size_ = this->declare_parameter<double>("local_box_size", local_box_size_);
    z_band_ = this->declare_parameter<double>("z_band", z_band_);
    step_size_ = this->declare_parameter<double>("step_size", step_size_);
    goal_radius_ = this->declare_parameter<double>("goal_radius", goal_radius_);
    max_iterations_ = this->declare_parameter<int>("max_iterations", max_iterations_);
    max_segment_length_ = this->declare_parameter<double>("max_segment_length", max_segment_length_);

    path_topic_ = this->declare_parameter<std::string>("path_topic", "waypoints");
    goal_topic_ = this->declare_parameter<std::string>("goal_topic", "/planner_a/goal");

    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/current_state_est");
    octomap_topic_ = this->declare_parameter<std::string>("octomap_topic", "/octomap_binary");

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 1);
    planner_status_pub_ = this->create_publisher<std_msgs::msg::String>("/planner/status", 10);

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic_, 1,
      std::bind(&GlobalPlannerNode::octomapCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&GlobalPlannerNode::odomCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10,
      std::bind(&GlobalPlannerNode::goalCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "GlobalPlannerNode initialized. Res=%.2f, MaxSeg=%.2f",
                step_size_, max_segment_length_);
  }

private:
  struct AStarNode
  {
    Eigen::Vector3d pos;
    double g_cost;
    double f_cost;
    bool operator>(const AStarNode & other) const {
      return f_cost > other.f_cost;
    }
  };

  struct GridIndex {
    int x, y, z;
    bool operator==(const GridIndex & other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct GridIndexHash {
    std::size_t operator()(const GridIndex & k) const {
      return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 1);
    }
  };

  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    octomap::AbstractOcTree * abstract = octomap_msgs::binaryMsgToMap(*msg);
    if (abstract) {
      octree_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(abstract));
      has_octomap_ = true;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pos_ = Eigen::Vector3d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    
    current_vel_ = Eigen::Vector3d(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z);

    has_odom_ = true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!has_octomap_ || !has_odom_) {
      publishPlannerStatus("PLAN_FAILED");
      return;
    }

    Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    nav_msgs::msg::Path path_msg;
    
    // Attempt plan
    if (!planPath(current_pos_, goal, path_msg)) {
      RCLCPP_WARN(get_logger(), "Failed to find a path.");
      publishPlannerStatus("PLAN_FAILED");
      return;
    }

    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = msg->header.frame_id.empty() ? "world" : msg->header.frame_id;
    path_pub_->publish(path_msg);
    
    RCLCPP_INFO(get_logger(), "Published path with %zu waypoints.", path_msg.poses.size());
  }

  void publishPlannerStatus(const std::string & status)
  {
    if (planner_status_pub_) {
      std_msgs::msg::String msg;
      msg.data = status;
      planner_status_pub_->publish(msg);
    }
  }

  // --- PATH PROCESSING TOOLS ---

  std::vector<Eigen::Vector3d> simplifyPath(const std::vector<Eigen::Vector3d> & path)
  {
    if (path.size() < 3) return path;
    std::vector<Eigen::Vector3d> smoothed;
    smoothed.push_back(path.front());
    size_t current_idx = 0;
    
    while (current_idx < path.size() - 1) {
      size_t best_next_idx = current_idx + 1;
      // Greedy Line-of-Sight Check
      for (size_t i = path.size() - 1; i > current_idx + 1; --i) {
        if (isSegmentCollisionFree(path[current_idx], path[i])) {
          best_next_idx = i;
          break;
        }
      }
      smoothed.push_back(path[best_next_idx]);
      current_idx = best_next_idx;
    }
    return smoothed;
  }

  std::vector<Eigen::Vector3d> interpolatePath(const std::vector<Eigen::Vector3d> & path)
  {
    if (path.empty()) return path;
    std::vector<Eigen::Vector3d> dense_path;
    dense_path.push_back(path[0]);

    for (size_t i = 0; i < path.size() - 1; ++i) {
      Eigen::Vector3d start = path[i];
      Eigen::Vector3d end = path[i+1];
      double dist = (end - start).norm();
      
      if (dist > max_segment_length_) {
        int segments = std::ceil(dist / max_segment_length_);
        for (int j = 1; j < segments; ++j) {
          double alpha = static_cast<double>(j) / segments;
          dense_path.push_back((1.0 - alpha) * start + alpha * end);
        }
      }
      dense_path.push_back(end);
    }
    return dense_path;
  }

  // AGGRESSIVE Nudging: Expands search radius to 3.0m if stuck
  Eigen::Vector3d findNearestFree(const Eigen::Vector3d & p) 
  {
    if (isStateFree(p)) return p;
    
    double max_search_r = 3.0; // Increased range
    double d_step = 0.5;
    
    // 1. Priority: Vertical Search (Often clearer in caves/tunnels)
    for (double z = -max_search_r; z <= max_search_r; z += d_step) {
        Eigen::Vector3d candidate = p + Eigen::Vector3d(0,0,z);
        if (isStateFree(candidate)) return candidate;
    }

    // 2. Secondary: Full Spiral Search
    for (double r = d_step; r <= max_search_r; r += d_step) {
      for (double x = -r; x <= r; x += r) { 
        for (double y = -r; y <= r; y += r) {
           // Keep vertical small here since we already checked main vertical axis
           for (double z = -1.0; z <= 1.0; z += 1.0) { 
              Eigen::Vector3d candidate = p + Eigen::Vector3d(x,y,z);
              if (isStateFree(candidate)) return candidate;
           }
        }
      }
    }
    return p; // Return original if nothing found (will fail later)
  }

  bool planPath(
    const Eigen::Vector3d & current_drone_pos,
    const Eigen::Vector3d & goal_raw,
    nav_msgs::msg::Path & out_path)
  {
    if (!octree_) return false;

    // 1. FIND VALID START/GOAL
    // We use the current position to start planning, but we NUDGE it if it's inside a wall.
    Eigen::Vector3d start_node_pos = findNearestFree(current_drone_pos);
    Eigen::Vector3d goal = findNearestFree(goal_raw);

    if (!isStateFree(start_node_pos)) {
        RCLCPP_WARN(get_logger(), "Start occupied even after 3m nudge.");
        return false;
    }
    if (!isStateFree(goal)) {
        // Fallback: Pull goal towards start if blocked
        Eigen::Vector3d dir = (start_node_pos - goal).normalized();
        goal = goal + dir * 1.0; 
        if (!isStateFree(goal)) {
           RCLCPP_WARN(get_logger(), "Goal occupied even after nudge & pull.");
           return false;
        }
    }

    // A* Initialization
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
    std::unordered_map<GridIndex, double, GridIndexHash> closed_list;
    std::unordered_map<GridIndex, Eigen::Vector3d, GridIndexHash> parent_map;

    open_list.push({start_node_pos, 0.0, (goal - start_node_pos).norm()});
    closed_list[posToGridIndex(start_node_pos)] = 0.0;
    
    // 26-Connectivity Directions
    std::vector<Eigen::Vector3d> directions;
    for(int dx = -1; dx <= 1; ++dx) {
      for(int dy = -1; dy <= 1; ++dy) {
        for(int dz = -1; dz <= 1; ++dz) {
          if(dx == 0 && dy == 0 && dz == 0) continue;
          directions.push_back(Eigen::Vector3d(dx, dy, dz).normalized() * step_size_);
        }
      }
    }

    Eigen::Vector3d final_node_pos = start_node_pos;
    bool found_goal = false;
    int nodes_expanded = 0;

    while (!open_list.empty()) {
      AStarNode current = open_list.top();
      open_list.pop();

      nodes_expanded++;
      if (nodes_expanded > max_iterations_) break;

      // Relaxed Goal Check: Reach within radius, then LOS check
      if ((current.pos - goal).norm() < goal_radius_ || (current.pos - goal).norm() < 3.0) {
         if (isSegmentCollisionFree(current.pos, goal)) {
            final_node_pos = goal;
            parent_map[posToGridIndex(goal)] = current.pos;
            found_goal = true;
            break;
         }
      }

      for (const auto & dir : directions) {
        Eigen::Vector3d neighbor_pos = current.pos + dir;

        // Bounding Box
        if (std::abs(neighbor_pos.x() - start_node_pos.x()) > local_box_size_ ||
            std::abs(neighbor_pos.y() - start_node_pos.y()) > local_box_size_ ||
            std::abs(neighbor_pos.z() - goal.z()) > z_band_) continue;

        double new_g = current.g_cost + step_size_; 
        GridIndex n_idx = posToGridIndex(neighbor_pos);

        if (closed_list.count(n_idx) && new_g >= closed_list[n_idx]) continue;
        if (!isSegmentCollisionFree(current.pos, neighbor_pos)) continue;

        closed_list[n_idx] = new_g;
        parent_map[n_idx] = current.pos;
        
        // 1.1 Heuristic Weight (Greedy search helps find paths faster)
        open_list.push({neighbor_pos, new_g, new_g + (goal - neighbor_pos).norm() * 1.1});
      }
    }

    if (!found_goal) return false;

    // Reconstruct Path
    std::vector<Eigen::Vector3d> raw_path;
    Eigen::Vector3d curr = final_node_pos;
    raw_path.push_back(curr);
    
    while (true) {
      GridIndex idx = posToGridIndex(curr);
      if (parent_map.find(idx) == parent_map.end()) break;
      Eigen::Vector3d parent = parent_map[idx];
      if ((parent - curr).norm() < 1e-4) break; 
      raw_path.push_back(parent);
      curr = parent;
      if ((curr - start_node_pos).norm() < step_size_ * 0.5) break; 
    }
    // Ensure start is included
    if ((raw_path.back() - start_node_pos).norm() > 1e-3) raw_path.push_back(start_node_pos);
    std::reverse(raw_path.begin(), raw_path.end());

    // 1. Smooth the path (Remove Zigzags)
    std::vector<Eigen::Vector3d> smoothed = simplifyPath(raw_path);

    // 2. Interpolate (Add points for trajectory optimizer)
    std::vector<Eigen::Vector3d> final_points = interpolatePath(smoothed);

    // 3. DRIFT COMPENSATION (Fix for "Looping Back")
    // Compare current drone position vs where we started planning.
    double drift_dist = (current_pos_ - start_node_pos).norm();
    if (drift_dist > 0.5) {
        // If the drone moved >0.5m during planning, PREPEND the current position.
        // This ensures the trajectory starts EXACTLY where the drone is now, 
        // preventing the controller from jerking backward.
        final_points.insert(final_points.begin(), current_pos_);
        RCLCPP_INFO(get_logger(), "Compensated for drift of %.2fm", drift_dist);
    }

    if (final_points.size() < 2) return false;

    out_path.poses.clear();
    for (const auto & p : final_points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();
      pose.pose.orientation.w = 1.0;
      out_path.poses.push_back(pose);
    }
    return true;
  }

  // --- UTILS ---
  GridIndex posToGridIndex(const Eigen::Vector3d & pos) const {
    GridIndex idx;
    double inv = 1.0 / step_size_;
    idx.x = static_cast<int>(std::floor(pos.x() * inv + 0.5));
    idx.y = static_cast<int>(std::floor(pos.y() * inv + 0.5));
    idx.z = static_cast<int>(std::floor(pos.z() * inv + 0.5));
    return idx;
  }

  bool isStateFree(const Eigen::Vector3d & p) const
  {
    if (!octree_) return false;
    octomap::point3d query(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
    octomap::OcTreeNode * node = octree_->search(query);
    if (node && node->getOccupancy() >= octree_->getOccupancyThres()) return false;
    
    // Inflation
    const double r = robot_radius_;
    const double step = std::max(collision_check_resolution_, 0.2); 
    
    for (double dx = -r; dx <= r; dx += step) {
      for (double dy = -r; dy <= r; dy += step) {
        for (double dz = -r; dz <= r; dz += step) {
          if (dx * dx + dy * dy + dz * dz > r * r) continue;
          octomap::point3d q(
            static_cast<float>(p.x() + dx),
            static_cast<float>(p.y() + dy),
            static_cast<float>(p.z() + dz));
          octomap::OcTreeNode * n = octree_->search(q);
          if (n && n->getOccupancy() >= octree_->getOccupancyThres()) return false;
        }
      }
    }
    return true;
  }

  bool isSegmentCollisionFree(const Eigen::Vector3d & from, const Eigen::Vector3d & to) const
  {
    const double dist = (to - from).norm();
    if (dist < 1e-3) return isStateFree(from);
    const double step = 0.2;
    const int steps = static_cast<int>(std::ceil(dist / step));
    for (int i = 0; i <= steps; ++i) {
      double t = static_cast<double>(i) / static_cast<double>(steps);
      if (!isStateFree((1.0 - t) * from + t * to)) return false;
    }
    return true;
  }

  // Parameters
  double robot_radius_;
  double collision_check_resolution_;
  double local_box_size_;
  double z_band_;
  double step_size_;
  double goal_radius_;
  int max_iterations_;
  double max_segment_length_;

  std::string path_topic_;
  std::string goal_topic_;
  std::string odom_topic_;
  std::string octomap_topic_;

  std::shared_ptr<octomap::OcTree> octree_;
  bool has_octomap_;
  bool has_odom_;
  Eigen::Vector3d current_pos_{0.0, 0.0, 0.0};
  Eigen::Vector3d current_vel_{0.0, 0.0, 0.0};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}