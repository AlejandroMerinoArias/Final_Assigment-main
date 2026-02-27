#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "octomap/OcTree.h"

using namespace std::chrono_literals;

class GlobalPlannerNode : public rclcpp::Node {
public:
  GlobalPlannerNode()
      : rclcpp::Node("global_planner_node"), has_octomap_(false),
        has_odom_(false),
        // Slightly inflated collision radius to keep more distance to walls.
        robot_radius_(0.7),
        collision_check_resolution_(0.1), // NEW: explicit control over sampling
        local_box_size_(40.0), z_band_(1.5), step_size_(2.0), goal_radius_(3.0),
        max_iterations_(2000), max_planning_time_sec_(1.5) {
    robot_radius_ =
        this->declare_parameter<double>("robot_radius", robot_radius_);
    collision_check_resolution_ = this->declare_parameter<double>(
        "collision_check_resolution", collision_check_resolution_);
    local_box_size_ =
        this->declare_parameter<double>("local_box_size", local_box_size_);
    z_band_ = this->declare_parameter<double>("z_band", z_band_);
    step_size_ = this->declare_parameter<double>("step_size", step_size_);
    goal_radius_ = this->declare_parameter<double>("goal_radius", goal_radius_);
    max_iterations_ =
        this->declare_parameter<int>("max_iterations", max_iterations_);
    max_planning_time_sec_ = this->declare_parameter<double>(
        "max_planning_time_sec", max_planning_time_sec_);

    path_topic_ =
        this->declare_parameter<std::string>("path_topic", "waypoints");
    goal_topic_ =
        this->declare_parameter<std::string>("goal_topic", "/planner/goal");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic",
                                                       "/current_state_est");
    octomap_topic_ = this->declare_parameter<std::string>("octomap_topic",
                                                          "/octomap_binary");

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 1);
    planner_status_pub_ =
        this->create_publisher<std_msgs::msg::String>("/planner/status", 10);
    drone_radius_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/planner/drone_radius_marker", 10);

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        octomap_topic_, 1,
        std::bind(&GlobalPlannerNode::octomapCallback, this,
                  std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&GlobalPlannerNode::odomCallback, this,
                  std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_, 10,
        std::bind(&GlobalPlannerNode::goalCallback, this,
                  std::placeholders::_1));

    drone_radius_timer_ = this->create_wall_timer(
        100ms, std::bind(&GlobalPlannerNode::publishDroneRadiusMarker, this));

    RCLCPP_INFO(get_logger(),
                "GlobalPlannerNode initialized. robot_radius=%.2f m, "
                "local_box_size=%.1f m, "
                "z_band=%.1f m, step_size=%.1f m, goal_radius=%.1f m, "
                "max_iterations=%d, max_planning_time_sec=%.2f",
                robot_radius_, local_box_size_, z_band_, step_size_,
                goal_radius_, max_iterations_, max_planning_time_sec_);
  }

private:
  struct Node {
    Eigen::Vector3d pos;
    int parent = -1;
    double cost = 0.0;
  };

  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    octomap::AbstractOcTree *abstract = octomap_msgs::binaryMsgToMap(*msg);
    if (!abstract) {
      RCLCPP_WARN(get_logger(), "Failed to convert Octomap message to OcTree.");
      return;
    }

    std::shared_ptr<octomap::OcTree> tree(
        dynamic_cast<octomap::OcTree *>(abstract));
    if (!tree) {
      RCLCPP_WARN(get_logger(), "Received Octomap is not an OcTree.");
      delete abstract;
      return;
    }

    octree_ = tree;
    has_octomap_ = true;
    map_resolution_ = octree_->getResolution();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_ =
        Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
    has_odom_ = true;
  }

  // Visualize drone safety radius
  void publishDroneRadiusMarker() {
    if (!has_odom_) {
      return;
    }
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->now();
    marker.ns = "drone_radius";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = current_pos_.x();
    marker.pose.position.y = current_pos_.y();
    marker.pose.position.z = current_pos_.z();
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.scale.x = robot_radius_ * 2.0;
    marker.scale.y = robot_radius_ * 2.0;
    marker.scale.z = robot_radius_ * 2.0;

    // Check if current position is collision-free (for color feedback)
    if (isStateFree(current_pos_)) {
      // Safe: Cyan, semi-transparent
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 0.3;
    } else {
      // Collision: Red, more opaque
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 0.6;
    }

    drone_radius_pub_->publish(marker);
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!has_octomap_) {
      RCLCPP_WARN(get_logger(), "No Octomap received yet, cannot plan.");
      publishPlannerStatus("PLAN_FAILED");
      return;
    }
    if (!has_odom_) {
      RCLCPP_WARN(get_logger(), "No odometry received yet, cannot plan.");
      publishPlannerStatus("PLAN_FAILED");
      return;
    }

    Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y,
                         msg->pose.position.z);
    
    if (!isStateFree(current_pos_)) {
      RCLCPP_WARN(get_logger(),
                  "Planning start is in collision at [%.2f, %.2f, %.2f].",
                  current_pos_.x(), current_pos_.y(), current_pos_.z());
    }
    if (!isStateFree(goal)) {
      RCLCPP_WARN(get_logger(),
                  "Planning goal is in collision at [%.2f, %.2f, %.2f].",
                  goal.x(), goal.y(), goal.z());
      publishPlannerStatus("PLAN_FAILED");
      publishEmptyPath(msg->header.frame_id);
      return;
    }

    nav_msgs::msg::Path path_msg;
    std::string failure_reason;
    if (!planPath(current_pos_, goal, path_msg, &failure_reason)) {
      RCLCPP_WARN(get_logger(), "Failed to find a path with RRT*: %s",
                  failure_reason.c_str());
      publishPlannerStatus("PLAN_FAILED");
      publishEmptyPath(msg->header.frame_id);
      return;
      publishPlannerStatus("PLAN_FAILED");
      return;
    }

    path_msg.header.stamp = this->now();
    path_msg.header.frame_id =
        msg->header.frame_id.empty() ? "world" : msg->header.frame_id;

    path_pub_->publish(path_msg);
    RCLCPP_INFO(get_logger(),
                "Published RRT* path with %zu waypoints from [%.2f, %.2f, "
                "%.2f] to [%.2f, %.2f, %.2f].",
                path_msg.poses.size(), current_pos_.x(), current_pos_.y(),
                current_pos_.z(), goal.x(), goal.y(), goal.z());
  }

  void publishPlannerStatus(const std::string &status) {
    if (!planner_status_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = status;
    planner_status_pub_->publish(msg);
  }

  void publishEmptyPath(const std::string &frame_id) {
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->now();
    empty_path.header.frame_id = frame_id.empty() ? "world" : frame_id;
    path_pub_->publish(empty_path);
  }

  bool planPath(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                nav_msgs::msg::Path &out_path, std::string *failure_reason) {
    if (!octree_) {
      if (failure_reason) {
        *failure_reason = "octree not available";
      }
      return false;
    }

    const auto planning_start = std::chrono::steady_clock::now();

    Eigen::Vector3d check_start = start;
    if (!isStateFree(start)) {
      Eigen::Vector3d recovered_start;
      if (!findNearbyFreeStart(start, recovered_start)) {
        RCLCPP_WARN(
            get_logger(),
            "Start state is in collision and no nearby free state was found.");
        return false;
      }
      check_start = recovered_start;
      RCLCPP_WARN(get_logger(),
                  "Start state is in collision. Recovered planning start from "
                  "[%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f].",
                  start.x(), start.y(), start.z(), check_start.x(),
                  check_start.y(), check_start.z());
    }

    std::vector<Node> nodes;
    nodes.reserve(static_cast<size_t>(max_iterations_) + 2);
    nodes.push_back(Node{check_start, -1, 0.0});

    std::mt19937 rng(std::random_device{}());
    const double z_center = goal.z();
    std::uniform_real_distribution<double> dist_x(
        check_start.x() - local_box_size_, check_start.x() + local_box_size_);
    std::uniform_real_distribution<double> dist_y(
        check_start.y() - local_box_size_, check_start.y() + local_box_size_);
    std::uniform_real_distribution<double> dist_z(z_center - z_band_,
                                                  z_center + z_band_);
    std::uniform_real_distribution<double> dist01(0.0, 1.0);

    const double rewire_radius = std::max(3.0, 2.0 * step_size_);
    int goal_index = -1;

    for (int iter = 0; iter < max_iterations_; ++iter) {
      const double elapsed_sec =
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        planning_start)
              .count();
      if (elapsed_sec > max_planning_time_sec_) {
        if (failure_reason) {
          *failure_reason = "planning time budget exceeded";
        }
        return false;
      }

      Eigen::Vector3d q_rand;
      // Occasionally sample the goal directly to bias towards it.
      if (dist01(rng) < 0.1) {
        q_rand = goal;
      } else {
        q_rand = Eigen::Vector3d(dist_x(rng), dist_y(rng), dist_z(rng));
      }

      int nearest_idx = findNearest(nodes, q_rand);
      Eigen::Vector3d q_new = steer(nodes[nearest_idx].pos, q_rand, step_size_);

      if (!isSegmentCollisionFree(nodes[nearest_idx].pos, q_new)) {
        continue;
      }

      // RRT* choose parent
      std::vector<int> neighbors = findNeighbors(nodes, q_new, rewire_radius);
      int best_parent = nearest_idx;
      double best_cost =
          nodes[nearest_idx].cost + (q_new - nodes[nearest_idx].pos).norm();

      for (int idx : neighbors) {
        double c = nodes[idx].cost + (q_new - nodes[idx].pos).norm();
        if (c + 1e-6 < best_cost &&
            isSegmentCollisionFree(nodes[idx].pos, q_new)) {
          best_parent = idx;
          best_cost = c;
        }
      }

      Node new_node;
      new_node.pos = q_new;
      new_node.parent = best_parent;
      new_node.cost = best_cost;
      nodes.push_back(new_node);
      int new_idx = static_cast<int>(nodes.size()) - 1;

      // RRT* rewire
      for (int idx : neighbors) {
        double c_through_new = best_cost + (nodes[idx].pos - q_new).norm();
        if (c_through_new + 1e-6 < nodes[idx].cost &&
            isSegmentCollisionFree(q_new, nodes[idx].pos)) {
          nodes[idx].parent = new_idx;
          nodes[idx].cost = c_through_new;
        }
      }

      // Goal check
      if ((q_new - goal).norm() < goal_radius_ &&
          isSegmentCollisionFree(q_new, goal)) {
        Node goal_node;
        goal_node.pos = goal;
        goal_node.parent = new_idx;
        goal_node.cost = best_cost + (goal - q_new).norm();
        nodes.push_back(goal_node);
        goal_index = static_cast<int>(nodes.size()) - 1;
        break;
      }
    }

    if (goal_index < 0) {
      // Pick the node closest to goal as fallback.
      goal_index = findNearest(nodes, goal);
    }

    // Reconstruct path: collect nodes from goal back to start.
    std::vector<Eigen::Vector3d> waypoints_raw;
    for (int idx = goal_index; idx >= 0; idx = nodes[idx].parent) {
      waypoints_raw.push_back(nodes[idx].pos);
      if (nodes[idx].parent == -1) {
        break;
      }
    }
    if (waypoints_raw.size() < 2) {
      if (failure_reason) {
        *failure_reason = "insufficient waypoints after path reconstruction";
      }
      return false;
    }
    std::reverse(waypoints_raw.begin(), waypoints_raw.end());

    // Post-process waypoints to avoid zero-length segments that break
    // the trajectory optimizer (segment_time == 0 assertions).
    const double kMinSegmentDist = std::max(0.5 * map_resolution_, 0.05);
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(waypoints_raw.size());
    Eigen::Vector3d last_p = waypoints_raw.front();
    waypoints.push_back(last_p);
    for (size_t i = 1; i < waypoints_raw.size(); ++i) {
      const Eigen::Vector3d &p = waypoints_raw[i];
      if ((p - last_p).norm() < kMinSegmentDist) {
        continue; // skip near-duplicate point
      }
      waypoints.push_back(p);
      last_p = p;
    }
    if (waypoints.size() < 2) {
      if (failure_reason) {
        *failure_reason = "all waypoints collapsed after filtering";
      }
      return false;
    }

    out_path.poses.clear();
    out_path.poses.reserve(waypoints.size());
    for (const auto &p : waypoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();
      pose.pose.orientation.w = 1.0;
      out_path.poses.push_back(pose);
    }

    return true;
  }

  int findNearest(const std::vector<Node> &nodes,
                  const Eigen::Vector3d &q) const {
    int best_idx = 0;
    double best_dist2 = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
      double d2 = (nodes[i].pos - q).squaredNorm();
      if (d2 < best_dist2) {
        best_dist2 = d2;
        best_idx = i;
      }
    }
    return best_idx;
  }

  std::vector<int> findNeighbors(const std::vector<Node> &nodes,
                                 const Eigen::Vector3d &q,
                                 double radius) const {
    std::vector<int> neighbors;
    const double r2 = radius * radius;
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
      double d2 = (nodes[i].pos - q).squaredNorm();
      if (d2 < r2) {
        neighbors.push_back(i);
      }
    }
    return neighbors;
  }

  Eigen::Vector3d steer(const Eigen::Vector3d &from, const Eigen::Vector3d &to,
                        double step) const {
    Eigen::Vector3d dir = to - from;
    double dist = dir.norm();
    if (dist <= step) {
      return to;
    }
    return from + (step / dist) * dir;
  }

  bool isStateFree(const Eigen::Vector3d &p) const {
    if (!octree_) {
      return false;
    }

    octomap::point3d query(static_cast<float>(p.x()), static_cast<float>(p.y()),
                           static_cast<float>(p.z()));
    octomap::OcTreeNode *node = octree_->search(query);
    if (!node) {
      // Unknown space: for exploration, allow stepping into unknown as long as
      // we don't see any occupied voxels within the inflated robot radius
      // below. So we do NOT early-reject here.
    } else {
      const double occ_prob = node->getOccupancy();
      const double occ_thresh = octree_->getOccupancyThres();
      if (occ_prob >= occ_thresh) {
        return false;
      }
    }
    const double occ_thresh = octree_->getOccupancyThres();

    // Simple inflation by checking neighboring samples within robot_radius_.
    const double r = robot_radius_;
    // Use finer sampling for collision checking to avoid missing thin obstacles
    // Step size should be at most half the robot radius to ensure coverage
    // and explicitly controlled by collision_check_resolution_
    const double step = std::min(collision_check_resolution_,
                                 std::min(robot_radius_ * 0.3, 0.1));
    for (double dx = -r; dx <= r; dx += step) {
      for (double dy = -r; dy <= r; dy += step) {
        for (double dz = -r; dz <= r; dz += step) {
          if (dx * dx + dy * dy + dz * dz > r * r) {
            continue;
          }
          octomap::point3d q(static_cast<float>(p.x() + dx),
                             static_cast<float>(p.y() + dy),
                             static_cast<float>(p.z() + dz));
          octomap::OcTreeNode *n = octree_->search(q);
          if (!n) {
            // Unknown neighbor voxel: treat as free for exploration.
            continue;
          }
          if (n->getOccupancy() >= occ_thresh) {
            return false;
          }
        }
      }
    }
    return true;
  }

  bool isSegmentCollisionFree(const Eigen::Vector3d &from,
                              const Eigen::Vector3d &to) const {
    const double dist = (to - from).norm();
    if (dist < 1e-3) {
      return isStateFree(from);
    }
    const double step = std::min(collision_check_resolution_,
                                 std::min(robot_radius_ * 0.5, 0.1));
    const int steps = static_cast<int>(std::ceil(dist / step));
    for (int i = 0; i <= steps; ++i) {
      double t = static_cast<double>(i) / static_cast<double>(steps);
      Eigen::Vector3d p = (1.0 - t) * from + t * to;
      if (!isStateFree(p)) {
        return false;
      }
    }
    return true;
  }

  bool findNearbyFreeStart(const Eigen::Vector3d &start,
                           Eigen::Vector3d &free_start) const {
    if (!octree_) {
      return false;
    }

    // Try concentric shells; prioritize descending offsets first, since
    // ceiling contacts are a common failure mode in this mission.
    const std::vector<double> search_radii = {0.3, 0.5, 0.8, 1.2, 1.8, 2.5};
    std::vector<double> offsets;
    offsets.reserve(9);
    offsets.push_back(0.0);
    for (int i = 1; i <= 4; ++i) {
      const double d = static_cast<double>(i) * 0.25;
      offsets.push_back(-d);
      offsets.push_back(d);
    }

    for (double radius : search_radii) {
      for (double dz_norm : offsets) {
        const double dz = dz_norm * radius;
        const double radial_xy = std::sqrt(std::max(0.0, radius * radius - dz * dz));

        const std::vector<Eigen::Vector2d> xy_samples = {
            {0.0, 0.0},          {radial_xy, 0.0},       {-radial_xy, 0.0},
            {0.0, radial_xy},    {0.0, -radial_xy},      {0.707 * radial_xy, 0.707 * radial_xy},
            {-0.707 * radial_xy, 0.707 * radial_xy},     {0.707 * radial_xy, -0.707 * radial_xy},
            {-0.707 * radial_xy, -0.707 * radial_xy}};

        for (const auto &xy : xy_samples) {
          Eigen::Vector3d candidate(start.x() + xy.x(), start.y() + xy.y(),
                                    start.z() + dz);
          if (isStateFree(candidate)) {
            free_start = candidate;
            return true;
          }
        }
      }
    }
    return false;
  }

  // Parameters
  double robot_radius_;
  double collision_check_resolution_;
  double local_box_size_;
  double z_band_;
  double step_size_;
  double goal_radius_;
  int max_iterations_;
  double max_planning_time_sec_;

  std::string path_topic_;
  std::string goal_topic_;
  std::string odom_topic_;
  std::string octomap_topic_;

  // State
  std::shared_ptr<octomap::OcTree> octree_;
  bool has_octomap_;
  bool has_odom_;
  double map_resolution_ = 0.2;
  Eigen::Vector3d current_pos_{0.0, 0.0, 0.0};

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      drone_radius_pub_;
  rclcpp::TimerBase::SharedPtr drone_radius_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
