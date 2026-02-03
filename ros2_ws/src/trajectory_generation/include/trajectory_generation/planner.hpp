#ifndef TRAJECTORY_GENERATION__PLANNER_HPP_
#define TRAJECTORY_GENERATION__PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "mav_msgs/conversions.hpp"
#include <tf2_eigen/tf2_eigen.hpp>


#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include "mav_trajectory_generation/trajectory_sampling.h"
#include "mav_trajectory_generation/ros_conversions.h"

class TrajectoryPlanner
{
public:
  explicit TrajectoryPlanner(rclcpp::Node & node);

  void uavOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
  bool hasOdom() const;

  void setMaxSpeed(double max_v);

  bool planTrajectoryFromPath(
    const nav_msgs::msg::Path & path,
    mav_trajectory_generation::Trajectory * trajectory);

  void drawMavTrajectory(
    const mav_trajectory_generation::Trajectory & trajectory,
    double distance, const std::string & frame_id,
    visualization_msgs::msg::MarkerArray * marker_array);

private:
  bool loadWaypointConstraints(
    const nav_msgs::msg::Path & path,
    std::vector<Eigen::Vector3d> * velocities,
    std::vector<Eigen::Vector3d> * accelerations) const;

  rclcpp::Node * node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  Eigen::Affine3d current_pose_;
  Eigen::Vector3d current_velocity_;
  Eigen::Vector3d current_angular_velocity_;
  bool odom_received_;

  double max_v_;      // m/s
  double max_a_;      // m/s^2
  double max_ang_v_;
  double max_ang_a_;

  std::vector<double> waypoint_velocities_;
  std::vector<double> waypoint_accelerations_;
  std::vector<double> default_waypoint_velocity_;
  std::vector<double> default_waypoint_acceleration_;
};

#endif  // TRAJECTORY_GENERATION__PLANNER_HPP_
