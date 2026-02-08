#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "mav_msgs/conversions.hpp"
#include "mav_msgs/default_topics.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "mav_trajectory_generation/trajectory_sampling.h"
#include "trajectory_generation/planner.hpp"

using namespace std::chrono_literals;

class TrajectoryGenerationNode : public rclcpp::Node
{
public:
  TrajectoryGenerationNode()
  : rclcpp::Node("trajectory_generation_node"),
    publish_whole_trajectory_(false),
    dt_(0.01),
    current_sample_time_(0.0),
    marker_distance_(0.2),
    command_topic_(mav_msgs::default_topics::COMMAND_TRAJECTORY),
    path_topic_("waypoints")
  {
    publish_whole_trajectory_ =
      this->declare_parameter<bool>("publish_whole_trajectory", publish_whole_trajectory_);
    dt_ = this->declare_parameter<double>("dt", dt_);
    marker_distance_ = this->declare_parameter<double>("marker_distance", marker_distance_);
    command_topic_ = this->declare_parameter<std::string>("command_topic", command_topic_);
    path_topic_ = this->declare_parameter<std::string>("path_topic", path_topic_);

    planner_ = std::make_unique<TrajectoryPlanner>(*this);

    command_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      command_topic_, 1);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "trajectory_markers", 10);

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10,
      std::bind(&TrajectoryGenerationNode::pathCallback, this, std::placeholders::_1));

    publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&TrajectoryGenerationNode::commandTimerCallback, this));
    publish_timer_->cancel();
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr path)
  {
    if (!path || path->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty path message.");
      return;
    }

    if (!planner_->hasOdom()) {
      RCLCPP_WARN(get_logger(), "No odometry received yet, skipping trajectory planning.");
      return;
    }

    mav_trajectory_generation::Trajectory trajectory;
    if (!planner_->planTrajectoryFromPath(*path, &trajectory)) {
      RCLCPP_WARN(get_logger(), "Failed to plan trajectory from path.");
      return;
    }

    trajectory_ = trajectory;
    last_frame_id_ = path->header.frame_id.empty() ? "world" : path->header.frame_id;
    segment_times_ = trajectory_.getSegmentTimes();
    mav_msgs::EigenTrajectoryPoint start_point;
    bool has_start_point = mav_trajectory_generation::sampleTrajectoryAtTime(
      trajectory_, 0.0, &start_point);
    const double start_yaw = has_start_point ? start_point.getYaw() : 0.0;
    segment_yaws_ = computeSegmentYaws(*path, segment_times_.size(), has_start_point,
      start_point.position_W, start_yaw);

    visualization_msgs::msg::MarkerArray markers;
    planner_->drawMavTrajectory(trajectory_, marker_distance_, last_frame_id_, &markers);
    marker_pub_->publish(markers);

    processTrajectory();
  }

  void processTrajectory()
  {
    if (publish_whole_trajectory_) {
      mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
      mav_trajectory_generation::sampleWholeTrajectory(
        trajectory_, dt_, &trajectory_points);
      for (size_t i = 0; i < trajectory_points.size(); ++i) {
        const double sample_time = static_cast<double>(i) * dt_;
        trajectory_points[i].setFromYaw(yawForTime(sample_time));
      }

      trajectory_msgs::msg::MultiDOFJointTrajectory msg_pub;
      mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_points, &msg_pub);
      msg_pub.header.stamp = this->now();
      msg_pub.header.frame_id = last_frame_id_;
      command_pub_->publish(msg_pub);
    } else {
      current_sample_time_ = 0.0;
      publish_timer_->reset();
    }
  }

  void commandTimerCallback()
  {
    if (current_sample_time_ <= trajectory_.getMaxTime()) {
      trajectory_msgs::msg::MultiDOFJointTrajectory msg;
      mav_msgs::EigenTrajectoryPoint trajectory_point;

      bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_, current_sample_time_, &trajectory_point);
      if (!success) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to sample trajectory at time %f, stopping timer.", current_sample_time_);
        publish_timer_->cancel();
        return;
      }

      trajectory_point.setFromYaw(yawForTime(current_sample_time_));
      mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);

      if (!msg.points.empty()) {
        auto & tfs = msg.points[0].time_from_start;
        const double t = current_sample_time_;
        tfs.sec = static_cast<int32_t>(t);
        tfs.nanosec = static_cast<uint32_t>((t - tfs.sec) * 1e9);
      }
      msg.header.frame_id = last_frame_id_;

      command_pub_->publish(msg);
      current_sample_time_ += dt_;
    } else {
      publish_timer_->cancel();
      RCLCPP_INFO(get_logger(), "Finished streaming trajectory.");
    }
  }

  std::vector<double> computeWaypointYaws(
    const nav_msgs::msg::Path & path,
    bool has_start_position,
    const Eigen::Vector3d & start_position,
    double start_yaw) const
  {
    const size_t waypoint_count = path.poses.size();
    std::vector<double> yaws;
    if (waypoint_count == 0) {
      return yaws;
    }

    yaws.reserve(waypoint_count);
    if (waypoint_count == 1) {
      if (has_start_position) {
        const auto & waypoint = path.poses.front().pose.position;
        const double dx = waypoint.x - start_position.x();
        const double dy = waypoint.y - start_position.y();
        if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
          yaws.push_back(start_yaw);
        } else {
          yaws.push_back(std::atan2(dy, dx));
        }
      } else {
        yaws.push_back(0.0);
      }
      return yaws;
    }

    for (size_t i = 0; i + 1 < waypoint_count; ++i) {
      const auto & current = path.poses[i].pose.position;
      const auto & next = path.poses[i + 1].pose.position;
      const double dx = next.x - current.x;
      const double dy = next.y - current.y;
      if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
        if (!yaws.empty()) {
          yaws.push_back(yaws.back());
        } else if (has_start_position) {
          yaws.push_back(start_yaw);
        } else {
          yaws.push_back(0.0);
        }
      } else {
        yaws.push_back(std::atan2(dy, dx));
      }
    }

    yaws.push_back(yaws.back());
    return yaws;
  }

  std::vector<double> computeSegmentYaws(
    const nav_msgs::msg::Path & path,
    size_t segment_count,
    bool has_start_position,
    const Eigen::Vector3d & start_position,
    double start_yaw) const
  {
    std::vector<double> segment_yaws;
    segment_yaws.reserve(segment_count);
    const std::vector<double> waypoint_yaws = computeWaypointYaws(
      path, has_start_position, start_position, start_yaw);
    for (size_t i = 0; i < segment_count; ++i) {
      if (waypoint_yaws.empty()) {
        segment_yaws.push_back(0.0);
        continue;
      }

      if (i == 0) {
        segment_yaws.push_back(waypoint_yaws.front());
        continue;
      }

      const bool is_last_segment = (i == segment_count - 1);
      const size_t waypoint_index =
        is_last_segment ? waypoint_yaws.size() - 1 : std::min(i - 1, waypoint_yaws.size() - 1);
      segment_yaws.push_back(waypoint_yaws[waypoint_index]);
    }

    return segment_yaws;
  }

  double yawForTime(double sample_time) const
  {
    if (segment_times_.empty() || segment_yaws_.empty()) {
      return 0.0;
    }

    double accumulated_time = 0.0;
    for (size_t i = 0; i < segment_times_.size(); ++i) {
      accumulated_time += segment_times_[i];
      if (sample_time <= accumulated_time || i == segment_times_.size() - 1) {
        const size_t yaw_index = std::min(i, segment_yaws_.size() - 1);
        return segment_yaws_[yaw_index];
      }
    }

    return segment_yaws_.back();
  }

  std::unique_ptr<TrajectoryPlanner> planner_;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr command_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  bool publish_whole_trajectory_;
  double dt_;
  double current_sample_time_;
  double marker_distance_;
  std::string command_topic_;
  std::string path_topic_;
  std::string last_frame_id_;

  mav_trajectory_generation::Trajectory trajectory_;
  std::vector<double> segment_times_;
  std::vector<double> segment_yaws_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryGenerationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}