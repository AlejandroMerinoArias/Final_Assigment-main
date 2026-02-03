#include <chrono>
#include <memory>
#include <string>

#include "mav_msgs/conversions.hpp"
#include "mav_msgs/default_topics.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryGenerationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
