#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>

#define STATIC_POSE 0
#ifndef TFOUTPUT
#define TFOUTPUT 1
#endif

class TrajPublisher : public rclcpp::Node
{
public:
  TrajPublisher()
  : rclcpp::Node("traj_publisher"),
    start_(this->now())
  {
    pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
      "desired_state", rclcpp::QoS(1));

#if TFOUTPUT
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
#endif

    // 500 Hz timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2),
      std::bind(&TrajPublisher::tick, this));

    RCLCPP_INFO(this->get_logger(), "traj_publisher started (500 Hz).");
  }

private:
  void tick()
  {
    const double t = (this->now() - start_).seconds();

    // Desired pose (tf2 types for convenience)
    tf2::Transform desired_pose(tf2::Transform::getIdentity());
    geometry_msgs::msg::Twist velocity;
    geometry_msgs::msg::Twist acceleration;

#if STATIC_POSE
    // Example static pose (commented in original)
    desired_pose.setOrigin(tf2::Vector3(0.0, 0.0, 2.0));
    tf2::Quaternion q; q.setRPY(0, 0, M_PI/4.0);
    desired_pose.setRotation(q);
#else
    // Circle in XY plane at z=2
    const double R = 5.0;
    const double timeScale = 2.0;

    desired_pose.setOrigin(tf2::Vector3(
      R * std::sin(t / timeScale),
      R * std::cos(t / timeScale),
      2.0));

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, -t / timeScale);
    desired_pose.setRotation(q);

    // Linear velocity in world frame
    velocity.linear.x =  R * std::cos(t / timeScale) / timeScale;
    velocity.linear.y = -R * std::sin(t / timeScale) / timeScale;
    velocity.linear.z = 0.0;

    // Angular velocity (yaw rate)
    velocity.angular.x = 0.0;
    velocity.angular.y = 0.0;
    velocity.angular.z = -1.0 / timeScale;

    // Linear acceleration
    const double inv_ts2 = 1.0 / (timeScale * timeScale);
    acceleration.linear.x = -R * std::sin(t / timeScale) * inv_ts2;
    acceleration.linear.y = -R * std::cos(t / timeScale) * inv_ts2;
    acceleration.linear.z = 0.0;

    acceleration.angular.x = 0.0;
    acceleration.angular.y = 0.0;
    acceleration.angular.z = 0.0;
#endif

    // Publish MultiDOFJointTrajectoryPoint
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint msg;
    msg.transforms.resize(1);
    msg.velocities.resize(1);
    msg.accelerations.resize(1);

    // Transform
    msg.transforms[0].translation.x = desired_pose.getOrigin().x();
    msg.transforms[0].translation.y = desired_pose.getOrigin().y();
    msg.transforms[0].translation.z = desired_pose.getOrigin().z();
    msg.transforms[0].rotation.x    = desired_pose.getRotation().x();
    msg.transforms[0].rotation.y    = desired_pose.getRotation().y();
    msg.transforms[0].rotation.z    = desired_pose.getRotation().z();
    msg.transforms[0].rotation.w    = desired_pose.getRotation().w();

    // Twist & acceleration
    msg.velocities[0]     = velocity;
    msg.accelerations[0]  = acceleration;

    pub_->publish(msg);

    // Log (optional)
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000 /*ms*/,
      "Trajectory Position x:%.3f y:%.3f z:%.3f",
      msg.transforms[0].translation.x,
      msg.transforms[0].translation.y,
      msg.transforms[0].translation.z);

#if TFOUTPUT
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp    = this->now();
    ts.header.frame_id = "world";
    ts.child_frame_id  = "av-desired";
    ts.transform.translation.x = msg.transforms[0].translation.x;
    ts.transform.translation.y = msg.transforms[0].translation.y;
    ts.transform.translation.z = msg.transforms[0].translation.z;
    ts.transform.rotation      = msg.transforms[0].rotation;
    tf_broadcaster_->sendTransform(ts);
#endif
  }

  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_;

#if TFOUTPUT
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
#endif
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajPublisher>());
  rclcpp::shutdown();
  return 0;
}
