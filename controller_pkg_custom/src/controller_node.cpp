#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <mav_msgs/msg/actuators.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>


#define PI M_PI

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  PART 0 |  Autonomous Systems - Fall 2025  - Lab 2 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to implement a geometric controller for a
//  simulated UAV, following the publication:
//
//  [1] Lee, Taeyoung, Melvin Leoky, N. Harris McClamroch. "Geometric tracking
//      control of a quadrotor UAV on SE (3)." Decision and Control (CDC),
//      49th IEEE Conference on. IEEE, 2010
//
//  We use variable names as close as possible to the conventions found in the
//  paper, however, we have slightly different conventions for the aerodynamic
//  coefficients of the propellers (refer to the lecture notes for these).
//  Additionally, watch out for the different conventions on reference frames
//  (see Lab Handout for more details).
//
//  Eigen is a C++ library for linear algebra that will help you significantly 
//  with the implementation. Check the reference page to learn the basics:
//
//  https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
//
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//                                 end part 0
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class ControllerNode : public rclcpp::Node {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  PART 1 |  Declare ROS callback handlers
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // In this section, you need to declare:
  //   1. two subscribers (for the desired and current UAVStates)
  //   2. one publisher (for the propeller speeds)
  //   3. a timer for your main control loop
  //
  // ~~~~ begin solution

  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr
      desired_state_sub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      current_state_sub_;

  rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr
      motor_pub_;

  rclcpp::TimerBase::SharedPtr
      control_timer_;

  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                                 end part 1
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Controller parameters
  double kx, kv, kr, komega; // controller gains - [1] eq (15), (16)

  // Physical constants (we will set them below)
  double m;              // mass of the UAV
  double g;              // gravity acceleration
  double d;              // distance from the center of propellers to the c.o.m.
  double cf,             // Propeller lift coefficient
         cd;             // Propeller drag coefficient
  Eigen::Matrix3d J;     // Inertia Matrix
  Eigen::Vector3d e3;    // [0,0,1]
  Eigen::MatrixXd F2W;   // Wrench-rotor speeds map

  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle

  double hz;             // frequency of the main control loop

  static Eigen::Vector3d Vee(const Eigen::Matrix3d& in){
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

  static double signed_sqrt(double val){
    return val>0?sqrt(val):-sqrt(-val);
  }

public:
  ControllerNode()
  : rclcpp::Node("controller_node"),
    e3(0,0,1),
    F2W(4,4),
    hz(1000.0)
  {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 2 |  Initialize ROS callback handlers
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // In this section, you need to initialize your handlers from part 1.
    // Specifically:
    //  - bind controllerNode::onDesiredState() to the topic "command/trajectory"
    //  - bind controllerNode::onCurrentState() to the topic "current_state"
    //  - bind controllerNode::controlLoop() to the created timer, at frequency
    //    given by the "hz" variable
    //
    // Hints:
    //  - read the lab handout to find the message type
    //
    // ~~~~ begin solution
    // Subscribers
    desired_state_sub_ =
        this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
            "command/trajectory",
            rclcpp::QoS(10),
            [this](trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg) {
              this->onDesiredState(msg);
            });

    current_state_sub_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "current_state",
            rclcpp::QoS(10),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
              this->onCurrentState(msg);
            });

    // Publisher (motor commands)
    motor_pub_ = this->create_publisher<mav_msgs::msg::Actuators>(
        "rotor_speed_cmds",
        rclcpp::QoS(10));

    // Main control loop timer
    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / hz),
        [this]() { this->controlLoop(); });
    // ~~~~ end solution
    //
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //                                 end part 2
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 6 [NOTE: save this for last] |  Tune your gains!
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Controller gains (move to parameters.yaml for convenience if you like)
    kx = 2.0;
    kv = 3.0;             //    **** FIDDLE WITH THESE! ***
    kr = 6.0;
    komega = 2.0;

    // Initialize constants
    m = 1.0;
    cd = 1e-5;
    cf = 1e-3;
    g = 9.81;
    d = 0.8;
    J << 1.0,0.0,0.0,
         0.0,1.0,0.0,
         0.0,0.0,1.0;

    RCLCPP_INFO(this->get_logger(), "controller_node ready (hz=%.1f)", hz);
  }

  void onDesiredState(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr des_state_msg){
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  PART 3 | Objective: fill in xd, vd, ad, yawd
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //
      // 3.1 Get the desired position, velocity and acceleration from the in-
      //     coming ROS message and fill in the class member variables xd, vd
      //     and ad accordingly. You can ignore the angular acceleration.
      //
      // Hint: use "v << vx, vy, vz;" to fill in a vector with Eigen.
      //
      // ~~~~ begin solution
      if (des_state_msg->points.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "DesiredState message missing trajectory points.");
        return;
      }

      const auto &point = des_state_msg->points.front();
      if (point.transforms.empty() ||
          point.velocities.empty() ||
          point.accelerations.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "DesiredState message missing transforms/velocities/accelerations.");
        return;
      }

      const auto &T = point.transforms[0];
      const auto &V = point.velocities[0];
      const auto &A = point.accelerations[0];

      xd << T.translation.x, T.translation.y, T.translation.z;

      vd << V.linear.x, V.linear.y, V.linear.z;

      ad << A.linear.x, A.linear.y, A.linear.z;
      // ~~~~ end solution
      //
      // 3.2 Extract the yaw component from the quaternion in the incoming ROS
      //     message and store in the yawd class member variable
      //
      //  Hints:
      //    - use tf2::getYaw(des_state_msg->transforms[0].rotation)
      //
      // ~~~~ begin solution
      yawd = tf2::getYaw(T.rotation);
      // ~~~~ end solution
  }

  void onCurrentState(const nav_msgs::msg::Odometry::SharedPtr cur_state_msg){
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  PART 4 | Objective: fill in x, v, R and omega
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //
      // Get the current position and velocity from the incoming ROS message and
      // fill in the class member variables x, v, R and omega accordingly.
      //
      //  CAVEAT: cur_state.twist.twist.angular is in the world frame, while omega
      //          needs to be in the body frame!
      //
      // ~~~~ begin solution
      // Position (world)
      x << cur_state_msg->pose.pose.position.x,
           cur_state_msg->pose.pose.position.y,
           cur_state_msg->pose.pose.position.z;

      // Linear velocity (world)
      v << cur_state_msg->twist.twist.linear.x,
           cur_state_msg->twist.twist.linear.y,
           cur_state_msg->twist.twist.linear.z;

      // Orientation: quaternion -> rotation matrix (body -> world)
      const auto &q_msg = cur_state_msg->pose.pose.orientation;
      Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
      q.normalize();
      R = q.toRotationMatrix();

      // Angular velocity: message is in world frame, controller needs body frame
      Eigen::Vector3d omega_world;
      omega_world << cur_state_msg->twist.twist.angular.x,
                     cur_state_msg->twist.twist.angular.y,
                     cur_state_msg->twist.twist.angular.z;

      omega = R.transpose() * omega_world;
      // ~~~~ end solution
  }

  void controlLoop(){
    Eigen::Vector3d ex, ev, er, eomega;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 5 | Objective: Implement the controller!
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //
    // 5.1 Compute position and velocity errors. Objective: fill in ex, ev.
    //  Hint: [1], eq. (6), (7)
    //
    // ~~~~ begin solution
    ex = x - xd;
    ev = v - vd;
    // ~~~~ end solution

    // 5.2 Compute the Rd matrix.
    //
    //  Hint: break it down in 3 parts:
    //    - b3d vector = z-body axis of the quadrotor, [1] eq. (12)
    //    - check out [1] fig. (3) for the remaining axes [use cross product]
    //    - assemble the Rd matrix, eigen offers: "MATRIX << col1, col2, col3"
    //
    //  CAVEATS:
    //    - Compare the reference frames in the Lab handout with Fig. 1 in the
    //      paper. The z-axes are flipped, which affects the signs of:
    //         i) the gravity term and
    //        ii) the overall sign (in front of the fraction) in equation (12)
    //            of the paper
    //    - remember to normalize your axes!
    //
    // ~~~~ begin solution
    Eigen::Matrix3d Rd;
    Eigen::Vector3d F_des = -kx * ex - kv * ev + m * g * e3 + m * ad;
    Eigen::Vector3d b3d;
    const double Fnorm = F_des.norm();
    if (Fnorm > 1e-6) {
      b3d = F_des / Fnorm;
    } else {
      b3d = e3;
    }

    Eigen::Vector3d b1d;
    b1d << std::cos(yawd), std::sin(yawd), 0.0;

    Eigen::Vector3d b2d = b3d.cross(b1d);
    if (b2d.norm() < 1e-6) {
      b1d << 1.0, 0.0, 0.0;
      b2d = b3d.cross(b1d);
    }
    b2d.normalize();

    Eigen::Vector3d b1c = b2d.cross(b3d);
    b1c.normalize();

    Rd.col(0) = b1c;
    Rd.col(1) = b2d;
    Rd.col(2) = b3d;
    // ~~~~ end solution
    //
    // 5.3 Compute the orientation error (er) and the rotation-rate error (eomega)
    //  Hints:
    //     - [1] eq. (10) and (11)
    //     - you can use the Vee() static method implemented above
    //
    //  CAVEATS: feel free to ignore the second addend in eq (11), since it
    //        requires numerical differentiation of Rd and it has negligible
    //        effects on the closed-loop dynamics.
    //
    // ~~~~ begin solution
    er = 0.5 * Vee(Rd.transpose() * R - R.transpose() * Rd);
    eomega = omega;
    // ~~~~ end solution
    //
    // 5.4 Compute the desired wrench (force + torques) to control the UAV.
    //  Hints:
    //     - [1] eq. (15), (16)
    //
    // CAVEATS:
    //    - Compare the reference frames in the Lab handout with Fig. 1 in the
    //      paper. The z-axes are flipped, which affects the signs of:
    //         i) the gravity term
    //        ii) the overall sign (in front of the bracket) in equation (15)
    //            of the paper
    //
    //    - feel free to ignore all the terms involving \Omega_d and its time
    //      derivative as they are of the second order and have negligible
    //      effects on the closed-loop dynamics.
    //
    // ~~~~ begin solution
    const double f = F_des.dot(R * e3);

    Eigen::Vector3d M;
    M = -kr * er - komega * eomega + omega.cross(J * omega);
    // ~~~~ end solution
    //
    // 5.5 Recover the rotor speeds from the wrench computed above
    //
    //  Hints:
    //     - [1] eq. (1)
    //
    // CAVEATs:
    //     - we have different conventions for the arodynamic coefficients,
    //       Namely: C_{\tau f} = c_d / c_f
    //               (LHS paper [1], RHS our conventions [lecture notes])
    //
    //     - Compare the reference frames in the Lab handout with Fig. 1 in the
    //       paper. In the paper [1], the x-body axis [b1] is aligned with a
    //       quadrotor arm, whereas for us, it is 45° from it (i.e., "halfway"
    //       between b1 and b2). To resolve this, check out equation 6.9 in the
    //       lecture notes!
    //
    //     - The thrust forces are **in absolute value** proportional to the
    //       square of the propeller speeds. Negative propeller speeds - although
    //       uncommon - should be a possible outcome of the controller when
    //       appropriate. Note that this is the case in unity but not in real
    //       life, where propellers are aerodynamically optimized to spin in one
    //       direction!
    //
    // ~~~~ begin solution
    const double l = d / std::sqrt(2.0);

    F2W.resize(4,4);
    F2W <<
      cf,      cf,      cf,      cf,
      cf*l,    cf*l,   -cf*l,   -cf*l,
     -cf*l,    cf*l,    cf*l,   -cf*l,
      cd,     -cd,      cd,     -cd;

    Eigen::Vector4d W;
    W << f, M(0), M(1), M(2);

    Eigen::Vector4d u = F2W.fullPivLu().solve(W);
    // ~~~~ end solution

    // 5.6 Populate and publish the control message
    //
    // Hint: do not forget that the propeller speeds are signed (maybe you want
    // to use signed_sqrt function).
    //
    // ~~~~ begin solution
    mav_msgs::msg::Actuators cmd;
    cmd.angular_velocities.resize(4);
    cmd.angular_velocities[0] = signed_sqrt(u(0));
    cmd.angular_velocities[1] = signed_sqrt(u(1));
    cmd.angular_velocities[2] = signed_sqrt(u(2));
    cmd.angular_velocities[3] = signed_sqrt(u(3));
    motor_pub_->publish(cmd);
    // ~~~~ end solution

    // Example publish skeleton (keep after you compute rotor speeds):
    // mav_msgs::msg::Actuators cmd;
    // cmd.angular_velocities.resize(4);
    // cmd.angular_velocities[0] = /* w1 */;
    // cmd.angular_velocities[1] = /* w2 */;
    // cmd.angular_velocities[2] = /* w3 */;
    // cmd.angular_velocities[3] = /* w4 */;
    // motor_pub_->publish(cmd);
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}