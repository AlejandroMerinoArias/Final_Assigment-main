#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class CurrentStateRelayNode : public rclcpp::Node {
 public:
  CurrentStateRelayNode() : Node("current_state_relay_node") {
    current_state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("current_state", 1);
    current_state_est_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/current_state_est", 1,
        std::bind(&CurrentStateRelayNode::OnCurrentStateEstimate, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Current state relay node ready");
  }

 private:
  void OnCurrentStateEstimate(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_state_pub_->publish(*msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_state_est_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr current_state_pub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CurrentStateRelayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}