#include <rclcpp/rclcpp.hpp>
#include "FSM/mission_fsm_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<control::MissionFsmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
