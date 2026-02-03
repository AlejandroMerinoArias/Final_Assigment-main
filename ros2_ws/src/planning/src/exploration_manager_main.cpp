#include "planning/exploration_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<planning::ExplorationManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
