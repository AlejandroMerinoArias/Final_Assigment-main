#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

/// Simple relay node that gates point cloud data.
/// Only forwards /camera/depth/points → /gated_cloud when enabled.
/// The FSM publishes true to /enable_mapping when entering EXPLORE state.
class CloudGateNode : public rclcpp::Node {
public:
  CloudGateNode() : Node("cloud_gate"), enabled_(false) {
    // Subscribe to enable signal
    enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/enable_mapping", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data && !enabled_) {
            RCLCPP_INFO(this->get_logger(), "Mapping ENABLED — forwarding point clouds.");
          }
          enabled_ = msg->data;
        });

    // Subscribe to raw point cloud (best_effort QoS to match camera)
    auto qos = rclcpp::QoS(10);
    qos.best_effort();
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/points", qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          if (enabled_) {
            cloud_pub_->publish(*msg);
          }
        });

    // Publisher for gated output
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/gated_cloud", 10);

    RCLCPP_INFO(this->get_logger(),
                "Cloud gate node started. Waiting for /enable_mapping...");
  }

private:
  bool enabled_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudGateNode>());
  rclcpp::shutdown();
  return 0;
}
