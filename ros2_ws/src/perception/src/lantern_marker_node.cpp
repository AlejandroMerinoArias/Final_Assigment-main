#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class LanternMarkerNode : public rclcpp::Node {
public:
  LanternMarkerNode()
  : Node("lantern_marker"), marker_id_(0) {
    declare_parameter<std::string>("detections_topic", "/detected_lanterns");
    declare_parameter<std::string>("marker_topic", "/lantern_marker");
    declare_parameter<double>("shaft_length", 0.6);
    declare_parameter<double>("shaft_diameter", 0.08);
    declare_parameter<double>("head_diameter", 0.14);

    const auto detections_topic = get_parameter("detections_topic").as_string();
    const auto marker_topic = get_parameter("marker_topic").as_string();

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10);
    detection_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      detections_topic, 10,
      std::bind(&LanternMarkerNode::OnDetection, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Lantern marker node listening on %s", detections_topic.c_str());
  }

private:
  void OnDetection(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    visualization_msgs::msg::Marker marker;
    marker.header = msg->header;
    marker.ns = "lanterns";
    marker.id = marker_id_++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = msg->pose;

    if (marker.pose.orientation.x == 0.0 && marker.pose.orientation.y == 0.0 &&
        marker.pose.orientation.z == 0.0 && marker.pose.orientation.w == 0.0) {
      marker.pose.orientation.w = 1.0;
    }

    marker.scale.x = get_parameter("shaft_length").as_double();
    marker.scale.y = get_parameter("shaft_diameter").as_double();
    marker.scale.z = get_parameter("head_diameter").as_double();

    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker_pub_->publish(marker);
  }

  size_t marker_id_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr detection_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LanternMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
