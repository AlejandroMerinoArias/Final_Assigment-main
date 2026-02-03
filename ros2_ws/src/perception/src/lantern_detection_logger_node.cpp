#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class LanternDetectionLoggerNode : public rclcpp::Node {
public:
  LanternDetectionLoggerNode() : Node("lantern_detection_logger") {
    declare_parameter<std::string>("detections_topic", "/detected_lanterns");
    declare_parameter<std::string>("output_file", "lantern_detections.txt");

    const auto detections_topic = get_parameter("detections_topic").as_string();
    detections_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      detections_topic, 10,
      std::bind(&LanternDetectionLoggerNode::OnDetection, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Lantern detection logger initialized");
  }

  void WriteDetectionsToFile() {
    const auto output_file = get_parameter("output_file").as_string();
    std::filesystem::path output_path(output_file);
    std::vector<std::string> lines;
    lines.emplace_back("frame_id,x,y,z");
    lines.reserve(detections_.size() + 1);
    for (const auto & detection : detections_) {
      const auto & position = detection.pose.position;
      std::ostringstream line;
      line << detection.header.frame_id << ','
           << std::fixed << std::setprecision(3)
           << position.x << ',' << position.y << ',' << position.z;
      lines.push_back(line.str());
    }

    try {
      if (output_path.has_parent_path() && !output_path.parent_path().empty()) {
        std::filesystem::create_directories(output_path.parent_path());
      }
      std::ofstream file(output_path);
      if (!file.is_open()) {
        RCLCPP_WARN(get_logger(), "Failed to open lantern detections file: %s", output_file.c_str());
        return;
      }
      for (const auto & line : lines) {
        file << line << '\n';
      }
      RCLCPP_INFO(get_logger(), "Wrote lantern detections to %s", output_file.c_str());
    } catch (const std::exception & exc) {
      RCLCPP_WARN(get_logger(), "Failed to write lantern detections: %s", exc.what());
    }
  }

private:
  void OnDetection(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    detections_.push_back(*msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr detections_sub_;
  std::vector<geometry_msgs::msg::PoseStamped> detections_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LanternDetectionLoggerNode>();
  rclcpp::spin(node);
  node->WriteDetectionsToFile();
  rclcpp::shutdown();
  return 0;
}
