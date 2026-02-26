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



private:
  void OnDetection(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    detections_.push_back(*msg);
    
    // Write immediately to file
    const auto output_file = this->get_parameter("output_file").as_string();
    std::filesystem::path output_path(output_file);
    
    try {
      if (output_path.has_parent_path() && !output_path.parent_path().empty()) {
        std::filesystem::create_directories(output_path.parent_path());
      }
      
      bool file_exists = std::filesystem::exists(output_path);
      // Open in append mode
      std::ofstream file(output_path, std::ios::app);
      if (!file.is_open()) {
        RCLCPP_WARN(get_logger(), "Failed to open lantern detections file: %s", output_file.c_str());
        return;
      }
      
      // Write header if file is new
      if (!file_exists || std::filesystem::file_size(output_path) == 0) {
        file << "frame_id,x,y,z\n";
      }
      
      const auto & position = msg->pose.position;
      file << msg->header.frame_id << ','
           << std::fixed << std::setprecision(3)
           << position.x << ',' << position.y << ',' << position.z << "\n";
           
      file.flush(); // Ensure it's written immediately
      RCLCPP_INFO(get_logger(), "Logged lantern detection to %s", output_file.c_str());
      
    } catch (const std::exception & exc) {
      RCLCPP_WARN(get_logger(), "Failed to write lantern detection: %s", exc.what());
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr detections_sub_;
  std::vector<geometry_msgs::msg::PoseStamped> detections_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LanternDetectionLoggerNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
