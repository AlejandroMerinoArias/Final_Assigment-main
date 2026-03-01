#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/time.h"

#include <opencv2/imgproc.hpp>

namespace {
struct Track {
  std::array<double, 3> position{};
  int observations = 0;
  bool published = false;
};

std::optional<std::pair<int, int>> ComputeCentroid(const std::vector<cv::Point> & contour) {
  cv::Moments moments = cv::moments(contour);
  if (moments.m00 == 0.0) {
    return std::nullopt;
  }
  int u = static_cast<int>(moments.m10 / moments.m00);
  int v = static_cast<int>(moments.m01 / moments.m00);
  return std::make_pair(u, v);
}

std::optional<double> MedianOf(std::vector<float> values) {
  if (values.empty()) {
    return std::nullopt;
  }
  const size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + mid, values.end());
  if (values.size() % 2 == 1) {
    return static_cast<double>(values[mid]);
  }
  const float upper = values[mid];
  std::nth_element(values.begin(), values.begin() + mid - 1, values.end());
  const float lower = values[mid - 1];
  return 0.5 * (static_cast<double>(lower) + static_cast<double>(upper));
}

std::optional<double> DepthAtPixel(const cv::Mat & depth, int u, int v) {
  if (u < 0 || v < 0 || u >= depth.cols || v >= depth.rows) {
    return std::nullopt;
  }
  switch (depth.type()) {
    case CV_16U: {
      const uint16_t value = depth.at<uint16_t>(v, u);
      if (value == 0) {
        return std::nullopt;
      }
      return static_cast<double>(value) / 1000.0;
    }
    case CV_32F: {
      const float value = depth.at<float>(v, u);
      if (!std::isfinite(value) || value <= 0.0f) {
        return std::nullopt;
      }
      return static_cast<double>(value);
    }
    case CV_64F: {
      const double value = depth.at<double>(v, u);
      if (!std::isfinite(value) || value <= 0.0) {
        return std::nullopt;
      }
      return value;
    }
    default:
      return std::nullopt;
  }
}

std::string ToLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}
}  // namespace

class LanternDetectorNode : public rclcpp::Node {
public:
  LanternDetectorNode()
  : Node("lantern_detector") {
    declare_parameter<std::string>("semantic_topic", "/realsense/semantic/image_rect_raw");
    declare_parameter<std::string>("depth_topic", "/realsense/depth/image");
    declare_parameter<std::string>("camera_info_topic", "/realsense/semantic/camera_info");
    declare_parameter<std::string>("world_frame", "world");
    declare_parameter<std::string>("body_frame", "body");
    declare_parameter<std::vector<double>>("camera_offset", {0.1, 0.0, 0.0});
    declare_parameter<double>("min_area", 5.0);
    declare_parameter<double>("depth_window_scale", 0.5);
    declare_parameter<std::vector<int64_t>>("hsv_lower", {20, 70, 70});
    declare_parameter<std::vector<int64_t>>("hsv_upper", {70, 255, 255});
    declare_parameter<double>("gating_distance", 5.0);
    declare_parameter<int>("min_observations", 10);
    declare_parameter<double>("tf_timeout_s", 0.2);
    declare_parameter<bool>("use_latest_tf_on_extrapolation", true);

    detections_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/detected_lanterns", 10);

    const auto semantic_topic = get_parameter("semantic_topic").as_string();
    const auto depth_topic = get_parameter("depth_topic").as_string();
    const auto camera_info_topic = get_parameter("camera_info_topic").as_string();

    semantic_sub_.subscribe(this, semantic_topic);
    depth_sub_.subscribe(this, depth_topic);
    info_sub_.subscribe(this, camera_info_topic);

    sync_ = std::make_shared<Sync>(SyncPolicy(10), semantic_sub_, depth_sub_, info_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
    sync_->registerCallback(std::bind(
      &LanternDetectorNode::OnImages, this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "Lantern detector initialized");
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  void OnImages(
    const sensor_msgs::msg::Image::ConstSharedPtr & semantic_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg) {
    camera_info_ = info_msg;

    cv::Mat semantic;
    cv::Mat depth;
    try {
      semantic = cv_bridge::toCvShare(semantic_msg, "bgr8")->image;
      depth = cv_bridge::toCvShare(depth_msg)->image;
    } catch (const cv_bridge::Exception & exc) {
      RCLCPP_WARN(get_logger(), "Failed to convert images: %s", exc.what());
      return;
    }

    cv::Mat mask = CreateYellowMask(semantic);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto & contour : contours) {
      const double area = cv::contourArea(contour);
      if (area < get_parameter("min_area").as_double()) {
        continue;
      }

      auto centroid = ComputeCentroid(contour);
      if (!centroid) {
        continue;
      }

      const int depth_window = ComputeDepthWindow(area);
        auto depth_m = SampleDepthFromMask(depth, mask, contour);
      if (!depth_m) {
        depth_m = SampleDepth(depth, *centroid, depth_window);
      }
      if (!depth_m || std::isnan(*depth_m) || *depth_m <= 0.0) {
        continue;
      }

      auto camera_point = ProjectToCamera(*centroid, *depth_m);
      if (!camera_point) {
        continue;
      }

      auto world_point = CameraToWorld(*camera_point, semantic_msg->header.stamp);
      if (!world_point) {
        continue;
      }

      UpdateTracks(*world_point);
    }

    PublishTracks(semantic_msg->header.stamp);
  }

  cv::Mat CreateYellowMask(const cv::Mat & image) {
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    const auto lower_param = get_parameter("hsv_lower").as_integer_array();
    const auto upper_param = get_parameter("hsv_upper").as_integer_array();
    cv::Scalar lower(
      static_cast<int>(lower_param.at(0)),
      static_cast<int>(lower_param.at(1)),
      static_cast<int>(lower_param.at(2)));
    cv::Scalar upper(
      static_cast<int>(upper_param.at(0)),
      static_cast<int>(upper_param.at(1)),
      static_cast<int>(upper_param.at(2)));
    cv::Mat mask;
    cv::inRange(hsv, lower, upper, mask);
    return mask;
  }

  int ComputeDepthWindow(double contour_area) {
    const double scale = get_parameter("depth_window_scale").as_double();
    const double safe_area = std::max(0.0, contour_area);
    const int window = static_cast<int>(std::round(std::sqrt(safe_area) * scale));
    return std::max(1, window);
  }

  std::optional<double> SampleDepth(
    const cv::Mat & depth,
    const std::pair<int, int> & centroid,
    int window) {
    const int half = std::max(1, window / 2);
    const int u = centroid.first;
    const int v = centroid.second;
    const int h = depth.rows;
    const int w = depth.cols;
    const int u_min = std::max(0, u - half);
    const int u_max = std::min(w, u + half + 1);
    const int v_min = std::max(0, v - half);
    const int v_max = std::min(h, v + half + 1);
    if (u_min >= u_max || v_min >= v_max) {
      return std::nullopt;
    }

    cv::Mat patch = depth(cv::Range(v_min, v_max), cv::Range(u_min, u_max));
    cv::Mat patch_float;
    if (depth.type() == CV_16U) {
      patch.convertTo(patch_float, CV_32F, 1.0 / 1000.0);
    } else {
      patch.convertTo(patch_float, CV_32F);
    }

    std::vector<float> values;
    if (patch_float.isContinuous()) {
      values.assign(
        reinterpret_cast<const float *>(patch_float.datastart),
        reinterpret_cast<const float *>(patch_float.dataend));
    } else {
      values.reserve(static_cast<size_t>(patch_float.total()));
      for (int row = 0; row < patch_float.rows; ++row) {
        const float * row_ptr = patch_float.ptr<float>(row);
        values.insert(values.end(), row_ptr, row_ptr + patch_float.cols);
      }
    }
    return MedianOf(values);
  }

   std::optional<double> SampleDepthFromMask(
    const cv::Mat & depth,
    const cv::Mat & mask,
    const std::vector<cv::Point> & contour) {
    if (depth.empty() || mask.empty()) {
      return std::nullopt;
    }

    const cv::Rect bounds(0, 0, depth.cols, depth.rows);
    cv::Rect roi = cv::boundingRect(contour) & bounds;
    if (roi.width <= 0 || roi.height <= 0) {
      return std::nullopt;
    }

    struct DepthSample {
      float depth;
      cv::Point point;
    };
    std::vector<DepthSample> samples;
    samples.reserve(static_cast<size_t>(roi.area()));

    for (int y = roi.y; y < roi.y + roi.height; ++y) {
      const uint8_t * mask_row = mask.ptr<uint8_t>(y);
      for (int x = roi.x; x < roi.x + roi.width; ++x) {
        if (mask_row[x] == 0) {
          continue;
        }
        if (cv::pointPolygonTest(contour, cv::Point2f(x, y), false) < 0.0) {
          continue;
        }
        auto depth_value = DepthAtPixel(depth, x, y);
        if (!depth_value || !std::isfinite(*depth_value) || *depth_value <= 0.0) {
          continue;
        }
        samples.push_back({static_cast<float>(*depth_value), cv::Point(x, y)});
      }
    }

    if (samples.empty()) {
      return std::nullopt;
    }

    std::sort(samples.begin(), samples.end(),
      [](const DepthSample & a, const DepthSample & b) { return a.depth < b.depth; });
    const size_t subset = std::max<size_t>(1, samples.size() / 5);
    std::vector<float> closest_depths;
    closest_depths.reserve(subset);
    for (size_t i = 0; i < subset; ++i) {
      closest_depths.push_back(samples[i].depth);
    }
    return MedianOf(closest_depths);
  }

  std::optional<std::array<double, 3>> ProjectToCamera(
    const std::pair<int, int> & centroid,
    double depth_m) const {
    if (!camera_info_) {
      return std::nullopt;
    }
    const int u = centroid.first;
    const int v = centroid.second;
    const auto & k = camera_info_->k;
    const double fx = k[0];
    const double fy = k[4];
    const double cx = k[2];
    const double cy = k[5];
    if (fx == 0.0 || fy == 0.0) {
      return std::nullopt;
    }
    const double x = (static_cast<double>(u) - cx) / fx * depth_m;
    const double y = (static_cast<double>(v) - cy) / fy * depth_m;
    const double z = depth_m;
    return std::array<double, 3>{x, y, z};
  }

  std::optional<std::array<double, 3>> CameraToWorld(
    const std::array<double, 3> & camera_point,
    const builtin_interfaces::msg::Time & stamp) {
    const auto body_frame = get_parameter("body_frame").as_string();
    const auto offset = get_parameter("camera_offset").as_double_array();
    const auto body_point = CameraToBody(camera_point);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = body_frame;
    pose.header.stamp = stamp;
    pose.pose.position.x = body_point[0] + offset.at(0);
    pose.pose.position.y = body_point[1] + offset.at(1);
    pose.pose.position.z = body_point[2] + offset.at(2);
    pose.pose.orientation.w = 1.0;

    const double timeout_s = get_parameter("tf_timeout_s").as_double();
    const bool use_latest = get_parameter("use_latest_tf_on_extrapolation").as_bool();
    auto world_pose = TransformPose(pose, timeout_s, use_latest);
    if (!world_pose) {
      return std::nullopt;
    }

    return std::array<double, 3>{
      world_pose->pose.position.x,
      world_pose->pose.position.y,
      world_pose->pose.position.z};
  }

  static std::array<double, 3> CameraToBody(const std::array<double, 3> & camera_point) {
    return {camera_point[2], -camera_point[0], -camera_point[1]};
  }

  std::optional<geometry_msgs::msg::PoseStamped> TransformPose(
    const geometry_msgs::msg::PoseStamped & pose,
    double timeout_s,
    bool use_latest) {
    const auto target_frame = get_parameter("world_frame").as_string();
    try {
      return tf_buffer_->transform(pose, target_frame, tf2::durationFromSec(timeout_s));
    } catch (const tf2::TransformException & exc) {
      const auto message = ToLower(exc.what());
      if (use_latest && message.find("extrapolation into the future") != std::string::npos) {
        auto latest_pose = pose;
        try {
          const auto latest_transform = tf_buffer_->lookupTransform(
            target_frame, pose.header.frame_id, tf2::TimePointZero);
          const rclcpp::Time latest_ros_time(latest_transform.header.stamp);
          const rclcpp::Time stamp_time(pose.header.stamp);
          if (latest_ros_time < stamp_time) {
            latest_pose.header.stamp = latest_transform.header.stamp;
          }
        } catch (const tf2::TransformException & lookup_exc) {
          RCLCPP_WARN(get_logger(), "TF latest transform lookup failed: %s", lookup_exc.what());
        }
        try {
          return tf_buffer_->transform(latest_pose, target_frame, tf2::durationFromSec(timeout_s));
        } catch (const tf2::TransformException & retry_exc) {
          RCLCPP_WARN(get_logger(), "TF transform failed: %s", retry_exc.what());
          return std::nullopt;
        }
      }
      RCLCPP_WARN(get_logger(), "TF transform failed: %s", exc.what());
      return std::nullopt;
    }
  }

  void UpdateTracks(const std::array<double, 3> & world_point) {
    const double gating_distance = get_parameter("gating_distance").as_double();
    Track * best_track = nullptr;
    double best_distance = 0.0;
    for (auto & track : tracks_) {
      const double dx = track.position[0] - world_point[0];
      const double dy = track.position[1] - world_point[1];
      const double dz = track.position[2] - world_point[2];
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance <= gating_distance && (!best_track || distance < best_distance)) {
        best_track = &track;
        best_distance = distance;
      }
    }

    if (!best_track) {
      Track new_track;
      new_track.position = world_point;
      new_track.observations = 1;
      tracks_.push_back(new_track);
      return;
    }

    const int count = best_track->observations;
    best_track->position = {
      (best_track->position[0] * count + world_point[0]) / (count + 1),
      (best_track->position[1] * count + world_point[1]) / (count + 1),
      (best_track->position[2] * count + world_point[2]) / (count + 1)};
    best_track->observations += 1;
  }

  void PublishTracks(const builtin_interfaces::msg::Time & stamp) {
    const int min_observations = get_parameter("min_observations").as_int();
    const auto world_frame = get_parameter("world_frame").as_string();
    for (auto & track : tracks_) {
      if (track.observations < min_observations || track.published) {
        continue;
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = world_frame;
      pose.pose.position.x = track.position[0];
      pose.pose.position.y = track.position[1];
      pose.pose.position.z = track.position[2];
      pose.pose.orientation.w = 1.0;
      detections_pub_->publish(pose);
      track.published = true;
    }
  }

  message_filters::Subscriber<sensor_msgs::msg::Image> semantic_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  std::shared_ptr<Sync> sync_;

  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  std::vector<Track> tracks_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr detections_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LanternDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
