#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp> // Vital for cloud transforms
#include <geometry_msgs/msg/transform_stamped.hpp>

class PointCloudTransformer : public rclcpp::Node {
public:
    PointCloudTransformer() : Node("pointcloud_transformer") {
        // Declare parameter for flexibility
        this->declare_parameter<std::string>("target_frame", "world");
        this->get_parameter("target_frame", target_frame_);

        // Setup TF Buffer and Listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create Publisher and Subscriber
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", 10);
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "input", 10,
            std::bind(&PointCloudTransformer::cloud_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "PointCloud Transformer Node Started. Target: %s", target_frame_.c_str());
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Check if the transform is available
        if (!tf_buffer_->canTransform(target_frame_, msg->header.frame_id, tf2::TimePointZero)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Waiting for transform from %s to %s", msg->header.frame_id.c_str(), target_frame_.c_str());
            return;
        }

        try {
            // Lookup transform
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                target_frame_, msg->header.frame_id, tf2::TimePointZero);
            
            // Transform the cloud
            sensor_msgs::msg::PointCloud2 cloud_out;
            tf2::doTransform(*msg, cloud_out, transform);
            
            // Publish
            pub_->publish(cloud_out);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        }
    }

    std::string target_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudTransformer>());
    rclcpp::shutdown();
    return 0;
}