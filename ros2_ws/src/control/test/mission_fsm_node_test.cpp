#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "control/mission_fsm_node.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class MissionFsmNodeTest : public ::testing::Test {
protected:
  void SetUp() override {
    // rclcpp::init is handled in main
    node_ = std::make_shared<control::MissionFsmNode>();
    
    // Create a companion node to publish fake inputs and subscribe to outputs
    tester_node_ = std::make_shared<rclcpp::Node>("tester_node");
    
    odom_pub_ = tester_node_->create_publisher<nav_msgs::msg::Odometry>(
      "/current_state_est", 10);
      
    lantern_pub_ = tester_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/detected_lanterns", 10);
      
    planner_status_pub_ = tester_node_->create_publisher<std_msgs::msg::String>(
      "/planner/status", 10);

    start_pub_ = tester_node_->create_publisher<std_msgs::msg::Empty>(
      "/mission/start", 10);
      
    state_sub_ = tester_node_->create_subscription<std_msgs::msg::String>(
      "/fsm/state", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        last_state_ = msg->data;
        if (msg->data == control::state_to_string(control::MissionState::LANTERN_FOUND)) {
          seen_lantern_found_ = true;
        }
      });
  }

  void TearDown() override {
    node_.reset();
    tester_node_.reset();
    // rclcpp::shutdown is handled in main
  }
  
  void Spin(std::chrono::milliseconds duration) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      rclcpp::spin_some(node_);
      rclcpp::spin_some(tester_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  
  void PublishOdom(double x, double y, double z) {
    nav_msgs::msg::Odometry msg;
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = z;
    odom_pub_->publish(msg);
  }

  void PublishStartSignal() {
    std_msgs::msg::Empty msg;
    start_pub_->publish(msg);
  }

  std::shared_ptr<control::MissionFsmNode> node_;
  std::shared_ptr<rclcpp::Node> tester_node_;
  
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lantern_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr start_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  
  std::string last_state_;
  bool seen_lantern_found_ = false;
};

TEST_F(MissionFsmNodeTest, InitialStateIsInit) {
  Spin(std::chrono::milliseconds(200));
  // Based on constructor: "Mission FSM Node initialized. State: INIT"
  // It should publish INIT initially.
  EXPECT_EQ(last_state_, control::state_to_string(control::MissionState::INIT));

  // Even with Odom, it should stay in INIT if no start signal
  PublishOdom(0.0, 0.0, 0.0);
  Spin(std::chrono::milliseconds(200));
  EXPECT_EQ(last_state_, control::state_to_string(control::MissionState::INIT));
}

TEST_F(MissionFsmNodeTest, TransitionsToTakeoffAfterOdomAndSignal) {
  // Initially INIT
  Spin(std::chrono::milliseconds(100));
  
  // Publish Odom
  PublishOdom(0.0, 0.0, 0.0);
  Spin(std::chrono::milliseconds(100));

  // Publish Start Signal
  PublishStartSignal();
  
  // Wait for processing
  Spin(std::chrono::milliseconds(200));
  
  EXPECT_EQ(last_state_, control::state_to_string(control::MissionState::TAKEOFF));
}

TEST_F(MissionFsmNodeTest, TransitionsToGotoEntranceAfterTakeoffHeightReached) {
  // 1. INIT -> TAKEOFF
  PublishOdom(0.0, 0.0, 0.0);
  PublishStartSignal();
  Spin(std::chrono::milliseconds(200));
  EXPECT_EQ(last_state_, control::state_to_string(control::MissionState::TAKEOFF));
  
  // 2. TAKEOFF -> GOTO_ENTRANCE
  // Takeoff altitude is 2.0 (default in constructor)
  // Simulate reaching 2.0m
  PublishOdom(0.0, 0.0, 2.0);
  Spin(std::chrono::milliseconds(200));
  
  EXPECT_EQ(last_state_, control::state_to_string(control::MissionState::GOTO_ENTRANCE));
}

TEST_F(MissionFsmNodeTest, TransitionsToExploreAfterReachingEntrance) {
  // 1. INIT -> TAKEOFF
  PublishOdom(0.0, 0.0, 0.0);
  PublishStartSignal();
  Spin(std::chrono::milliseconds(200));
  
  // 2. TAKEOFF -> GOTO_ENTRANCE
  PublishOdom(0.0, 0.0, 2.0);
  Spin(std::chrono::milliseconds(200));
  
  // 3. GOTO_ENTRANCE -> EXPLORE
  // Entrance is at (10.0, 0.0, 2.0) default
  PublishOdom(10.0, 0.0, 2.0);
  Spin(std::chrono::milliseconds(200));
  
  EXPECT_EQ(last_state_, control::state_to_string(control::MissionState::EXPLORE));
}

TEST_F(MissionFsmNodeTest, LanternDetectionInterruptsExplore) {
  // Fast forward to EXPLORE
  PublishOdom(0.0, 0.0, 0.0); // INIT -> TAKEOFF
  PublishStartSignal();
  Spin(std::chrono::milliseconds(100));
  PublishOdom(0.0, 0.0, 2.0); // TAKEOFF -> GOTO_ENTRANCE
  Spin(std::chrono::milliseconds(100));
  PublishOdom(10.0, 0.0, 2.0); // GOTO_ENTRANCE -> EXPLORE
  Spin(std::chrono::milliseconds(100));
  
  ASSERT_EQ(last_state_, control::state_to_string(control::MissionState::EXPLORE));
  
  // Wait for subscriber to connect
  auto start_wait = std::chrono::steady_clock::now();
  while (lantern_pub_->get_subscription_count() == 0 && 
         std::chrono::steady_clock::now() - start_wait < std::chrono::seconds(2)) {
    Spin(std::chrono::milliseconds(10));
  }
  ASSERT_GT(lantern_pub_->get_subscription_count(), 0U);

  // Publish Lantern repeatedly until state changes or timeout
  geometry_msgs::msg::PoseStamped lantern_msg;
  lantern_msg.pose.position.x = 25.0; 
  lantern_msg.pose.position.y = 5.0;
  lantern_msg.pose.position.z = 1.0;

  auto start_pub = std::chrono::steady_clock::now();
  while (last_state_ != "LANTERN_FOUND" && 
         std::chrono::steady_clock::now() - start_pub < std::chrono::seconds(3)) {
    lantern_pub_->publish(lantern_msg);
    Spin(std::chrono::milliseconds(50));
  }
  
  EXPECT_EQ(last_state_, control::state_to_string(control::MissionState::LANTERN_FOUND));
}

TEST_F(MissionFsmNodeTest, PlannerFailureTriggersRefineGoal) {
  // Fast forward to EXPLORE
  PublishOdom(0.0, 0.0, 0.0); // INIT -> TAKEOFF
  PublishStartSignal();
  Spin(std::chrono::milliseconds(100));
  PublishOdom(0.0, 0.0, 2.0); // TAKEOFF -> GOTO_ENTRANCE
  Spin(std::chrono::milliseconds(100));
  PublishOdom(10.0, 0.0, 2.0); // GOTO_ENTRANCE -> EXPLORE
  Spin(std::chrono::milliseconds(100));
  
  ASSERT_EQ(last_state_, control::state_to_string(control::MissionState::EXPLORE));
  
  // Publish PLAN_FAILED
  std_msgs::msg::String status_msg;
  status_msg.data = "PLAN_FAILED";
  planner_status_pub_->publish(status_msg);
  
  Spin(std::chrono::milliseconds(100));
  
  EXPECT_EQ(last_state_, control::state_to_string(control::MissionState::REFINE_GOAL));
}

TEST_F(MissionFsmNodeTest, FullMissionLifecycle) {
  // 1. INIT -> TAKEOFF
  PublishOdom(0.0, 0.0, 0.0);
  PublishStartSignal();
  Spin(std::chrono::milliseconds(100));
  
  // 2. TAKEOFF -> GOTO_ENTRANCE
  PublishOdom(0.0, 0.0, 2.0); // Reach takeoff altitude
  Spin(std::chrono::milliseconds(100));
  
  // 3. GOTO_ENTRANCE -> EXPLORE
  PublishOdom(10.0, 0.0, 2.0); // Reach entrance
  Spin(std::chrono::milliseconds(100));
  
  ASSERT_EQ(last_state_, control::state_to_string(control::MissionState::EXPLORE));

  // Wait for subscriber
  auto start_wait = std::chrono::steady_clock::now();
  while (lantern_pub_->get_subscription_count() == 0 && 
         std::chrono::steady_clock::now() - start_wait < std::chrono::seconds(2)) {
    Spin(std::chrono::milliseconds(10));
  }
  ASSERT_GT(lantern_pub_->get_subscription_count(), 0U);

  // 4. Find 4 Lanterns
  for (int i = 0; i < 4; ++i) {
    // Publish unique lantern
    geometry_msgs::msg::PoseStamped lantern_msg;
    lantern_msg.pose.position.x = 20.0 + i * 2.0; // Different positions to avoid dedup
    lantern_msg.pose.position.y = 5.0;
    lantern_msg.pose.position.z = 1.0;

    // Publish until state changes to LANTERN_FOUND
    auto start_pub = std::chrono::steady_clock::now();
    seen_lantern_found_ = false; // Reset flag
    
    while (std::chrono::steady_clock::now() - start_pub < std::chrono::seconds(2)) {
      if (seen_lantern_found_) {
        break;
      }
      lantern_pub_->publish(lantern_msg);
      Spin(std::chrono::milliseconds(50));
    }
    EXPECT_TRUE(seen_lantern_found_);

    // Wait for automatic transition back to EXPLORE or RETURN (if last lantern)
    auto start_wait_explore = std::chrono::steady_clock::now();
    while (last_state_ == control::state_to_string(control::MissionState::LANTERN_FOUND) &&
           std::chrono::steady_clock::now() - start_wait_explore < std::chrono::seconds(2)) {
      Spin(std::chrono::milliseconds(50));
    }
    
    if (i < 3) {
      EXPECT_EQ(last_state_, control::state_to_string(control::MissionState::EXPLORE));
    } else {
      // After 4th lantern, it goes LANTERN_FOUND -> EXPLORE -> RETURN
      // We might catch it in EXPLORE, so wait for RETURN
      auto start_wait_return = std::chrono::steady_clock::now();
      while (last_state_ != control::state_to_string(control::MissionState::RETURN) &&
             std::chrono::steady_clock::now() - start_wait_return < std::chrono::seconds(2)) {
        Spin(std::chrono::milliseconds(50));
      }
      EXPECT_EQ(last_state_, control::state_to_string(control::MissionState::RETURN));
    }
  }

  // 5. Verify Transition to RETURN
  // Already verified in loop above, but double check
  EXPECT_EQ(last_state_, control::state_to_string(control::MissionState::RETURN));

  // 6. RETURN -> LAND
  // Simulate returning to start (0,0 was initial)
  PublishOdom(0.0, 0.0, 2.0); 
  Spin(std::chrono::milliseconds(200));
  EXPECT_EQ(last_state_, control::state_to_string(control::MissionState::LAND));

  // 7. LAND -> Landed (Stay in LAND, but goal_active_ becomes false)
  // Simulate landing
  PublishOdom(0.0, 0.0, 0.1); 
  Spin(std::chrono::milliseconds(200));
  
  // State should still be LAND
  EXPECT_EQ(last_state_, control::state_to_string(control::MissionState::LAND));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
