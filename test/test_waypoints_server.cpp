#include "tortoisebot_waypoints/action/waypoint.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class WaypointTestClient : public rclcpp::Node {
public:
  using WaypointAction = tortoisebot_interfaces::action::Waypoint;

  WaypointTestClient() : Node("waypoint_test_client") {
    action_client_ =
        rclcpp_action::create_client<WaypointAction>(this, "tortoisebot_as");
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&WaypointTestClient::odom_callback, this,
                  std::placeholders::_1));
  }

  bool send_waypoint(double x, double y) {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Action server not available");
      return false;
    }

    auto goal = WaypointAction::Goal();
    goal.position.x = x;
    goal.position.y = y;

    auto send_goal_options =
        rclcpp_action::Client<WaypointAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&WaypointTestClient::goal_response_callback, this,
                  std::placeholders::_1);

    auto goal_handle_future =
        action_client_->async_send_goal(goal, send_goal_options);

    if (goal_handle_future.wait_for(std::chrono::seconds(5)) !=
        std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Failed to send goal");
      return false;
    }

    goal_handle_ = goal_handle_future.get();
    return goal_handle_ != nullptr;
  }

  bool wait_for_result() {
    if (!goal_handle_) {
      RCLCPP_ERROR(get_logger(), "Goal handle is null");
      return false;
    }

    auto result_future = action_client_->async_get_result(goal_handle_);

    if (result_future.wait_for(std::chrono::seconds(60)) !=
        std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Action did not complete within timeout");
      return false;
    }

    auto result = result_future.get();
    return result.code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  nav_msgs::msg::Odometry::SharedPtr get_latest_odom() const {
    return latest_odom_;
  }

private:
  rclcpp_action::Client<WaypointAction>::SharedPtr action_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::ClientGoalHandle<WaypointAction>::SharedPtr goal_handle_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odom_ = msg;
  }

  void goal_response_callback(
      const rclcpp_action::ClientGoalHandle<WaypointAction>::SharedPtr
          &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(get_logger(), "Goal accepted by server");
    }
  }
};

class WaypointServerTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    test_node_ = std::make_shared<WaypointTestClient>();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(test_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override {
    executor_->cancel();
    spin_thread_.join();
    rclcpp::shutdown();
  }

  std::shared_ptr<WaypointTestClient> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST_F(WaypointServerTest, TestWaypointNavigation) {
  const double goal_x = 1.0;
  const double goal_y = 1.0;
  const double position_tolerance = 0.1;
  const double yaw_tolerance = 0.1;

  ASSERT_TRUE(test_node_->send_waypoint(goal_x, goal_y));
  ASSERT_TRUE(test_node_->wait_for_result());

  auto final_odom = test_node_->get_latest_odom();
  ASSERT_NE(final_odom, nullptr);

  // Check final position
  EXPECT_NEAR(final_odom->pose.pose.position.x, goal_x, position_tolerance);
  EXPECT_NEAR(final_odom->pose.pose.position.y, goal_y, position_tolerance);

  // Check final orientation
  tf2::Quaternion q(
      final_odom->pose.pose.orientation.x, final_odom->pose.pose.orientation.y,
      final_odom->pose.pose.orientation.z, final_odom->pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  double expected_yaw = std::atan2(goal_y, goal_x);
  EXPECT_NEAR(yaw, expected_yaw, yaw_tolerance);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}