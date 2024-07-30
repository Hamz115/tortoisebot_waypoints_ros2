#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tortoisebot_waypoints/action/waypoint.hpp"

class TestWaypointActionServer : public ::testing::Test {
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("test_waypoint_client");
    action_client_ =
        rclcpp_action::create_client<tortoisebot_waypoints::action::Waypoint>(
            node_, "tortoisebot_as");

    ASSERT_TRUE(
        action_client_->wait_for_action_server(std::chrono::seconds(90)));

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          last_pose_ = msg->pose.pose;
        });

    char *x_env = std::getenv("TEST_X");
    char *y_env = std::getenv("TEST_Y");
    test_x_ = x_env ? std::stod(x_env) : 0.5;
    test_y_ = y_env ? std::stod(y_env) : 0.5;

    char *tolerance_env = std::getenv("TEST_TOLERANCE");
    tolerance_ = tolerance_env ? std::stod(tolerance_env) : 0.1;
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp_action::Client<tortoisebot_waypoints::action::Waypoint>::SharedPtr
      action_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  geometry_msgs::msg::Pose last_pose_;
  double test_x_, test_y_, tolerance_;

  double getYaw(const geometry_msgs::msg::Quaternion &q) {
    double roll, pitch, yaw;
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  double normalizeAngle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }
};

TEST_F(TestWaypointActionServer, TestEndPosition) {
  auto goal_msg = tortoisebot_waypoints::action::Waypoint::Goal();
  goal_msg.position.x = test_x_;
  goal_msg.position.y = test_y_;
  goal_msg.position.z = 0.0;

  auto goal_handle_future = action_client_->async_send_goal(goal_msg);
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS,
            rclcpp::spin_until_future_complete(node_, goal_handle_future,
                                               std::chrono::seconds(90)));

  auto goal_handle = goal_handle_future.get();
  ASSERT_NE(nullptr, goal_handle);

  auto result_future = action_client_->async_get_result(goal_handle);
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS,
            rclcpp::spin_until_future_complete(node_, result_future,
                                               std::chrono::seconds(90)));

  auto wrapped_result = result_future.get();
  ASSERT_EQ(rclcpp_action::ResultCode::SUCCEEDED, wrapped_result.code);
  ASSERT_TRUE(wrapped_result.result->success);

  rclcpp::sleep_for(std::chrono::seconds(2));

  EXPECT_NEAR(goal_msg.position.x, last_pose_.position.x, tolerance_);
  EXPECT_NEAR(goal_msg.position.y, last_pose_.position.y, tolerance_);
}

TEST_F(TestWaypointActionServer, TestEndRotation) {
  auto goal_msg = tortoisebot_waypoints::action::Waypoint::Goal();
  goal_msg.position.x = test_x_;
  goal_msg.position.y = test_y_;
  goal_msg.position.z = 0.0;

  auto goal_handle_future = action_client_->async_send_goal(goal_msg);
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS,
            rclcpp::spin_until_future_complete(node_, goal_handle_future,
                                               std::chrono::seconds(90)));

  auto goal_handle = goal_handle_future.get();
  ASSERT_NE(nullptr, goal_handle);

  auto result_future = action_client_->async_get_result(goal_handle);
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS,
            rclcpp::spin_until_future_complete(node_, result_future,
                                               std::chrono::seconds(90)));

  auto wrapped_result = result_future.get();
  ASSERT_EQ(rclcpp_action::ResultCode::SUCCEEDED, wrapped_result.code);
  ASSERT_TRUE(wrapped_result.result->success);

  rclcpp::sleep_for(std::chrono::seconds(2));

  double expected_yaw = std::atan2(goal_msg.position.y - last_pose_.position.y,
                                   goal_msg.position.x - last_pose_.position.x);
  double actual_yaw = getYaw(last_pose_.orientation);

  expected_yaw = normalizeAngle(expected_yaw);
  actual_yaw = normalizeAngle(actual_yaw);

  RCLCPP_INFO(node_->get_logger(), "Expected yaw: %f, Actual yaw: %f",
              expected_yaw, actual_yaw);

  double yaw_diff = std::abs(expected_yaw - actual_yaw);
  EXPECT_TRUE(yaw_diff <= tolerance_ ||
              std::abs(yaw_diff - 2 * M_PI) <= tolerance_)
      << "Expected yaw: " << expected_yaw << ", Actual yaw: " << actual_yaw
      << ", Difference: " << yaw_diff << ", Tolerance: " << tolerance_;
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}