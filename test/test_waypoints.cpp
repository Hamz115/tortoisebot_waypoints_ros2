// // Copyright 2024 Your Name or Organization
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.

#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"
#include "tortoisebot_waypoints/action/waypoint.hpp"
#include <chrono>
#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

class TestWaypoints : public ::testing::Test {
protected:
  using Waypoint = tortoisebot_waypoints::action::Waypoint;

  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("test_waypoints");

    // Get goal coordinates from environment variables
    const char *goal_x_env = std::getenv("GOAL_X");
    const char *goal_y_env = std::getenv("GOAL_Y");

    if (goal_x_env) {
      goal_x_ = std::stod(goal_x_env);
    }
    if (goal_y_env) {
      goal_y_ = std::stod(goal_y_env);
    }

    RCLCPP_INFO(node_->get_logger(), "Testing with goal: (%f, %f)", goal_x_,
                goal_y_);

    client_ = rclcpp_action::create_client<Waypoint>(node_, "tortoisebot_as");
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&TestWaypoints::odom_callback, this, std::placeholders::_1));

    ASSERT_TRUE(client_->wait_for_action_server(20s));
    wait_for_odom();
    initial_position_ = current_position_;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_ = msg->pose.pose.position;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);
  }

  void wait_for_odom() {
    rclcpp::Time start = node_->now();
    while (rclcpp::ok() && (node_->now() - start) < rclcpp::Duration(5s)) {
      rclcpp::spin_some(node_);
      if (current_position_.x != 0 || current_position_.y != 0 ||
          current_position_.z != 0) {
        return;
      }
      std::this_thread::sleep_for(100ms);
    }
    FAIL() << "Failed to receive odometry data";
  }

  bool check_position() {
    return (std::abs(current_position_.x - goal_x_) <= tolerance_ &&
            std::abs(current_position_.y - goal_y_) <= tolerance_);
  }

  bool check_orientation() {
    double expected_yaw = std::atan2(goal_y_, goal_x_);
    double yaw_error = std::abs(current_yaw_ - expected_yaw);
    yaw_error = std::min(yaw_error, 2 * M_PI - yaw_error);
    return yaw_error < M_PI / 2;
  }

  bool has_moved_significantly() {
    double distance_moved =
        std::sqrt(std::pow(current_position_.x - initial_position_.x, 2) +
                  std::pow(current_position_.y - initial_position_.y, 2));
    return distance_moved > 0.1; // Consider 10cm as significant movement
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp_action::Client<Waypoint>::SharedPtr client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  geometry_msgs::msg::Point current_position_;
  double current_yaw_;
  geometry_msgs::msg::Point initial_position_;
  double goal_x_ = 0.5;
  double goal_y_ = 0.5;
  double tolerance_ = 0.1;
};

TEST_F(TestWaypoints, WaypointTest) {
  auto goal_msg = Waypoint::Goal();
  goal_msg.position.x = goal_x_;
  goal_msg.position.y = goal_y_;
  goal_msg.position.z = 0.0;

  auto send_goal_options = rclcpp_action::Client<Waypoint>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      [](const rclcpp_action::ClientGoalHandle<Waypoint>::SharedPtr
             &goal_handle) { ASSERT_TRUE(goal_handle); };

  auto goal_handle_future =
      client_->async_send_goal(goal_msg, send_goal_options);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_, goal_handle_future),
            rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = goal_handle_future.get();
  auto result_future = client_->async_get_result(goal_handle);

  rclcpp::Time start_time = node_->now();
  rclcpp::Duration max_test_duration = rclcpp::Duration(38s);

  while (rclcpp::ok() && (node_->now() - start_time) < max_test_duration) {
    rclcpp::spin_some(node_);

    if (result_future.wait_for(100ms) == std::future_status::ready) {
      auto result = result_future.get();
      if (goal_x_ == 0.5 && goal_y_ == 0.5) {
        EXPECT_TRUE(result.result->success) << "Robot failed to reach the goal";
        EXPECT_TRUE(check_position())
            << "Robot did not reach the goal position";
        EXPECT_TRUE(check_orientation())
            << "Robot's orientation is not correct";
      } else {
        EXPECT_FALSE(result.result->success)
            << "Robot unexpectedly succeeded in reaching an unreachable goal";
      }
      return;
    }

    if (goal_x_ != 0.5 || goal_y_ != 0.5) {
      if (has_moved_significantly()) {
        FAIL() << "Test failed: Robot moved significantly towards unreachable "
                  "goal ("
               << goal_x_ << ", " << goal_y_ << ")";
        return;
      }
    }

    std::this_thread::sleep_for(100ms);
  }

  FAIL() << "Test timed out: Robot did not reach the goal (" << goal_x_ << ", "
         << goal_y_ << ") in time";
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
