// Copyright 2024 Your Name or Organization
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/utils.h"
#include "tortoisebot_waypoints/action/waypoint.hpp"

class WaypointActionServer : public rclcpp::Node {
public:
  using Waypoint = tortoisebot_waypoints::action::Waypoint;
  using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

  explicit WaypointActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("tortoisebot_action_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Waypoint>(
        this, "tortoisebot_as",
        std::bind(&WaypointActionServer::handle_goal, this, _1, _2),
        std::bind(&WaypointActionServer::handle_cancel, this, _1),
        std::bind(&WaypointActionServer::handle_accepted, this, _1));

    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&WaypointActionServer::odom_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Action server started");
  }

private:
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  geometry_msgs::msg::Point position_;
  double yaw_;
  const double yaw_precision_ = M_PI / 90.0; // +/- 2 degree allowed
  const double dist_precision_ = 0.05;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Waypoint::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with position [%f, %f, %f]",
                goal->position.x, goal->position.y, goal->position.z);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&WaypointActionServer::execute, this, _1),
                goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Waypoint::Feedback>();
    auto result = std::make_shared<Waypoint::Result>();

    while (rclcpp::ok()) {
      // Check if the goal has been canceled
      if (goal_handle->is_canceling()) {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        return;
      }

      double desired_yaw = std::atan2(goal->position.y - position_.y,
                                      goal->position.x - position_.x);
      double err_yaw = desired_yaw - yaw_;
      double err_pos = std::sqrt(std::pow(goal->position.y - position_.y, 2) +
                                 std::pow(goal->position.x - position_.x, 2));

      if (err_pos > dist_precision_) {
        geometry_msgs::msg::Twist twist_msg;
        if (std::fabs(err_yaw) > yaw_precision_) {
          // fix yaw
          twist_msg.angular.z = 0.65 * (err_yaw > 0 ? 1 : -1);
          feedback->state = "fix yaw";
        } else {
          // go to point
          twist_msg.linear.x = 0.6;
          twist_msg.angular.z = 0;
          feedback->state = "go to point";
        }
        cmd_vel_publisher_->publish(twist_msg);

        feedback->position = position_;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publishing feedback");
      } else {
        break;
      }

      loop_rate.sleep();
    }

    // Stop the robot
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    cmd_vel_publisher_->publish(twist_msg);

    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    position_ = msg->pose.pose.position;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw_);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointActionServer>());
  rclcpp::shutdown();
  return 0;
}
