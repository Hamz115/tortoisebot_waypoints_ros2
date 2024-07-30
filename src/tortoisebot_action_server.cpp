#include "tortoisebot_waypoints/action/waypoint.hpp"
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>

class WaypointActionServer : public rclcpp::Node {
public:
  using Waypoint = tortoisebot_waypoints::action::Waypoint;
  using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

  WaypointActionServer() : Node("tortoisebot_as") {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Waypoint>(
        this, "tortoisebot_as",
        std::bind(&WaypointActionServer::handle_goal, this, _1, _2),
        std::bind(&WaypointActionServer::handle_cancel, this, _1),
        std::bind(&WaypointActionServer::handle_accepted, this, _1));

    pub_cmd_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&WaypointActionServer::odom_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Action server started");
  }

private:
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  geometry_msgs::msg::Point current_position_;
  double current_yaw_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_ = msg->pose.pose.position;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);
    RCLCPP_DEBUG(this->get_logger(), "Current position: (%f, %f), yaw: %f",
                 current_position_.x, current_position_.y, current_yaw_);
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Waypoint::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with position: (%f, %f)",
                goal->position.x, goal->position.y);
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
    std::thread{
        std::bind(&WaypointActionServer::execute, this, std::placeholders::_1),
        goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Waypoint::Feedback>();
    auto result = std::make_shared<Waypoint::Result>();

    double goal_x = goal->position.x;
    double goal_y = goal->position.y;

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      double dx = goal_x - current_position_.x;
      double dy = goal_y - current_position_.y;
      double distance = std::sqrt(dx * dx + dy * dy);
      double desired_yaw = std::atan2(dy, dx);

      auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();

      if (distance > 0.1) {
        // Move towards the goal
        twist_msg->linear.x = 0.2; // Adjust speed as needed
        double yaw_error = desired_yaw - current_yaw_;
        twist_msg->angular.z =
            0.5 * yaw_error; // Proportional control for rotation
      } else {
        // At goal position, ensure correct final orientation
        double yaw_error = desired_yaw - current_yaw_;
        if (std::abs(yaw_error) > 0.05) {
          twist_msg->angular.z = 0.2 * yaw_error;
        } else {
          // Goal reached with correct orientation
          twist_msg->linear.x = 0.0;
          twist_msg->angular.z = 0.0;
          pub_cmd_vel_->publish(std::move(twist_msg));

          result->success = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
          return;
        }
      }

      pub_cmd_vel_->publish(std::move(twist_msg));

      feedback->position = current_position_;
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    result->success = false;
    goal_handle->abort(result);
    RCLCPP_INFO(this->get_logger(), "Goal aborted");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<WaypointActionServer>();
  rclcpp::spin(action_server);
  rclcpp::shutdown();
  return 0;
}