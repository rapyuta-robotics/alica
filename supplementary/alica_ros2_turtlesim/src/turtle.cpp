#include "alica_ros2_turtlesim/turtle.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/logging.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>

#include <chrono>

using namespace std::chrono_literals;

namespace turtlesim
{

ALICATurtle::ALICATurtle(rclcpp::Node::SharedPtr priv_nh)
{
    priv_nh->get_parameter("name", _name);
    _priv_nh = priv_nh;
    // initialize publisher, subscriber and service client.
    _vel_pub = priv_nh->create_publisher<geometry_msgs::msg::Twist>("/" + _name + "/cmd_vel", 1);
    _pose_sub = priv_nh->create_subscription<turtlesim::msg::Pose>(
            "/" + _name + "/pose", 1, std::bind(&ALICATurtle::pose_sub_callback, this, std::placeholders::_1));
    _teleport_client = priv_nh->create_client<turtlesim::srv::TeleportAbsolute>("/" + _name + "/teleport_absolute");
}

void ALICATurtle::teleport(float x, float y)
{
    auto request = std::make_shared<srv::TeleportAbsolute::Request>();
    request->x = x;
    request->y = y;

    RCLCPP_INFO(_priv_nh->get_logger(), "teleport req to x: %f, y: %f", x, y);
    auto result = _teleport_client->async_send_request(request);

    // result.wait();
    // RCLCPP_INFO(_priv_nh->get_logger(), "AAAAAA");
    // if (result.valid()) {
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s was teleported to (%f, %f)", _name.c_str(), x, y);
    // } else {
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to teleport %s to (%f, %f)", _name.c_str(), x, y);
    // }
}

void ALICATurtle::pose_sub_callback(const msg::Pose::ConstSharedPtr msg)
{
    float x = msg->x;
    float y = msg->y;
    // RCLCPP_INFO(_priv_nh->get_logger(), "Pose sub callback: %f, %f, %f", x, y, msg->theta);
    _current = *msg;
}

bool ALICATurtle::move_toward_goal(float x, float y)
{
    // RCLCPP_INFO(_priv_nh->get_logger(), "Move towards goal %f %f", x, y);
    _goal.x = x;
    _goal.y = y;
    return move_toward_goal();
}
bool ALICATurtle::move_toward_goal() const
{
    // Transform goal position into coordinates of turtle body frame
    float cos_theta = std::cos(_current.theta);
    float sin_theta = std::sin(_current.theta);
    float dx = _goal.x - _current.x;
    float dy = _goal.y - _current.y;

    // Calculate goal distance and return true if reach goal
    constexpr float goal_tolerance = 0.01;
    bool is_reachGoal = false;
    if (dx * dx + dy * dy <= goal_tolerance * goal_tolerance) {
        is_reachGoal = true;
        return is_reachGoal;
    }

    // Transform to turtle body frame
    float tx_g = dx * cos_theta + dy * sin_theta;
    float ty_g = -dx * sin_theta + dy * cos_theta;

    // Simple proposional control
    constexpr float k_lin = 2.0;
    constexpr float k_ang = 2.0;

    float linear_x = k_lin * tx_g;
    float linear_y = k_lin * ty_g;
    float heading_error = std::atan2(linear_y, linear_x);
    float angular = k_ang * heading_error;

    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear_x;
    msg.angular.z = angular;
    _vel_pub->publish(msg);

    return is_reachGoal;
}

} // namespace turtlesim
