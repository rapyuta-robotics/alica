#include "turtle.hpp"
#include "world_model.hpp"

#include <rclcpp/logging.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>

#include <chrono>

using namespace std::chrono_literals;

namespace turtlesim
{

ALICATurtle::ALICATurtle(ALICATurtleWorldModel* wm)
        : _wm(wm)
        , _spinner(rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), 1))
{
    _initTrigger = false;

    _nh = rclcpp::Node::make_shared("ros2_turtlesim");
    _spinner.add_node(_nh);
    _nh->declare_parameter("name", "Hello!");

    rclcpp::CallbackGroup::SharedPtr teleportCallback = _nh->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    _nh->get_parameter("name", _name);

    // initialize publisher, subscriber and service client.
    _velPub = _nh->create_publisher<geometry_msgs::msg::Twist>("/" + _name + "/cmd_vel", 1);
    _initTriggerSub = _nh->create_subscription<std_msgs::msg::Empty>("/init", 1, [&](const std_msgs::msg::Empty& msg) { _initTrigger = true; });
    _poseSub = _nh->create_subscription<turtlesim::msg::Pose>("/" + _name + "/pose", 1, [&](const msg::Pose::ConstSharedPtr msg) { _current = *msg; });
    _teleportClient = _nh->create_client<turtlesim::srv::TeleportAbsolute>("/" + _name + "/teleport_absolute", rmw_qos_profile_default, teleportCallback);

    _spinThread = std::thread([this]() { _spinner.spin(); });
}

void ALICATurtle::teleport(float x, float y)
{
    auto request = std::make_shared<srv::TeleportAbsolute::Request>();
    request->x = x;
    request->y = y;

    RCLCPP_INFO(_nh->get_logger(), "teleport req to x: %f, y: %f", x, y);
    auto result = _teleportClient->async_send_request(request);
}

bool ALICATurtle::moveTowardGoal(float x, float y)
{
    _goal.x = x;
    _goal.y = y;
    return moveTowardGoal();
}

bool ALICATurtle::moveTowardGoal() const
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
    _velPub->publish(msg);

    return is_reachGoal;
}

} // namespace turtlesim
