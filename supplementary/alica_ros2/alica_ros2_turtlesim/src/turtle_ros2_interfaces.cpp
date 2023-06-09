#include "alica_ros2_turtlesim/turtle_ros2_interfaces.hpp"

#include <rclcpp/logging.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>

#include <chrono>

namespace turtlesim
{

TurtleRos2Interfaces::TurtleRos2Interfaces(const std::string& name)
        : TurtleInterfaces(name)
        , _name(name)
{
    // initialize publisher, subscriber and service client.
    _initTrigger = false;

    _nh = rclcpp::Node::make_shared("ros2_turtlesim");
    _spinner.add_node(_nh);
    _nh->declare_parameter("name", "Hello!");

    rclcpp::CallbackGroup::SharedPtr teleportCallback = _nh->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    _nh->get_parameter("name", _name);

    // initialize publisher, subscriber and service client.
    _velPub = _nh->create_publisher<geometry_msgs::msg::Twist>("/" + _name + "/cmd_vel", 1);
    _initTriggerSub = _nh->create_subscription<std_msgs::msg::Empty>("/init", 1, [&](const std_msgs::msg::Empty& msg) { _initTrigger = true; });
    _poseSub = _nh->create_subscription<turtlesim::msg::Pose>("/" + _name + "/pose", 1, [&](const msg::Pose::ConstSharedPtr msg) { _current = msg; });
    _teleportClient = _nh->create_client<turtlesim::srv::TeleportAbsolute>("/" + _name + "/teleport_absolute", rmw_qos_profile_default, teleportCallback);

    _spinThread = std::thread([this]() { _spinner.spin(); });
}

bool TurtleRos2Interfaces::teleport(float x, float y)
{
    auto request = std::make_shared<srv::TeleportAbsolute::Request>();
    request->x = x;
    request->y = y;

    RCLCPP_INFO(_nh->get_logger(), "teleport req to x: %f, y: %f", x, y);
    auto result = _teleportClient->async_send_request(request);
    return true;
}

bool TurtleRos2Interfaces::spawn()
{
    auto request = std::make_shared<srv::Spawn::Request>();
    request->x = 5;
    request->y = 5;
    request->theta = 0;
    request->name = _name;
    auto result = _spawnClient->async_send_request(request);
    return true;
}

void TurtleRos2Interfaces::rotate(const float dYaw)
{
    geometry_msgs::msg::Twist msg;
    msg.angular.z = dYaw;
    _velPub->publish(msg);
}

bool TurtleRos2Interfaces::moveTowardPosition(float x, float y) const
{
    if (!_current) {
        RCLCPP_WARN(_nh->get_logger(), "Waiting for valid pose");
        // Wait until we have a valid pose
        return false;
    }
    // Transform goal position into coordinates of turtle body frame
    float cosTheta = std::cos(_current->theta);
    float sinTheta = std::sin(_current->theta);
    float dx = x - _current->x;
    float dy = y - _current->y;

    // Calculate goal distance and return true if reach goal
    constexpr float goalTolerance = 0.01;
    bool isReachGoal = false;
    if (dx * dx + dy * dy <= goalTolerance * goalTolerance) {
        isReachGoal = true;
        return isReachGoal;
    }

    // Transform to turtle body frame
    float txG = dx * cosTheta + dy * sinTheta;
    float tyG = -dx * sinTheta + dy * cosTheta;

    // Simple proposional control
    constexpr float kLin = 2.0;
    constexpr float kAng = 2.0;

    float linearX = kLin * txG;
    float linearY = kLin * tyG;
    float headingError = std::atan2(linearY, linearX);
    float angular = kAng * headingError;

    geometry_msgs::msg::Twist msg;
    msg.linear.x = linearX;
    msg.angular.z = angular;
    _velPub->publish(msg);

    return isReachGoal;
}

} // namespace turtlesim
