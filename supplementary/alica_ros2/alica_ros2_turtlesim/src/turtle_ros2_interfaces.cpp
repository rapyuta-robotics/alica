#include "alica_ros2_turtlesim/turtle_ros2_interfaces.hpp"

#include <engine/blackboard/Blackboard.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/logging.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>

#include <chrono>
#include <future>
using namespace std::chrono_literals;

namespace turtlesim
{

TurtleRos2Interfaces::TurtleRos2Interfaces(const std::string& name)
        : TurtleInterfaces(name)
        , _name(name)
{
    // initialize publisher, subscriber and service client.
    _initTrigger = false;

    _nh = rclcpp::Node::make_shared("alica_ros2_turtlesim");
    _spinner.add_node(_nh);
    _nh->declare_parameter("name", "Hello!");

    rclcpp::CallbackGroup::SharedPtr teleportCallback = _nh->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    _nh->get_parameter("name", _name);

    // initialize publisher, subscriber and service client.
    _velPub = _nh->create_publisher<geometry_msgs::msg::Twist>("/" + _name + "/cmd_vel", 10);
    _initTriggerSub = _nh->create_subscription<std_msgs::msg::Empty>("/init", 10, [&](const std_msgs::msg::Empty& msg) { _initTrigger = true; });
    _poseSub = _nh->create_subscription<turtlesim::msg::Pose>("/" + _name + "/pose", 10, [&](const msg::Pose::ConstSharedPtr msg) { _current = msg; });
    _teleportClient = _nh->create_client<turtlesim::srv::TeleportAbsolute>("/" + _name + "/teleport_absolute", rmw_qos_profile_default, teleportCallback);

    rclcpp::CallbackGroup::SharedPtr spawnCallback = _nh->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    _spawnClient = _nh->create_client<turtlesim::srv::Spawn>("/spawn", rmw_qos_profile_services_default, spawnCallback);
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

    auto request = std::make_shared<turtlesim::srv::Spawn_Request>();
    request->x = 1;
    request->y = 1;
    request->theta = 0;
    request->name = _name;
    auto result = _spawnClient->async_send_request(request);
    RCLCPP_INFO(_nh->get_logger(), "Request to spawn %s", _name.c_str());
    auto status = result.wait_for(3s);
    if (status == std::future_status::ready) {
        RCLCPP_INFO(_nh->get_logger(), "Spawned %s", _name.c_str());
    } else {
        RCLCPP_ERROR(_nh->get_logger(), "Spawn of %s failed", _name.c_str());
    }
    RCLCPP_INFO(_nh->get_logger(), "Spawned %s", _name.c_str());
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
    const float cosTheta = std::cos(_current->theta);
    const float sinTheta = std::sin(_current->theta);
    const float dx = x - _current->x;
    const float dy = y - _current->y;

    // Calculate goal distance and return true if reach goal
    constexpr float goalTolerance = 0.01;
    bool isReachGoal = false;
    if (dx * dx + dy * dy <= goalTolerance * goalTolerance) {
        isReachGoal = true;
        return isReachGoal;
    }

    // Transform to turtle body frame
    const float txG = dx * cosTheta + dy * sinTheta;
    const float tyG = -dx * sinTheta + dy * cosTheta;

    // Simple proposional control
    constexpr float kLin = 2.0;
    constexpr float kAng = 2.0;

    const float linearX = kLin * txG;
    const float linearY = kLin * tyG;
    const float headingError = std::atan2(linearY, linearX);
    const float angular = kAng * headingError;

    geometry_msgs::msg::Twist msg;
    msg.linear.x = linearX;
    msg.angular.z = angular;
    _velPub->publish(msg);

    return isReachGoal;
}

} // namespace turtlesim