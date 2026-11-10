#pragma once

#include <rclcpp/qos.hpp>
#include <rclcpp/version.h>

// The turtlesim interfaces moved from the turtlesim package to turtlesim_msgs after Humble
#if __has_include(<turtlesim_msgs/msg/pose.hpp>)
#include <turtlesim_msgs/msg/pose.hpp>
#include <turtlesim_msgs/srv/kill.hpp>
#include <turtlesim_msgs/srv/spawn.hpp>
#include <turtlesim_msgs/srv/teleport_absolute.hpp>

namespace turtlesim
{
namespace msg = turtlesim_msgs::msg;
namespace srv = turtlesim_msgs::srv;
} // namespace turtlesim
#else
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#endif

namespace turtlesim
{
// create_client takes rclcpp::QoS after Humble, and only rmw_qos_profile_t in Humble
inline auto servicesQos()
{
#if RCLCPP_VERSION_GTE(17, 0, 0)
    return rclcpp::ServicesQoS();
#else
    return rmw_qos_profile_services_default;
#endif
}
} // namespace turtlesim
