#pragma once

#include <rclcpp/rclcpp.hpp>
// #include <turtlesim/msg/pose.hpp>
#include <string>
#include <turtle_interfaces.hpp>

#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>

namespace turtlesim
{
class ALICATurtleWorldModel;
/*
    - Turtle control class for ALICA ros turtlesim which interfaces between ALICA and ROS
    - ROS:
        - Publish: turtleX/cmd_vel
        - Subscribe: turtleX/pose
        - ServiceClient: turtleX/teleport_absolute
        - Parameter: name
*/
class TurtleRos2Interfaces : public TurtleInterfaces
{
public:
    TurtleRos2Interfaces(const std::string& name);
    bool teleport(const float x, const float y) override;                 // teleport turtle to (x,y)
    bool spawn() override;                                                // Spawn the turtle in the middle of the map
    bool moveTowardPosition(const float x, const float y) const override; // publish cmd_vel based on input(x,y) and current pose
    void rotate(const float dYaw) override;

private:
    std::string _name;                                                // name of turtle
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _velPub;  // publish cmd_vel to the turtlesim
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr _poseSub;   // subscribe turtleX/pose from the turtlesim
    rclcpp::Client<srv::TeleportAbsolute>::SharedPtr _teleportClient; // client of teleportAbsolute service
    rclcpp::Client<srv::Spawn>::SharedPtr _spawnClient;
    msg::Pose::ConstSharedPtr _current; // current position
    msg::Pose _goal;                    // goal position
    rclcpp::executors::MultiThreadedExecutor _spinner;
    std::thread _spinThread;
    rclcpp::Node::SharedPtr _nh;
    ALICATurtleWorldModel* _wm;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _initTriggerSub; // user input for initialize,
    bool _initTrigger;
};

} // namespace turtlesim
