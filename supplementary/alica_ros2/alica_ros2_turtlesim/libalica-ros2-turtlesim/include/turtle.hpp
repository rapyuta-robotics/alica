#pragma once

#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>

namespace turtlesim
{

class ALICATurtleWorldModel;

/*
    ALICATurtleWoroldModel
    - Turtle control classfor ALICA ros turtlesim which interface between ALICA and ROS.
    - A few class is static since one robot has one world model
    - ROS:
        - Publish: turtleX/cmd_vel
        - Subscribe: turtleX/pose
        - ServiceClient: turtleX/teleport_absolute
        - Parameter: name
*/
class ALICATurtle
{
public:
    ALICATurtle(ALICATurtleWorldModel* wm);
    void teleport(const float x, const float y);         // teleport turtle to (x,y)
    bool moveTowardGoal(const float x, const float y); // publish cmd_vel based on input(x,y) and current pose
    bool moveTowardGoal() const;                       // publish cmd_vel based on goal and current pose

    bool getInit() const { return _initTrigger; };
    void setInit(const bool input) { _initTrigger = input; };

private:
    std::string _name;                                                 // name of turtle
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _velPub;  // publish cmd_vel to the turtlesim
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr _poseSub;   // subscribe turtleX/pose from the turtlesim
    rclcpp::Client<srv::TeleportAbsolute>::SharedPtr _teleportClient; // client of teleportAbsolute service
    msg::Pose _current;                                                // current position
    msg::Pose _goal;                                                   // goal position
    rclcpp::executors::MultiThreadedExecutor _spinner;
    std::thread _spinThread;
    rclcpp::Node::SharedPtr _nh;
    ALICATurtleWorldModel* _wm;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _initTriggerSub; // user input for initialize,
    bool _initTrigger;                                                     // become true when /init topic published
};
} // namespace turtlesim
