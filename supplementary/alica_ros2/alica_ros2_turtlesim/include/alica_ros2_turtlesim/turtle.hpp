#ifndef ALICA_TURTLE_SIM_TURTLE_HPP
#define ALICA_TURTLE_SIM_TURTLE_HPP

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>

namespace turtlesim
{
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
    ALICATurtle(rclcpp::Node::SharedPtr priv_nh);
    void teleport(const float x, const float y);         // teleport turtle to (x,y)
    bool move_toward_goal(const float x, const float y); // publish cmd_vel based on input(x,y) and current pose
    bool move_toward_goal() const;                       // publish cmd_vel based on goal and current pose
    msg::Pose get_current_pose() const { return _current; };

private:
    void pose_sub_callback(const msg::Pose::ConstSharedPtr msg);       // callback of /pose from the turtlesim
    std::string _name;                                                 // name of turtle
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _vel_pub;  // publish cmd_vel to the turtlesim
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr _pose_sub;   // subscribe turtleX/pose from the turtlesim
    rclcpp::Client<srv::TeleportAbsolute>::SharedPtr _teleport_client; // client of teleportAbsolute service
    msg::Pose _current;                                                // current position
    msg::Pose _goal;                                                   // goal position
    rclcpp::Node::SharedPtr _priv_nh;
};
} // namespace turtlesim

#endif /* ALICA_TURTLE_ALICA_TURTLE_HPP */
