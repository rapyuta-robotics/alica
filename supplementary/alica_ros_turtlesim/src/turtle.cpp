#include "alica_ros_turtlesim/turtle.hpp"
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>

namespace turtlesim {

ALICATurtle::ALICATurtle(ros::NodeHandle& priv_nh) {
    priv_nh.getParam("name", _name);

    // initialize publisher, subscriber and service client.
    _vel_pub = priv_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    _pose_sub = priv_nh.subscribe("pose", 1, &ALICATurtle::pose_sub_callback, this);
    _teleport_client = priv_nh.serviceClient<TeleportAbsolute>("teleport_absolute");
}

void ALICATurtle::teleport(float x, float y) {
    TeleportAbsolute srv;
    srv.request.x = x;
    srv.request.y = y;
    if (_teleport_client.waitForExistence() && _teleport_client.call(srv)) {
        ROS_INFO_STREAM(_name << " was teleported to (" << x << ", " << y << ")");
    } else {
        ROS_ERROR_STREAM("Failed to teleport " << _name << " to (" << x << ", " << y << ")");
    }
}

void ALICATurtle::pose_sub_callback(const PoseConstPtr& msg) {
    _current = *msg;
}

bool ALICATurtle::move_toward_goal(float x, float y) {
    _goal.x = x;
    _goal.y = y;
    return move_toward_goal();
}
bool ALICATurtle::move_toward_goal() const {
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

    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular;
    _vel_pub.publish(msg);

    return is_reachGoal;
}

}  // namespace turtlesim
