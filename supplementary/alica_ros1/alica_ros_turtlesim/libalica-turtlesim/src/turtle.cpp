#include "turtle.hpp"

#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>

namespace turtlesim
{

ALICATurtle::ALICATurtle(const std::string& name)
        : _name(name)
{
    // initialize publisher, subscriber and service client.
    ros::NodeHandle nh("~");
    _velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    _poseSub = nh.subscribe("pose", 1, &ALICATurtle::poseSubCallback, this);
    _teleportClient = nh.serviceClient<TeleportAbsolute>("teleport_absolute");
}

void ALICATurtle::teleport(float x, float y)
{
    TeleportAbsolute srv;
    srv.request.x = x;
    srv.request.y = y;
    if (_teleportClient.waitForExistence() && _teleportClient.call(srv)) {
        ROS_INFO_STREAM(_name << " was teleported to (" << x << ", " << y << ")");
    } else {
        ROS_ERROR_STREAM("Failed to teleport " << _name << " to (" << x << ", " << y << ")");
    }
}

void ALICATurtle::poseSubCallback(const PoseConstPtr& msg)
{
    _current = *msg;
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
    float cosTheta = std::cos(_current.theta);
    float sinTheta = std::sin(_current.theta);
    float dx = _goal.x - _current.x;
    float dy = _goal.y - _current.y;

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

    geometry_msgs::Twist msg;
    msg.linear.x = linearX;
    msg.angular.z = angular;
    _velPub.publish(msg);

    return isReachGoal;
}

} // namespace turtlesim
