#include "alica_ros_turtlesim/turtle_ros1_interfaces.hpp"

#include <engine/blackboard/Blackboard.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/TeleportAbsolute.h>

namespace turtlesim
{

TurtleRos1Interfaces::TurtleRos1Interfaces(const std::string& name)
        : TurtleInterfaces(name)
        , _name(name)
{
    // initialize publisher, subscriber and service client.
    ros::NodeHandle nh("~");
    _velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    _poseSub = nh.subscribe("pose", 1, &TurtleRos1Interfaces::poseSubCallback, this);
    _teleportClient = nh.serviceClient<TeleportAbsolute>("teleport_absolute");
    _spawnClient = nh.serviceClient<Spawn>("/spawn");
}

bool TurtleRos1Interfaces::teleport(float x, float y)
{
    TeleportAbsolute srv;
    srv.request.x = x;
    srv.request.y = y;
    if (_teleportClient.waitForExistence() && _teleportClient.call(srv)) {
        ROS_INFO_STREAM("Teleported to (" << x << ", " << y << ")");
        return true;
    } else {
        ROS_ERROR_STREAM("Failed to teleport to (" << x << ", " << y << ")");
        return false;
    }
}

bool TurtleRos1Interfaces::spawn()
{
    turtlesim::Spawn spawnSrv;
    spawnSrv.request.x = 5;
    spawnSrv.request.y = 5;
    spawnSrv.request.theta = 0;
    spawnSrv.request.name = _name;
    if (_spawnClient.waitForExistence() && _spawnClient.call(spawnSrv)) {
        return true;
    } else {
        return false;
    }
}

void TurtleRos1Interfaces::poseSubCallback(const PoseConstPtr& msg)
{
    _currentPose = msg;
}

void TurtleRos1Interfaces::rotate(const float dYaw)
{
    geometry_msgs::Twist msg;
    msg.angular.z = dYaw;
    _velPub.publish(msg);
}

bool TurtleRos1Interfaces::moveTowardPosition(float x, float y) const
{
    if (!_currentPose) {
        ROS_WARN_THROTTLE(5, "Waiting for valid pose");
        // Wait until we have a valid pose
        return false;
    }
    // Transform goal position into coordinates of turtle body frame
    float cosTheta = std::cos(_currentPose->theta);
    float sinTheta = std::sin(_currentPose->theta);
    float dx = x - _currentPose->x;
    float dy = y - _currentPose->y;

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

void TurtleRos1Interfaces::subOnMsg(const std::string& topic, alica::Blackboard* globalBlackboard)
{
    _subOnMsg = ros::NodeHandle("~").subscribe<std_msgs::String>(topic, 1, [&](const std_msgs::StringConstPtr& msg) {
        alica::LockedBlackboardRW gb(*globalBlackboard);
        gb.set("msg", msg->data);
    });
}

void TurtleRos1Interfaces::subOnTrigger(const std::string& topic, alica::Blackboard* globalBlackboard)
{
    _subOnTrigger = ros::NodeHandle("~").subscribe<std_msgs::Empty>(topic, 1, [&](const std_msgs::EmptyConstPtr& msg) {
        alica::LockedBlackboardRW gb(*globalBlackboard);
        gb.set("triggered", true);
    });
}

} // namespace turtlesim
