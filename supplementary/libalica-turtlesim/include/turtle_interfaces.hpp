#pragma once

// #include <ros/ros.h>
// #include <turtlesim/Pose.h>

namespace turtlesim
{
/*
    - Turtle control class for ALICA ros turtlesim which interfaces between ALICA and ROS
    - ROS:
        - Publish: turtleX/cmd_vel
        - Subscribe: turtleX/pose
        - ServiceClient: turtleX/teleport_absolute
        - Parameter: name
*/
class TurtleInterfaces
{
public:
    TurtleInterfaces(const std::string& name){};
    virtual bool teleport(const float x, const float y) = 0;                 // teleport turtle to (x,y)
    virtual bool spawn() = 0;                                                // Spawn the turtle in the middle of the map
    virtual bool moveTowardPosition(const float x, const float y) const = 0; // publish cmd_vel based on input(x,y) and current pose
    // PoseConstPtr getCurrentPose() const { return _currentPose; }; // Retrieve current pose if available
    void rotate(const float dYaw);

private:
    // void poseSubCallback(const PoseConstPtr& msg); // callback of /pose from the turtlesim
    // ros::Publisher _velPub;                        // publish cmd_vel to the turtlesim
    // ros::Subscriber _poseSub;                      // subscribe turtleX/pose from the turtlesim
    // ros::ServiceClient _teleportClient;            // client of teleportAbsolute service
    // ros::ServiceClient _spawnClient;
    // PoseConstPtr _currentPose; // current position
    std::string _name;
};

} // namespace turtlesim
