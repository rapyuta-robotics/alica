#pragma once

#include <ros/ros.h>
#include <turtlesim/Pose.h>

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
class ALICATurtle
{
public:
    ALICATurtle(const std::string& name);
    void teleport(const float x, const float y);       // teleport turtle to (x,y)
    bool moveTowardGoal(const float x, const float y); // publish cmd_vel based on input(x,y) and current pose
    bool moveTowardGoal() const;                       // publish cmd_vel based on goal and current pose
    Pose getCurrentPose() const { return _current; };

private:
    void poseSubCallback(const PoseConstPtr& msg); // callback of /pose from the turtlesim
    std::string _name;                             // name of turtle
    ros::Publisher _velPub;                        // publish cmd_vel to the turtlesim
    ros::Subscriber _poseSub;                      // subscribe turtleX/pose from the turtlesim
    ros::ServiceClient _teleportClient;            // client of teleportAbsolute service
    Pose _current;                                 // current position
    Pose _goal;                                    // goal position
};

} // namespace turtlesim
