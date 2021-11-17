#ifndef ALICA_TURTLE_SIM_TURTLE_HPP
#define ALICA_TURTLE_SIM_TURTLE_HPP

#include <ros/ros.h>
#include <turtlesim/Pose.h>

namespace turtlesim {
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
class ALICATurtle {
public:
    ALICATurtle(ros::NodeHandle& priv_nh);
    void teleport(const float x, const float y);          // teleport turtle to (x,y)
    bool move_toward_goal(const float x, const float y);  // publish cmd_vel based on input(x,y) and current pose
    bool move_toward_goal() const;                        // publish cmd_vel based on goal and current pose
    Pose get_current_pose() const { return _current; };

private:
    void pose_sub_callback(const PoseConstPtr& msg);  // callback of /pose from the turtlesim
    std::string _name;                                // name of turtle
    ros::Publisher _vel_pub;                          // publish cmd_vel to the turtlesim
    ros::Subscriber _pose_sub;                        // subscribe turtleX/pose from the turtlesim
    ros::ServiceClient _teleport_client;              // client of teleportAbsolute service
    Pose _current;                                    // current position
    Pose _goal;                                       // goal position
};

}  // namespace turtlesim

#endif /* ALICA_TURTLE_ALICA_TURTLE_HPP */
