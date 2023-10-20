#pragma once

#include <string>

namespace alica
{
class Blackboard;
}

namespace turtlesim
{
/*
    - Turtle control class for ALICA which provides interface to real world or simulation
    - ROS:
        - Publish: turtleX/cmd_vel
        - Subscribe: turtleX/pose
        - ServiceClient: turtleX/teleport_absolute
        - Parameter: name
*/
class TurtleInterfaces
{
public:
    TurtleInterfaces(const std::string& name)
            : _name(name){};
    virtual bool teleport(const float x, const float y) = 0;                 // teleport turtle to (x,y)
    virtual bool spawn() = 0;                                                // Spawn the turtle in the middle of the map
    virtual bool moveTowardPosition(const float x, const float y) const = 0; // publish cmd_vel based on input(x,y) and current pose
    virtual void rotate(const float dYaw) = 0;                               // publish rotating speed of turtle based on (dYaw)

protected:
    std::string _name;
};

} // namespace turtlesim
