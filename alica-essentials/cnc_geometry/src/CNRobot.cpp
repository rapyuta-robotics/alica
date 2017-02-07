/*
 * CNRobot.cpp
 *
 *  Created on: Feb 27, 2016
 *      Author: Carpe Noctem
 */

#include "cnc_geometry/CNRobot.h"

namespace geometry
{

CNRobot::CNRobot() : CNPositionAllo()
{
    this->radius = 0;
    this->id = 0;
    this->certainty = 0;
    this->rotation = 0;
    this->opposer = std::make_shared<std::vector<int>>();
    this->supporter = std::make_shared<std::vector<int>>();
}

CNRobot::~CNRobot()
{
}

/**
 * Creates a string representation of this robot.
 * @return the string representing the robot.
 */
std::string CNRobot::toString()
{
    std::stringstream ss;
    ss << "CNRobot: ID: " << this->id << " Pose X: " << this->x << " Y: " << this->y << " Orientation: " << this->theta << " Velocity X: " << this->velocity.x
       << " Y: " << this->velocity.y << std::endl;
    return ss.str();
}

}
