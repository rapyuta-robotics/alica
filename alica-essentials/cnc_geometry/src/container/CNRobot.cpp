/*
 * CNRobot.cpp
 *
 *  Created on: Feb 27, 2016
 *      Author: Carpe Noctem
 */

#include "container/CNRobot.h"

namespace geometry
{

CNRobot::CNRobot()
{
	this->id = 0;
    this->position = CNPositionAllo();
	this->velocity = CNVec2DEgo();
    this->radius = 0;
    this->certainty = 0;
}

CNRobot::~CNRobot() {}

string CNRobot::toString()
{
    stringstream ss;
    ss << "CNRobot: position: (" << this->position.toString() << ")" << " velocity: (" << this->velocity.toString() << ") id: " << this->id << endl;
    return ss.str();
}

} /* namespace geometry */
