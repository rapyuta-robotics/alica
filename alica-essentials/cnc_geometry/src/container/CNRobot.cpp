/*
 * CNRobot.cpp
 *
 *  Created on: Feb 27, 2016
 *      Author: Carpe Noctem
 */

#include "container/CNRobot.h"

namespace geometry {

CNRobot::CNRobot() {
    this->radius = 0;
    this->velocityX = 0;
    this->velocityY = 0;
    this->id = nullptr;
    this->certainty = 0;
    this->rotation = 0;
}

CNRobot::~CNRobot() {}

string CNRobot::toString() {
    stringstream ss;
    ss << "CNRobot: x: " << this->x << " y: " << this->y << " angle: " << this->theta << " velX: " << this->velocityX
       << " velY: " << this->velocityY << " id: " << this->id << endl;
    return ss.str();
}
}  // namespace geometry
