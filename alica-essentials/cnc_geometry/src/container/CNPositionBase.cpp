/*
 * CNPositionBase.cpp
 *
 *  Created on: 24.10.2016
 *      Author: Philipp Mandler
 */

#include "container/CNPositionBase.h"

#include <sstream>

namespace geometry
{

CNPositionBase::CNPositionBase(double x, double y, double theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
}

CNPositionBase::~CNPositionBase() {}

string CNPositionBase::toString()
{
    stringstream ss;
    ss << "CNPositionBase: x: " << this->x << " y: " << this->y << " theta: " << this->theta << endl;
    return ss.str();
}

shared_ptr<CNVec2D> CNPositionBase::toPoint()
{
    return make_shared<CNVec2D>(this->x, this->y);
}

}
