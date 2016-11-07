/*
 * CNPoint2DEgo.cpp
 *
 *  Created on: 07.11.2016
 *      Author: Philipp Mandler
 */

#include "container/CNPoint2DEgo.h"
#include <sstream>

#include "container/CNPoint2DAllo.h"

namespace geometry
{

CNPoint2DEgo::CNPoint2DEgo(double x, double y)
{
	this->x = x;
	this->y = y;
}

CNPoint2DEgo::~CNPoint2DEgo() {}

string CNPoint2DEgo::toString()
{
    stringstream ss;
    ss << "CNPoint2DEgo: x: " << this->x << " y: " << this->y << endl;
    return ss.str();
}

shared_ptr<CNPoint2DAllo> CNPoint2DEgo::toAllo(CNPositionAllo &me)
{
    return nullptr; // TODO
}

} /* namespace geometry */
