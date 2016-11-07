/*
 * CNPoint2DAllo.cpp
 *
 *  Created on: 04.11.2016
 *      Author: Philipp Mandler
 */

#include "container/CNPoint2DAllo.h"
#include <sstream>

#include "container/CNPoint2DEgo.h"

namespace geometry
{

CNPoint2DAllo::CNPoint2DAllo(double x, double y)
{
	this->x = x;
	this->y = y;
}

CNPoint2DAllo::~CNPoint2DAllo() {}

string CNPoint2DAllo::toString()
{
    stringstream ss;
    ss << "CNPoint2DAllo: x: " << this->x << " y: " << this->y << endl;
    return ss.str();
}

shared_ptr<CNPoint2DEgo> CNPoint2DAllo::toEgo(CNPositionAllo &me)
{
    return nullptr; // TODO
}

} /* namespace geometry */
