/*
 * CNPoint2DEgo.cpp
 *
 *  Created on: 07.11.2016
 *      Author: Philipp Mandler
 */

#include "container/CNPoint2DEgo.h"
#include <sstream>

#include "container/CNPoint2DAllo.h"
#include "container/CNPositionAllo.h"

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
	shared_ptr<CNPoint2DAllo> allo = make_shared<CNPoint2DAllo>();

	// rotate rel point around origin -> rel point with allo orientation
	double s = sin(me.theta);
	double c = cos(me.theta);

	double x = c * this->x - s * this->y;
	double y = s * this->x - c * this->y;

	// sum me pos and rel pos -> allo pos with allo rotaion
	allo->x = x + me.x;
	allo->y = y + me.y;

	return allo;
}

} /* namespace geometry */
