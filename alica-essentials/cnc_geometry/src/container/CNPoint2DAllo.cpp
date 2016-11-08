/*
 * CNPoint2DAllo.cpp
 *
 *  Created on: 04.11.2016
 *      Author: Philipp Mandler
 */

#include "container/CNPoint2DAllo.h"
#include <sstream>

#include "container/CNPoint2DEgo.h"
#include "container/CNPositionAllo.h"

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
    shared_ptr<CNPoint2DEgo> ego = make_shared<CNPoint2DEgo>();

    // sub me pos from allo pos -> rel pos with allo orientation
    double relX = this->x - me.x;
    double relY = this->y - me.y;

    // rotate rel point around origin -> rel point with ego orientation
    double s = sin(-me.theta);
    double c = cos(-me.theta);

    ego->x = c * relX - s * relY;
    ego->y = s * relX - c * relY;

    return ego;
}

} /* namespace geometry */
