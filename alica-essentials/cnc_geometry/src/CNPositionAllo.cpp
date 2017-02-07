/*
 * CNPositionAllo.cpp
 *
 *  Created on: 24.10.2016
 *      Author: Philipp Mandler
 */

#include "cnc_geometry/container/CNPositionAllo.h"

#include <sstream>
#include "cnc_geometry/container/CNPositionEgo.h"
#include "cnc_geometry/container/CNVec2DAllo.h"

using std::string;
using std::shared_ptr;

namespace geometry
{

CNPositionAllo::CNPositionAllo(double x, double y, double theta)
{
	this->x = x;
	this->y = y;
	this->theta = theta;
}

CNPositionAllo::~CNPositionAllo() {}

string CNPositionAllo::toString()
{
    std::stringstream ss;
    ss << "CNPositionAllo: X: " << this->x << " Y: " << this->y << " Orientation: " << this->theta << std::endl;
    return ss.str();
}

shared_ptr<CNPositionEgo> CNPositionAllo::toEgo(CNPositionAllo &me)
{
    shared_ptr<CNPositionEgo> ego = std::make_shared<CNPositionEgo>();

    // sub me pos from allo pos -> rel pos with allo orientation
    double relX = this->x - me.x;
    double relY = this->y - me.y;

    // rotate rel point around origin -> rel point with ego orientation
    double s = sin(-me.theta);
    double c = cos(-me.theta);

    ego->x = c * relX - s * relY;
    ego->y = s * relX - c * relY;

    // rotate theta
    ego->theta = this->theta - me.theta;

    return ego;
}

shared_ptr<CNPositionAllo> operator+(const shared_ptr<CNPositionAllo> &left, const shared_ptr<CNVec2DAllo> &right)
{
	return std::make_shared<CNPositionAllo>(
			left->x + right->x,
			left->y + right->y,
			left->theta);
}

shared_ptr<CNPositionAllo> operator-(const shared_ptr<CNPositionAllo> &left, const shared_ptr<CNVec2DAllo> &right)
{
	return std::make_shared<CNPositionAllo>(
			left->x - right->x,
			left->y - right->y,
			left->theta);
}

} /* namespace geometry */
