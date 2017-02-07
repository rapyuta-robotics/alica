/*
 * CNVec2DEgo.cpp
 *
 *  Created on: 07.11.2016
 *      Author: Philipp Mandler
 */

#include "cnc_geometry/CNVec2DEgo.h"

#include <sstream>

#include "cnc_geometry/CNVec2DAllo.h"
#include "cnc_geometry/CNPositionAllo.h"

using std::endl;
using std::string;
using std::shared_ptr;
using std::make_shared;

namespace geometry
{

CNVec2DEgo::CNVec2DEgo(double x, double y)
{
	this->x = x;
	this->y = y;
}

CNVec2DEgo::~CNVec2DEgo() {}

string CNVec2DEgo::toString()
{
    std::stringstream ss;
    ss << "CNVec2DEgo: x: " << this->x << " y: " << this->y << endl;
    return ss.str();
}

shared_ptr<CNVec2DAllo> CNVec2DEgo::toAllo(CNPositionAllo &me)
{
	shared_ptr<CNVec2DAllo> allo = make_shared<CNVec2DAllo>();

	// rotate rel point around origin -> rel point with allo orientation
	double s = sin(me.theta);
	double c = cos(me.theta);

	double x = c * this->x - s * this->y;
	double y = s * this->x - c * this->y;

	return allo;
}

} /* namespace geometry */
