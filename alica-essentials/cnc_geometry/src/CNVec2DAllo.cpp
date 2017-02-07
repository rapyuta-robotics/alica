/*
 * CNVec2DAllo.cpp
 *
 *  Created on: 07.11.2016
 *      Author: Philipp Mandler
 */

#include "cnc_geometry/CNVec2DAllo.h"

#include <sstream>

#include "cnc_geometry/CNVec2DEgo.h"
#include "cnc_geometry/CNPositionAllo.h"

using std::string;
using std::shared_ptr;

namespace geometry
{

CNVec2DAllo::CNVec2DAllo(double x, double y)
{
	this->x = x;
	this->y = y;
}

CNVec2DAllo::~CNVec2DAllo() {}

string CNVec2DAllo::toString()
{
    std::stringstream ss;
    ss << "CNVec2DAllo: x: " << this->x << " y: " << this->y << std::endl;
    return ss.str();
}

shared_ptr<CNVec2DEgo> CNVec2DAllo::toEgo(CNPositionAllo &me)
{
    shared_ptr<CNVec2DEgo> ego = std::make_shared<CNVec2DEgo>();

    // rotate rel point around origin -> rel point with ego orientation
    double s = sin(-me.theta);
    double c = cos(-me.theta);

    ego->x = c * this->x - s * this->y;
    ego->y = s * this->x - c * this->y;

    return ego;
}

} /* namespace geometry */
