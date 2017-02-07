/*
 * CNVec2DAllo.cpp
 *
 *  Created on: 07.11.2016
 *      Author: Philipp Mandler
 */

#include <cnc_geometry/CNVecAllo.h>
#include <cnc_geometry/CNVecEgo.h>
#include <sstream>

#include "cnc_geometry/CNPositionAllo.h"

using std::string;
using std::shared_ptr;

namespace geometry
{

CNVecAllo::CNVecAllo(double x, double y, double z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

CNVecAllo::~CNVecAllo() {}

string CNVecAllo::toString()
{
    std::stringstream ss;
    ss << "CNVecAllo: x: " << x << " y: " << y << " z: " << z << std::endl;
    return ss.str();
}

shared_ptr<CNVecEgo> CNVecAllo::toEgo(CNPositionAllo &me)
{
	// TODO: fix
    shared_ptr<CNVecEgo> ego = std::make_shared<CNVecEgo>();

    // rotate rel point around origin -> rel point with ego orientation
    double s = sin(-me.theta);
    double c = cos(-me.theta);

    ego->x = c * this->x - s * this->y;
    ego->y = s * this->x - c * this->y;

    return ego;
}

} /* namespace geometry */
