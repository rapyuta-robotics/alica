/*
 * CNVec2DEgo.cpp
 *
 *  Created on: 07.11.2016
 *      Author: Philipp Mandler
 */

#include <cnc_geometry/CNVecAllo.h>
#include <cnc_geometry/CNVecEgo.h>
#include <sstream>

#include "cnc_geometry/CNPositionAllo.h"

using std::endl;
using std::make_shared;
using std::shared_ptr;
using std::string;

namespace geometry
{

CNVecEgo::CNVecEgo(double x, double y, double z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

CNVecEgo::CNVecEgo(const CNVecEgo& obj)
{
    this->x = obj.x;
    this->y = obj.y;
    this->z = obj.z;
}

CNVecEgo::~CNVecEgo() {}

string CNVecEgo::toString() const
{
    std::stringstream ss;
    ss << "CNVecEgo: x: " << x << " y: " << y << " z: " << z << endl;
    return ss.str();
}

CNVecAllo CNVecEgo::toAllo(CNPositionAllo& me) const
{
    auto allo = CNVecAllo();

    // rotate rel point around origin -> rel point with allo orientation
    double s = sin(me.theta);
    double c = cos(me.theta);

    allo.x = c * this->x - s * this->y;
    allo.y = s * this->x + c * this->y;
    allo.z = this->z;

    return allo;
}

} /* namespace geometry */
