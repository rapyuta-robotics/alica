/*
 * CNPositionEgo.cpp
 *
 *  Created on: 24.10.2016
 *      Author: Philipp Mandler
 */

#include "cnc_geometry/CNPositionEgo.h"
#include <cnc_geometry/CNVecEgo.h>

#include <sstream>

#include "cnc_geometry/CNPositionAllo.h"
#include <cnc_geometry/Calculator.h>

namespace geometry
{

CNPositionEgo::CNPositionEgo(double x, double y, double theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
}

CNPositionEgo::CNPositionEgo(const CNPositionEgo& obj)
{
    this->x = obj.x;
    this->y = obj.y;
    this->theta = obj.theta;
}

CNPositionEgo::~CNPositionEgo() {}

std::string CNPositionEgo::toString() const
{
    std::stringstream ss;
    ss << "CNPositionEgo: X: " << this->x << " Y: " << this->y << " Orientation: " << this->theta << std::endl;
    return ss.str();
}
CNPointEgo CNPositionEgo::getPoint() const
{
    return CNPointEgo(this->x, this->y, 0);
}
CNPositionAllo CNPositionEgo::toAllo(CNPositionAllo& me) const
{
    auto allo = CNPositionAllo();
    allo.x = me.x;
    allo.y = me.y;
    // rotate rel point around origin -> rel point with allo orientation
    double s = sin(me.theta);
    double c = cos(me.theta);

    // sum me pos and rel pos -> allo pos with allo rotaion
    allo.x += c * this->x - s * this->y;
    allo.y += s * this->x + c * this->y;

    // rotate theta
    allo.theta = normalizeAngle(this->theta + me.theta);

    return allo;
}

CNPositionEgo CNPositionEgo::operator+(const CNVecEgo& right) const
{
    return CNPositionEgo(this->x + right.x, this->y + right.y, this->theta);
}

CNPositionEgo CNPositionEgo::operator-(const CNVecEgo& right) const
{
    return CNPositionEgo(this->x - right.x, this->y - right.y, this->theta);
}

} /* namespace geometry */
