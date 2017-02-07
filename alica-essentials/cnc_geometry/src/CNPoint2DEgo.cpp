/*
 * CNPoint2DEgo.cpp
 *
 *  Created on: 07.11.2016
 *      Author: Philipp Mandler
 */

#include "cnc_geometry/container/CNPoint2DEgo.h"
#include "cnc_geometry/container/CNPoint2DAllo.h"
#include "cnc_geometry/container/CNPositionAllo.h"
#include "cnc_geometry/container/CNVec2DEgo.h"

#include <sstream>

using std::string;
using std::shared_ptr;

namespace geometry
{

CNPoint2DEgo::CNPoint2DEgo(double x, double y)
{
    this->x = x;
    this->y = y;
}

CNPoint2DEgo::~CNPoint2DEgo()
{
}

string CNPoint2DEgo::toString()
{
    std::stringstream ss;
    ss << "CNPoint2DEgo: X: " << this->x << " Y: " << this->y << std::endl;
    return ss.str();
}

shared_ptr<CNPoint2DAllo> CNPoint2DEgo::toAllo(CNPositionAllo &me)
{
    shared_ptr<CNPoint2DAllo> allo = std::make_shared<CNPoint2DAllo>();

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

shared_ptr<CNPoint2DEgo> operator+(const shared_ptr<CNPoint2DEgo> &left, const shared_ptr<CNVec2DEgo> &right)
{
    return std::make_shared<CNPoint2DEgo>(left->x + right->x, left->y + right->y);
}

shared_ptr<CNPoint2DEgo> operator-(const shared_ptr<CNPoint2DEgo> &left, const shared_ptr<CNVec2DEgo> &right)
{
    return std::make_shared<CNPoint2DEgo>(left->x - right->x, left->y - right->y);
}

} /* namespace geometry */
