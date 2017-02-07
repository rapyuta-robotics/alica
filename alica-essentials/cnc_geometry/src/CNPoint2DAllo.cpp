/*
 * CNPoint2DAllo.cpp
 *
 *  Created on: 04.11.2016
 *      Author: Philipp Mandler
 */

#include "cnc_geometry/container/CNPoint2DAllo.h"
#include "cnc_geometry/container/CNPoint2DEgo.h"
#include "cnc_geometry/container/CNPositionAllo.h"
#include "cnc_geometry/container/CNVec2DAllo.h"

#include <sstream>

using std::string;
using std::shared_ptr;

namespace geometry
{

CNPoint2DAllo::CNPoint2DAllo(double x, double y)
{
    this->x = x;
    this->y = y;
}

CNPoint2DAllo::~CNPoint2DAllo()
{
}

string CNPoint2DAllo::toString()
{
    std::stringstream ss;
    ss << "CNPoint2DAllo: x: " << this->x << " y: " << this->y << std::endl;
    return ss.str();
}

/**
 * Converts this allocentric 2d point into an egocentric 2d point with respect to
 * the given allocentric position.
 * @param alloPos the allocentric reference position
 * @return an egocentric 2d point with alloPos as origin of ordinates
 */
shared_ptr<CNPoint2DEgo> CNPoint2DAllo::toEgo(CNPositionAllo &alloPos)
{
    shared_ptr<CNPoint2DEgo> ego = std::make_shared<CNPoint2DEgo>();

    // sub me pos from allo pos -> rel pos with allo orientation
    double relX = this->x - alloPos.x;
    double relY = this->y - alloPos.y;

    // rotate rel point around origin -> rel point with ego orientation
    double s = sin(-alloPos.theta);
    double c = cos(-alloPos.theta);

    ego->x = c * relX - s * relY;
    ego->y = s * relX - c * relY;

    return ego;
}

shared_ptr<CNPoint2DAllo> operator+(const shared_ptr<CNPoint2DEgo> &left, const shared_ptr<CNVec2DAllo> &right)
{
    return std::make_shared<CNPoint2DAllo>(left->x + right->x, left->y + right->y);
}

shared_ptr<CNPoint2DAllo> operator-(const shared_ptr<CNPoint2DEgo> &left, const shared_ptr<CNVec2DAllo> &right)
{
    return std::make_shared<CNPoint2DAllo>(left->x - right->x, left->y - right->y);
}

} /* namespace geometry */
