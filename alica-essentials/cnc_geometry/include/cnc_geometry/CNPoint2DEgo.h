/*
 * CNPoint2DEgo.h
 *
 *  Created on: 07.11.2016
 *      Author: Philipp Mandler
 */

#pragma once

#include "container/CNPoint2DTemplate.h"

namespace geometry
{

class CNPoint2DAllo;
class CNPositionAllo;
class CNVec2DEgo;

class CNPoint2DEgo : public CNPoint2DTemplate<CNPoint2DEgo>
{
  public:
	CNPoint2DEgo() : CNPoint2DEgo(0, 0) {};
	CNPoint2DEgo(double x, double y);
    virtual ~CNPoint2DEgo();

    std::string toString();

    std::shared_ptr<CNPoint2DAllo> toAllo(CNPositionAllo &origin);

};

std::shared_ptr<CNPoint2DEgo> operator+(const std::shared_ptr<CNPoint2DEgo> &left, const std::shared_ptr<CNVec2DEgo> &right);
std::shared_ptr<CNPoint2DEgo> operator-(const std::shared_ptr<CNPoint2DEgo> &left, const std::shared_ptr<CNVec2DEgo> &right);

} /* namespace geometry */
