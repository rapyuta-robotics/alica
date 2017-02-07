/*
 * CNPoint2DAllo.h
 *
 *  Created on: 07.11.2016
 *      Author: Philipp Mandler
 */

#pragma once

#include "CNPoint2DTemplate.h"

namespace geometry
{

class CNPoint2DEgo;
class CNPositionAllo;
class CNVec2DAllo;

class CNPoint2DAllo : public CNPoint2DTemplate<CNPoint2DAllo>
{
  public:
	CNPoint2DAllo() : CNPoint2DAllo(0, 0) {};
	CNPoint2DAllo(double x, double y);
    virtual ~CNPoint2DAllo();

    std::string toString();

    std::shared_ptr<CNPoint2DEgo> toEgo(CNPositionAllo &origin);
};

std::shared_ptr<CNPoint2DAllo> operator+(const std::shared_ptr<CNPoint2DAllo> &left, const std::shared_ptr<CNVec2DAllo> &right);
std::shared_ptr<CNPoint2DAllo> operator-(const std::shared_ptr<CNPoint2DAllo> &left, const std::shared_ptr<CNVec2DAllo> &right);

} /* namespace geometry */
