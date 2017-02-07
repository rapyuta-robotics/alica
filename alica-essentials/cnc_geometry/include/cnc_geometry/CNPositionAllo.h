/*
 * CNPositionAllo.h
 *
 *  Created on: 24.10.2016
 *      Author: Philipp Mandler
 */

#pragma once

#include "CNPositionTemplate.h"
#include "CNPositionAllo.h"
#include "CNVec2DAllo.h"

namespace geometry
{

class CNPositionEgo;

class CNPositionAllo : public CNPositionTemplate<CNPositionAllo>
{
  public:
    CNPositionAllo() : CNPositionAllo(0, 0, 0) {};
    CNPositionAllo(double x, double y, double theta);
    virtual ~CNPositionAllo();

    std::string toString();

    std::shared_ptr<CNPositionEgo> toEgo(CNPositionAllo &origin);
};

std::shared_ptr<CNPositionAllo> operator+(const std::shared_ptr<CNPositionAllo> &left, const std::shared_ptr<CNVec2DAllo> &right);
std::shared_ptr<CNPositionAllo> operator-(const std::shared_ptr<CNPositionAllo> &left, const std::shared_ptr<CNVec2DAllo> &right);

} /* namespace geometry */
