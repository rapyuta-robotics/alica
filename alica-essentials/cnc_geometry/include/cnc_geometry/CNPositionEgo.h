/*
 * CNPositionEgo.h
 *
 *  Created on: 24.10.2016
 *      Author: Philipp Mandler
 */

#pragma once

#include "cnc_geometry/container/CNPositionTemplate.h"
#include "cnc_geometry/container/CNPositionAllo.h"
#include "cnc_geometry/container/CNVec2DEgo.h"

namespace geometry
{

class CNPositionAllo;

class CNPositionEgo : public CNPositionTemplate<CNPositionEgo>
{
  public:
    CNPositionEgo() : CNPositionEgo(0, 0, 0) {}
    CNPositionEgo(double x, double y, double theta);
    virtual ~CNPositionEgo();

    std::string toString();

    std::shared_ptr<CNPositionAllo> toAllo(CNPositionAllo &origin);

};

std::shared_ptr<CNPositionEgo> operator+(const std::shared_ptr<CNPositionEgo> &left, const std::shared_ptr<CNVec2DEgo> &right);
std::shared_ptr<CNPositionEgo> operator-(const std::shared_ptr<CNPositionEgo> &left, const std::shared_ptr<CNVec2DEgo> &right);

} /* namespace geometry */
