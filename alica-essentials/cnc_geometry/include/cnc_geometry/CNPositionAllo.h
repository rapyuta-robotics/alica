#pragma once

#include <cnc_geometry/CNVecAllo.h>
#include "CNPositionTemplate.h"
#include "CNPositionAllo.h"

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

std::shared_ptr<CNPositionAllo> operator+(const std::shared_ptr<CNPositionAllo> &left, const std::shared_ptr<CNVecAllo> &right);
std::shared_ptr<CNPositionAllo> operator-(const std::shared_ptr<CNPositionAllo> &left, const std::shared_ptr<CNVecAllo> &right);

} /* namespace geometry */
