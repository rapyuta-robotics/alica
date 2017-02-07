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

    CNPositionEgo toEgo(CNPositionAllo &origin);

    CNPositionAllo operator+(const CNVecAllo &right);
    CNPositionAllo operator-(const CNVecAllo &right);
};

} /* namespace geometry */
