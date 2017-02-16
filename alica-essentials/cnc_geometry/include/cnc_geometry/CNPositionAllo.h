#pragma once

#include <cnc_geometry/CNVecAllo.h>
#include "cnc_geometry/CNPointAllo.h"
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
    CNPositionAllo(const CNPositionAllo& obj);
    virtual ~CNPositionAllo();

    std::string toString();

    CNPositionEgo toEgo(CNPositionAllo &origin);
    double distanceTo(CNPointAllo &pos);
    CNPointAllo getPoint();

    CNPositionAllo operator+(const CNVecAllo &right);
    CNPositionAllo operator-(const CNVecAllo &right);
    CNVecAllo operator-(const CNPositionAllo &right);
    CNVecAllo operator-(const CNPointAllo &right);
};

} /* namespace geometry */
