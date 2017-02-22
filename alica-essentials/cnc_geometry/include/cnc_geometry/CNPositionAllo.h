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

    virtual std::string toString() const;

    CNPositionEgo toEgo(CNPositionAllo &origin) const;
    double distanceTo(CNPointAllo &pos) const;
    CNPointAllo getPoint() const;

    CNPositionAllo operator+(const CNVecAllo &right) const;
    CNPositionAllo operator-(const CNVecAllo &right) const;
    CNVecAllo operator-(const CNPositionAllo &right) const;
    CNVecAllo operator-(const CNPointAllo &right) const;
};

} /* namespace geometry */
