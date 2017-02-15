#pragma once

#include "CNVecAllo.h"
#include "CNPointAllo.h"
#include "CNPositionTemplate.h"

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

    CNPositionEgo toEgo(const CNPositionAllo &origin) const;
    CNPointAllo getPoint() const;
    double distanceTo(const CNPointAllo &other) const;

    CNPositionAllo operator+(const CNVecAllo &right) const;
    CNPositionAllo operator-(const CNVecAllo &right) const;
};

} /* namespace geometry */
