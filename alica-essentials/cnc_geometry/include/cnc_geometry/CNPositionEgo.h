#pragma once

#include "CNPositionTemplate.h"
#include "CNPositionAllo.h"
#include "CNVecEgo.h"

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

    CNPositionAllo toAllo(CNPositionAllo &origin);

    CNPositionEgo operator+(const CNVecEgo &right);
    CNPositionEgo operator-(const CNVecEgo &right);
};

} /* namespace geometry */
