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
    CNPositionEgo(const CNPositionEgo &obj);
    virtual ~CNPositionEgo();

    virtual std::string toString();

    CNPositionAllo toAllo(CNPositionAllo &origin);

    CNPositionEgo operator+(const CNVecEgo &right);
    CNPositionEgo operator-(const CNVecEgo &right);
    CNVecEgo operator-(const CNPositionEgo &right);
};

} /* namespace geometry */
