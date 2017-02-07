#pragma once

#include "CNPointTemplate.h"

namespace geometry
{

class CNPointAllo;
class CNPositionAllo;
class CNVecEgo;

class CNPointEgo : public CNPointTemplate<CNPointEgo>
{
  public:
	CNPointEgo() : CNPointEgo(0, 0, 0) {};
	CNPointEgo(double x, double y, double z = 0);
    virtual ~CNPointEgo();

    std::string toString();

    CNPointAllo toAllo(CNPositionAllo &origin);

    CNPointEgo operator+(const CNVecEgo &right);
    CNPointEgo operator-(const CNVecEgo &right);
};

} /* namespace geometry */
