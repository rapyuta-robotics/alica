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
	CNPointEgo(const CNPointEgo &obj);
    virtual ~CNPointEgo();

    std::string toString();

    CNPointAllo toAllo(const CNPositionAllo &origin);
    double distanceTo(const CNPointEgo &other);

    CNPointEgo operator+(const CNVecEgo &right);
    CNPointEgo operator-(const CNVecEgo &right);
    CNVecEgo operator-(const CNPointEgo &right);
};

} /* namespace geometry */
