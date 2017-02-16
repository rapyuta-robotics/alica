#pragma once

#include "CNPointTemplate.h"

namespace geometry
{

class CNPointEgo;
class CNPositionAllo;
class CNVecAllo;

class CNPointAllo : public CNPointTemplate<CNPointAllo>
{
  public:
	CNPointAllo() : CNPointAllo(0, 0, 0) {};
	CNPointAllo(double x, double y, double z = 0);
	CNPointAllo(const CNPointAllo& obj);
    virtual ~CNPointAllo();

    std::string toString();

    CNPointEgo toEgo(CNPositionAllo &origin);
    double distanceTo(const CNPointAllo &other);

    CNPointAllo operator+(const CNVecAllo &right);
    CNPointAllo operator-(const CNVecAllo &right);
    CNVecAllo operator-(const CNPointAllo &right);
};

} /* namespace geometry */
