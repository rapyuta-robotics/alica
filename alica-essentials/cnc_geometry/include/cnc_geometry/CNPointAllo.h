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
    virtual ~CNPointAllo();

    std::string toString();

    CNPointEgo toEgo(CNPositionAllo &origin);

    CNPointAllo operator+(const CNVecAllo &right);
    CNPointAllo operator-(const CNVecAllo &right);
};

} /* namespace geometry */
