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

    virtual std::string toString() const;

    CNPointEgo toEgo(CNPositionAllo &origin) const;
    double distanceTo(const CNPointAllo &other) const;

    CNPointAllo operator+(const CNVecAllo &right) const;
    CNPointAllo operator-(const CNVecAllo &right) const;
    CNVecAllo operator-(const CNPointAllo &right) const;
};

} /* namespace geometry */
