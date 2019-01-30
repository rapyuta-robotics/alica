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
    CNPointEgo()
        : CNPointEgo(0, 0, 0){};
    CNPointEgo(double x, double y, double z = 0);
    CNPointEgo(const CNPointEgo& obj);
    virtual ~CNPointEgo();

    virtual std::string toString() const;

    CNPointAllo toAllo(const CNPositionAllo& origin) const;
    double distanceTo(const CNPointEgo& other) const;

    CNPointEgo operator+(const CNPointEgo& right) const;
    CNPointEgo operator+(const CNVecEgo& right) const;
    CNPointEgo operator-(const CNVecEgo& right) const;
    CNVecEgo operator-(const CNPointEgo& right) const;
};

} /* namespace geometry */
