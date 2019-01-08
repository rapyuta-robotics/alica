#pragma once

#include "CNPositionAllo.h"
#include "CNPositionTemplate.h"
#include "CNVecEgo.h"
#include "cnc_geometry/CNPointEgo.h"

namespace geometry
{

class CNPositionAllo;

class CNPositionEgo : public CNPositionTemplate<CNPositionEgo>
{
public:
    CNPositionEgo()
            : CNPositionEgo(0, 0, 0)
    {
    }
    CNPositionEgo(double x, double y, double theta);
    CNPositionEgo(const CNPositionEgo& obj);
    virtual ~CNPositionEgo();

    virtual std::string toString() const;

    CNPositionAllo toAllo(CNPositionAllo& origin) const;

    CNPointEgo getPoint() const;
    CNPositionEgo operator+(const CNVecEgo& right) const;
    CNPositionEgo operator-(const CNVecEgo& right) const;
    CNVecEgo operator-(const CNPositionEgo& right) const;
};

} /* namespace geometry */
