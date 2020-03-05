#pragma once

#include "CNVecTemplate.h"

namespace geometry
{

class CNVecAllo;
class CNPositionAllo;
class CNVecAllo;

class CNVecEgo : public CNVecTemplate<CNVecEgo>
{
public:
    CNVecEgo()
        : CNVecEgo(0, 0, 0){};
    CNVecEgo(double x, double y, double z = 0);
    CNVecEgo(const CNVecEgo& obj);
    virtual ~CNVecEgo();

    virtual std::string toString() const;

    CNVecAllo toAllo(CNPositionAllo& origin) const;
};

} /* namespace geometry */
