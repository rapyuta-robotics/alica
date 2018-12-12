#pragma once

#include "CNVecTemplate.h"

namespace geometry {

class CNVecEgo;
class CNPositionAllo;
class CNVecEgo;

class CNVecAllo : public CNVecTemplate<CNVecAllo> {
public:
    CNVecAllo()
            : CNVecAllo(0, 0, 0){};
    CNVecAllo(double x, double y, double z = 0);
    CNVecAllo(const CNVecAllo& obj);
    virtual ~CNVecAllo();

    virtual std::string toString() const;

    CNVecEgo toEgo(CNPositionAllo& origin) const;
};

} /* namespace geometry */
