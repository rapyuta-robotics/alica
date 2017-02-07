#pragma once

#include "CNVecTemplate.h"

namespace geometry
{

class CNVecEgo;
class CNPositionAllo;
class CNVecEgo;

class CNVecAllo : public CNVecTemplate<CNVecAllo>
{
  public:
	CNVecAllo() : CNVecAllo(0, 0, 0) {};
	CNVecAllo(double x, double y, double z = 0);
    virtual ~CNVecAllo();

    std::string toString();

    std::shared_ptr<CNVecEgo> toEgo(CNPositionAllo &origin);
};

} /* namespace geometry */
