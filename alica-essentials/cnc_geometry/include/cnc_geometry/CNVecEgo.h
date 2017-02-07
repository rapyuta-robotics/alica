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
	CNVecEgo() : CNVecEgo(0, 0, 0) {};
	CNVecEgo(double x, double y, double z = 0);
    virtual ~CNVecEgo();

    std::string toString();

    std::shared_ptr<CNVecAllo> toAllo(CNPositionAllo &origin);
};

} /* namespace geometry */
