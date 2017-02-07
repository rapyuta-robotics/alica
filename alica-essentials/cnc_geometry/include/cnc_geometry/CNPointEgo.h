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
    virtual ~CNPointEgo();

    std::string toString();

    std::shared_ptr<CNPointAllo> toAllo(CNPositionAllo &origin);

};

std::shared_ptr<CNPointEgo> operator+(const std::shared_ptr<CNPointEgo> &left, const std::shared_ptr<CNVecEgo> &right);
std::shared_ptr<CNPointEgo> operator-(const std::shared_ptr<CNPointEgo> &left, const std::shared_ptr<CNVecEgo> &right);

} /* namespace geometry */
