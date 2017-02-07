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

    std::shared_ptr<CNPointEgo> toEgo(CNPositionAllo &origin);
};

std::shared_ptr<CNPointAllo> operator+(const std::shared_ptr<CNPointAllo> &left, const std::shared_ptr<CNVecAllo> &right);
std::shared_ptr<CNPointAllo> operator-(const std::shared_ptr<CNPointAllo> &left, const std::shared_ptr<CNVecAllo> &right);

} /* namespace geometry */
