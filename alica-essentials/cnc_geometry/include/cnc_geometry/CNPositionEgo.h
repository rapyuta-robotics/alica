#pragma once

#include "CNPositionTemplate.h"
#include "CNPositionAllo.h"
#include "CNVecEgo.h"

namespace geometry
{

class CNPositionAllo;

class CNPositionEgo : public CNPositionTemplate<CNPositionEgo>
{
  public:
    CNPositionEgo() : CNPositionEgo(0, 0, 0) {}
    CNPositionEgo(double x, double y, double theta);
    virtual ~CNPositionEgo();

    std::string toString();

    std::shared_ptr<CNPositionAllo> toAllo(CNPositionAllo &origin);

};

std::shared_ptr<CNPositionEgo> operator+(const std::shared_ptr<CNPositionEgo> &left, const std::shared_ptr<CNVecEgo> &right);
std::shared_ptr<CNPositionEgo> operator-(const std::shared_ptr<CNPositionEgo> &left, const std::shared_ptr<CNVecEgo> &right);

} /* namespace geometry */
