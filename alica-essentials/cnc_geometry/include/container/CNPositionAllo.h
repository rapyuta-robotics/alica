/*
 * CNPositionAllo.h
 *
 *  Created on: 24.10.2016
 *      Author: Philipp Mandler
 */

#ifndef CNC_GEOMETRY_CONTAINER_CNPOSITIONALLO_H_
#define CNC_GEOMETRY_CONTAINER_CNPOSITIONALLO_H_

#include "container/CNPositionTemplate.h"
#include "container/CNPositionAllo.h"
#include "container/CNVec2DAllo.h"

namespace geometry
{

class CNPositionEgo;

class CNPositionAllo : public CNPositionTemplate<CNPositionAllo>
{
  public:
    CNPositionAllo() : CNPositionAllo(0, 0, 0) {};
    CNPositionAllo(double x, double y, double theta);
    virtual ~CNPositionAllo();

    std::string toString();

    std::shared_ptr<CNPositionEgo> toEgo(CNPositionAllo &origin);
};

std::shared_ptr<CNPositionAllo> operator+(const std::shared_ptr<CNPositionAllo> &left, const std::shared_ptr<CNVec2DAllo> &right);
std::shared_ptr<CNPositionAllo> operator-(const std::shared_ptr<CNPositionAllo> &left, const std::shared_ptr<CNVec2DAllo> &right);

} /* namespace geometry */

#endif /* CNC_GEOMETRY_CONTAINER_CNPOSITIONALLO_H_ */
