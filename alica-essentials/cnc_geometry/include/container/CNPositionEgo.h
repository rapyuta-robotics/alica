/*
 * CNPositionEgo.h
 *
 *  Created on: 24.10.2016
 *      Author: Philipp Mandler
 */

#ifndef CNC_GEOMETRY_CONTAINER_CNPOSITIONEGO_H_
#define CNC_GEOMETRY_CONTAINER_CNPOSITIONEGO_H_

#include "container/CNPositionTemplate.h"
#include "container/CNPositionAllo.h"
#include "container/CNVec2DEgo.h"

namespace geometry
{

class CNPositionAllo;

class CNPositionEgo : public CNPositionTemplate<CNPositionEgo>
{
  public:
    CNPositionEgo() : CNPositionEgo(0, 0, 0) {}
    CNPositionEgo(double x, double y, double theta);
    virtual ~CNPositionEgo();

    string toString();

    shared_ptr<CNPositionAllo> toAllo(CNPositionAllo &origin);

    shared_ptr<CNPositionEgo> operator+(const shared_ptr<CNVec2DEgo> &right);
    shared_ptr<CNPositionEgo> operator-(const shared_ptr<CNVec2DEgo> &right);
};

} /* namespace geometry */

#endif /* CNC_GEOMETRY_CONTAINER_CNPOSITIONEGO_H_ */
