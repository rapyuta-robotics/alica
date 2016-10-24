/*
 * CNVelocity2DAllo.h
 *
 *  Created on: 24.10.2016
 *      Author: Philipp Mandler
 */

#ifndef CNC_GEOMETRY_CONTAINER_CNVELOCITY2DALLO_H_
#define CNC_GEOMETRY_CONTAINER_CNVELOCITY2DALLO_H_

#include "container/CNVec2D.h"

namespace geometry
{

class CNVelocity2DAllo : public CNVec2D
{
  public:
    CNVelocity2DAllo(double x, double y) : CNVec2(x, y);
    CNVelocity2DAllo() : CNVelocity2DAllo(0, 0) {}
    virtual ~CNVelocity2DAllo();

    string toString();

    shared_ptr<CNVelocity2DEgo> toEgo(CNPositionAllo &me);
};

} /* namespace geometry */

#endif /* CNC_GEOMETRY_CONTAINER_CNVELOCITY2DALLO_H_ */
