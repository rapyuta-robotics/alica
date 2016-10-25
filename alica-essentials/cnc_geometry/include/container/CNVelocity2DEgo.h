/*
 * CNVelocity2DEgo.h
 *
 *  Created on: 24.10.2016
 *      Author: Philipp Mandler
 */

#ifndef CNC_GEOMETRY_CONTAINER_CNVELOCITY2DEGO_H_
#define CNC_GEOMETRY_CONTAINER_CNVELOCITY2DEGO_H_

#include "container/CNVec2D.h"

namespace geometry
{

class CNVelocity2DAllo;

class CNVelocity2DEgo : public CNVec2D
{
  public:
    CNVelocity2DEgo(double x, double y) : CNVec2D(x, y) {};
    CNVelocity2DEgo() : CNVelocity2DEgo(0, 0) {}
    virtual ~CNVelocity2DEgo();

    string toString();

    shared_ptr<CNVelocity2DAllo> toAllo(CNPositionAllo &me);
};

} /* namespace geometry */

#endif /* CNC_GEOMETRY_CONTAINER_CNVELOCITY2DEGO_H_ */
