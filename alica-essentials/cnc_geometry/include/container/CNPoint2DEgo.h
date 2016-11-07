/*
 * CNPoint2DEgo.h
 *
 *  Created on: 07.11.2016
 *      Author: Philipp Mandler
 */

#ifndef CNC_GEOMETRY_CONTAINER_CNPOINT2DEGO_H_
#define CNC_GEOMETRY_CONTAINER_CNPOINT2DEGO_H_

#include "container/CNPoint2DTemplate.h"

namespace geometry
{

class CNPoint2DAllo;
class CNPositionAllo;

class CNPoint2DEgo : public CNPoint2DTemplate<CNPoint2DEgo>
{
  public:
	CNPoint2DEgo() : CNPoint2DEgo(0, 0) {};
	CNPoint2DEgo(double x, double y);
    virtual ~CNPoint2DEgo();

    string toString();

    shared_ptr<CNPoint2DAllo> toAllo(CNPositionAllo &origin);
};

} /* namespace geometry */

#endif /* CNC_GEOMETRY_CONTAINER_CNPOINT2DEGO_H_ */
