/*
 * CNPositionBase.h
 *
 *  Created on: 24.10.2016
 *      Author: Philipp Mandler
 */

#ifndef CNC_GEOMETRY_CONTAINER_CNPOSITIONBASE_H_
#define CNC_GEOMETRY_CONTAINER_CNPOSITIONBASE_H_

#include "geometry_msgs/Pose2D.h"
#include "container/CNVec2D.h"

namespace geometry
{

using namespace std;

class CNPositionBase : public geometry_msgs::Pose2D
{
  public:
    CNPositionBase() : CNPositionBase(0, 0, 0) {}
    CNPositionBase(double x, double y, double theta);
    virtual ~CNPositionBase();

    virtual string toString();

    shared_ptr<CNVec2D> toPoint();
};

} /* namespace geometry */

#endif /* CNC_GEOMETRY_CONTAINER_CNPOSITIONBASE_H_ */
