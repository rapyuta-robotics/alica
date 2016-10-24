/*
 * CNRobot.h
 *
 *  Created on: Feb 27, 2016
 *      Author: Carpe Noctem
 */

#ifndef SUPPLEMENTARY_CNC_GEOMETRY_SRC_CONTAINER_CNROBOT_H_
#define SUPPLEMENTARY_CNC_GEOMETRY_SRC_CONTAINER_CNROBOT_H_

#include "container/CNPositionAllo.h"
#include "container/CNVelocity2DEgo.h"

namespace geometry
{

class CNRobot
{
  public:
    CNRobot();
    virtual ~CNRobot();

	string toString();

	int id;
	CNPositionAllo position;
	CNVelocity2DEgo velocity;
	double radius;
    shared_ptr<vector<int>> opposer;
    shared_ptr<vector<int>> supporter;
    double certainty;
};

} /* namespace geometry */

#endif /* SUPPLEMENTARY_CNC_GEOMETRY_SRC_CONTAINER_CNROBOT_H_ */
