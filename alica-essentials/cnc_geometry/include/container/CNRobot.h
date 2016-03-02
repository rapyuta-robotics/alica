/*
 * CNRobot.h
 *
 *  Created on: Feb 27, 2016
 *      Author: Carpe Noctem
 */

#ifndef SUPPLEMENTARY_CNC_GEOMETRY_SRC_CONTAINER_CNROBOT_H_
#define SUPPLEMENTARY_CNC_GEOMETRY_SRC_CONTAINER_CNROBOT_H_

#include "container/CNPosition.h"

namespace geometry
{

	class CNRobot : public CNPosition
	{
	public:
		CNRobot();
		virtual ~CNRobot();
		double radius;
		double velocityX;
		double velocityY;
		int id;
		shared_ptr<vector<int>> opposer;
		shared_ptr<vector<int>> supporter;
		double certainty;
		double rotation;
		string toString();
	};
}

#endif /* SUPPLEMENTARY_CNC_GEOMETRY_SRC_CONTAINER_CNROBOT_H_ */
