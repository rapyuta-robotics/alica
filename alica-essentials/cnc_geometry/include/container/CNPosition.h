/*
 * CNPosition.h
 *
 *  Created on: 19.11.2014
 *      Author: tobi
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_CONTAINER_CNPOSITION_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_CONTAINER_CNPOSITION_H_

#include "geometry_msgs/Pose2D.h"
#include "container/CNPoint2D.h"
using namespace std;

namespace geometry {
	class CNPosition : public geometry_msgs::Pose2D {
	public:
		CNPosition() : CNPosition(0,0,0) {}
		CNPosition(double x, double y, double theta);
		virtual ~CNPosition();
		string toString();
		double distanceTo(shared_ptr<CNPoint2D> point);
		shared_ptr<CNPoint2D> operator+(const shared_ptr<CNPoint2D>& right);
		shared_ptr<CNPoint2D> operator-(const shared_ptr<CNPoint2D>& right);
		shared_ptr<CNPoint2D> getPoint();
	};

}

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_CONTAINER_CNPOSITION_H_ */
