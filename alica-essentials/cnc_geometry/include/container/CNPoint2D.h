/*
 * Point2D.h
 *
 *  Created on: 19.11.2014
 *      Author: Tobias Schellien
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_CONTAINER_POINT2D_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_CONTAINER_POINT2D_H_

#include "geometry_msgs/Point.h"
#include <memory>

using namespace std;


namespace geometry {
	class CNPosition;

	class CNPoint2D : public geometry_msgs::Point {
	public:


		CNPoint2D(double x, double y);
		CNPoint2D() : CNPoint2D(0,0) {}

		double length();
		shared_ptr<CNPoint2D> rotate(double radian);
		double angleTo();
		double angleToPoint(shared_ptr<CNPoint2D> point);
		double distanceTo(shared_ptr<CNPoint2D> point);
		shared_ptr<CNPoint2D> alloToEgo(CNPosition& me);
		shared_ptr<CNPoint2D> egoToAllo(CNPosition& me);
		shared_ptr<CNPoint2D> normalize();

		shared_ptr<CNPoint2D> operator*(const double& right);
		shared_ptr<CNPoint2D> operator/(const double& right);
		shared_ptr<CNPoint2D> operator+(const shared_ptr<CNPoint2D>& right);
		shared_ptr<CNPoint2D> operator-(const shared_ptr<CNPoint2D>& right);
		shared_ptr<CNPoint2D> operator+(const shared_ptr<CNPosition>& right);
		shared_ptr<CNPoint2D> operator-(const shared_ptr<CNPosition>& right);

		virtual ~CNPoint2D();
		string toString();
	};

	shared_ptr<CNPoint2D> operator+(const shared_ptr<CNPoint2D>& left, const shared_ptr<CNPoint2D>& right);
	shared_ptr<CNPoint2D> operator-(const shared_ptr<CNPoint2D>& left, const shared_ptr<CNPoint2D>& right);
	shared_ptr<CNPoint2D> operator+(const shared_ptr<CNPoint2D>& left, const shared_ptr<CNPosition>& right);
	shared_ptr<CNPoint2D> operator-(const shared_ptr<CNPoint2D>& left, const shared_ptr<CNPosition>& right);
	shared_ptr<CNPoint2D> operator+(const shared_ptr<CNPosition>& left, const shared_ptr<CNPoint2D>& right);
	shared_ptr<CNPoint2D> operator-(const shared_ptr<CNPosition>& left, const shared_ptr<CNPoint2D>& right);
	shared_ptr<CNPoint2D> operator*(const shared_ptr<CNPoint2D>& left, const double& right);
	shared_ptr<CNPoint2D> operator/(const shared_ptr<CNPoint2D>& left, const double& right);
}
#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_CONTAINER_POINT2D_H_ */
