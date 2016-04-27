/*
 * Point3D.h
 *
 *  Created on: 19.11.2014
 *      Author: Tobias Schellien
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_CONTAINER_POINT3D_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_CONTAINER_POINT3D_H_

#include "geometry_msgs/Point.h"
#include <memory>

using namespace std;


namespace geometry {
	class CNPosition;

	class CNPoint3D : public geometry_msgs::Point {
	public:


		CNPoint3D(double x, double y, double z);
		CNPoint3D() : CNPoint3D(0,0,0) {}
		shared_ptr<CNPoint3D> alloToEgo(CNPosition& me);
		shared_ptr<CNPoint3D> egoToAllo(CNPosition& me);
		shared_ptr<CNPoint3D> normalize();
		double length();
		string toString();
		virtual ~CNPoint3D();

		shared_ptr<CNPoint3D> operator*(const double& right);
		shared_ptr<CNPoint3D> operator/(const double& right);
		shared_ptr<CNPoint3D> operator+(const shared_ptr<CNPoint3D>& right);
		shared_ptr<CNPoint3D> operator-(const shared_ptr<CNPoint3D>& right);
	};

	shared_ptr<CNPoint3D> operator+(const shared_ptr<CNPoint3D>& left, const shared_ptr<CNPoint3D>& right);
	shared_ptr<CNPoint3D> operator-(const shared_ptr<CNPoint3D>& left, const shared_ptr<CNPoint3D>& right);
	shared_ptr<CNPoint3D> operator*(const shared_ptr<CNPoint3D>& left, const double& right);
	shared_ptr<CNPoint3D> operator/(const shared_ptr<CNPoint3D>& left, const double& right);
}
#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_CONTAINER_POINT2D_H_ */
