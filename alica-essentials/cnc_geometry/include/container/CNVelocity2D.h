/*
 * CNVelocity2D.h
 *
 *  Created on: 19.11.2014
 *      Author: Tobias Schellien
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_CONTAINER_CNVELOCITY2D_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_CONTAINER_CNVELOCITY2D_H_

#include "CNPoint2D.h"

namespace geometry {

	class CNVelocity2D : public geometry_msgs::Point {
		public:


		CNVelocity2D(double x, double y);
		CNVelocity2D() : CNVelocity2D(0,0) {}

			double length();
			shared_ptr<CNVelocity2D> rotate(double radian);
			double angleTo();
			shared_ptr<CNVelocity2D> alloToEgo(CNPosition& me);
			shared_ptr<CNVelocity2D> egoToAllo(CNPosition& me);
			shared_ptr<CNVelocity2D> normalize();

			shared_ptr<CNVelocity2D> operator*(const double& right);
			shared_ptr<CNVelocity2D> operator+(const shared_ptr<CNVelocity2D>& right);
			shared_ptr<CNPoint2D> operator+(const shared_ptr<CNPoint2D>& right);

			virtual ~CNVelocity2D();
			string toString();
		};

		shared_ptr<CNVelocity2D> operator+(const shared_ptr<CNVelocity2D>& left, const shared_ptr<CNVelocity2D>& right);
		shared_ptr<CNVelocity2D> operator*(const shared_ptr<CNVelocity2D>& left, const double& right);
		shared_ptr<CNPoint2D> operator+(const shared_ptr<CNVelocity2D>& left, const shared_ptr<CNPoint2D>& right);

}
#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_CONTAINER_CNVELOCITY2D_H_ */
