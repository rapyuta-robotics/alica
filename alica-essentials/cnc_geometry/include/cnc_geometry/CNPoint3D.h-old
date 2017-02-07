/*
 * Point3D.h
 *
 *  Created on: 19.11.2014
 *      Author: Tobias Schellien
 */

#pragma once

#include <geometry_msgs/Point.h>

namespace geometry {

class CNPosition;

class CNPoint3D : public geometry_msgs::Point {
public:


	CNPoint3D(double x, double y, double z);
	CNPoint3D() : CNPoint3D(0,0,0) {}
	std::shared_ptr<CNPoint3D> alloToEgo(CNPosition& me);
	std::shared_ptr<CNPoint3D> egoToAllo(CNPosition& me);
	std::shared_ptr<CNPoint3D> normalize();
	double length();
	std::string toString();
	virtual ~CNPoint3D();

	std::shared_ptr<CNPoint3D> operator*(const double& right);
	std::shared_ptr<CNPoint3D> operator/(const double& right);
	std::shared_ptr<CNPoint3D> operator+(const std::shared_ptr<CNPoint3D>& right);
	std::shared_ptr<CNPoint3D> operator-(const std::shared_ptr<CNPoint3D>& right);
};

std::shared_ptr<CNPoint3D> operator+(const std::shared_ptr<CNPoint3D>& left, const std::shared_ptr<CNPoint3D>& right);
std::shared_ptr<CNPoint3D> operator-(const std::shared_ptr<CNPoint3D>& left, const std::shared_ptr<CNPoint3D>& right);
std::shared_ptr<CNPoint3D> operator*(const std::shared_ptr<CNPoint3D>& left, const double& right);
std::shared_ptr<CNPoint3D> operator/(const std::shared_ptr<CNPoint3D>& left, const double& right);

}
