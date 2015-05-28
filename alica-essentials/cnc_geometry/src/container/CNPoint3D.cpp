/*
 * Point3D.cpp
 *
 *  Created on: 19.11.2014
 *      Author: Tobias Schellien
 */

#include "container/CNPoint3D.h"
#include "container/CNPosition.h"

using namespace std;

namespace geometry {

	CNPoint3D::CNPoint3D(double x, double y, double z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

	double CNPoint3D::length() {
		return sqrt(x * x + y * y + z * z);
	}

	CNPoint3D::~CNPoint3D() {
	}

	string CNPoint3D::toString()
	{
		stringstream ss;
		ss << "CNPoint3D: x: " << this->x << " y: " << this->y << " z: " << this->z << endl;
		return ss.str();
	}
}

