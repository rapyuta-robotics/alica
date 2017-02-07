/*
 * Point3D.cpp
 *
 *  Created on: 19.11.2014
 *      Author: Tobias Schellien
 */

#include "CNPoint3D.h"
#include "CNPosition.h"

using namespace std;

namespace geometry
{

	CNPoint3D::CNPoint3D(double x, double y, double z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}

	double CNPoint3D::length()
	{
		return sqrt(x * x + y * y + z * z);
	}

	CNPoint3D::~CNPoint3D()
	{
	}

	shared_ptr<CNPoint3D> CNPoint3D::normalize()
	{
		shared_ptr<CNPoint3D> norm = make_shared<CNPoint3D>();
		double length = this->length();

		if (length > 0)
		{
			norm->x = this->x / length;
			norm->y = this->y / length;
			norm->z = this->z / length;
		}
		else
		{
			cerr << "CNPoint3D: Trying to normalize 0.0!" << endl;
		}

		return norm;
	}

	string CNPoint3D::toString()
	{
		stringstream ss;
		ss << "CNPoint3D: x: " << this->x << " y: " << this->y << " z: " << this->z << endl;
		return ss.str();
	}

	shared_ptr<CNPoint3D> CNPoint3D::alloToEgo(CNPosition& me)
	{
		shared_ptr<CNPoint3D> ego = make_shared<CNPoint3D>();

		double x = this->x - me.x;
		double y = this->y - me.y;

		double angle = atan2(y, x) - me.theta;
		double dist = sqrt(x * x + y * y);

		ego->x = cos(angle) * dist;
		ego->y = sin(angle) * dist;
		ego->z = this->z;

		return ego;
	}

	shared_ptr<CNPoint3D> CNPoint3D::egoToAllo(CNPosition& me)
	{
		shared_ptr<CNPoint3D> allo = make_shared<CNPoint3D>();
		allo->x = me.x;
		allo->y = me.y;
		allo->z = this->z;

		allo->x += cos(me.theta) * x - sin(me.theta) * y;
		allo->y += sin(me.theta) * x + cos(me.theta) * y;

		return allo;
	}

	shared_ptr<CNPoint3D> CNPoint3D::operator *(const double& right)
	{
		auto ret = make_shared<CNPoint3D>(this->x, this->y, this->z);
		ret->x *= right;
		ret->y *= right;
		ret->z *= right;
		return ret;
	}

	shared_ptr<CNPoint3D> CNPoint3D::operator /(const double& right)
	{
		auto ret = make_shared<CNPoint3D>(this->x, this->y, this->z);
		ret->x /= right;
		ret->y /= right;
		ret->z /= right;
		return ret;
	}

	shared_ptr<CNPoint3D> CNPoint3D::operator +(const shared_ptr<CNPoint3D>& right)
	{
		auto ret = make_shared<CNPoint3D>(this->x, this->y, this->z);
		ret->x += right->x;
		ret->y += right->y;
		ret->z += right->z;
		return ret;
	}

	shared_ptr<CNPoint3D> CNPoint3D::operator -(const shared_ptr<CNPoint3D>& right)
	{
		auto ret = make_shared<CNPoint3D>(this->x, this->y, this->z);
		ret->x -= right->x;
		ret->y -= right->y;
		ret->z -= right->z;
		return ret;
	}

	shared_ptr<CNPoint3D> operator +(const shared_ptr<CNPoint3D>& left, const shared_ptr<CNPoint3D>& right)
	{
		auto ret = make_shared<CNPoint3D>(left->x, left->y, left->z);
		ret->x += right->x;
		ret->y += right->y;
		ret->z += right->z;
		return ret;
	}

	shared_ptr<CNPoint3D> operator -(const shared_ptr<CNPoint3D>& left, const shared_ptr<CNPoint3D>& right)
	{
		auto ret = make_shared<CNPoint3D>(left->x, left->y, left->z);
		ret->x -= right->x;
		ret->y -= right->y;
		ret->z -= right->z;
		return ret;
	}

	shared_ptr<CNPoint3D> operator *(const shared_ptr<CNPoint3D>& left, const double& right)
	{
		auto ret = make_shared<CNPoint3D>(left->x, left->y, left->z);
		ret->x *= right;
		ret->y *= right;
		ret->z *= right;
		return ret;
	}

	shared_ptr<CNPoint3D> operator /(const shared_ptr<CNPoint3D>& left, const double& right)
	{
		auto ret = make_shared<CNPoint3D>(left->x, left->y, left->z);
		ret->x /= right;
		ret->y /= right;
		ret->z /= right;
		return ret;
	}

}

