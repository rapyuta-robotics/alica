/*
 * CNVelocity2D.cpp
 *
 *  Created on: 19.11.2014
 *      Author: Tobias Schellien
 */

#include "container/CNVelocity2D.h"
#include "container/CNPosition.h"
#include <sstream>

using namespace std;

namespace geometry
{

	CNVelocity2D::CNVelocity2D(double x, double y)
	{
		this->x = x;
		this->y = y;
	}

	double CNVelocity2D::length()
	{
		return sqrt(x * x + y * y);
	}

	shared_ptr<CNVelocity2D> CNVelocity2D::rotate(double radian)
	{
		return make_shared<CNVelocity2D>(this->x * cos(radian) - this->y * sin(radian),
											this->x * sin(radian) + this->y * cos(radian));
	}

	double CNVelocity2D::angleTo()
	{
		return atan2(y, x);
	}

	shared_ptr<CNVelocity2D> CNVelocity2D::alloToEgo(CNPosition& me)
	{
		shared_ptr<CNVelocity2D> ego = make_shared<CNVelocity2D>();

		double angle = atan2(y, x) - me.theta;
		double dist = sqrt(x * x + y * y);

		ego->x = cos(angle) * dist;
		ego->y = sin(angle) * dist;

		return ego;
	}

	shared_ptr<CNVelocity2D> CNVelocity2D::egoToAllo(CNPosition& me)
	{
		shared_ptr<CNVelocity2D> allo = make_shared<CNVelocity2D>();

		allo->x += cos(me.theta) * x - sin(me.theta) * y;
		allo->y += sin(me.theta) * x + cos(me.theta) * y;

		return allo;
	}

	CNVelocity2D::~CNVelocity2D()
	{
	}

	shared_ptr<CNVelocity2D> CNVelocity2D::normalize()
	{
		shared_ptr<CNVelocity2D> norm = make_shared<CNVelocity2D>();
		double length = this->length();

		if (length > 0)
		{
			norm->x = this->x / length;
			norm->y = this->y / length;
		}
		else
		{
			cerr << "CNVelocity2D: Trying to normalize 0.0!" << endl;
		}

		return norm;
	}

	shared_ptr<CNVelocity2D> CNVelocity2D::clone()
	{
		return make_shared<CNVelocity2D>(x, y);
	}

	shared_ptr<CNVelocity2D> CNVelocity2D::operator*(const double& right)
	{
		auto ret = make_shared<CNVelocity2D>(this->x, this->y);
		ret->x *= right;
		ret->y *= right;
		return ret;
	}

	shared_ptr<CNVelocity2D> operator*(const shared_ptr<CNVelocity2D>& left, const double& right)
	{
		auto ret = make_shared<CNVelocity2D>(left->x, left->y);
		ret->x *= right;
		ret->y *= right;
		return ret;
	}

	shared_ptr<CNVelocity2D> CNVelocity2D::operator+(const shared_ptr<CNVelocity2D>& right)
	{
		auto ret = make_shared<CNVelocity2D>(this->x, this->y);
		ret->x += right->x;
		ret->y += right->y;
		return ret;
	}

	shared_ptr<CNPoint2D> CNVelocity2D::operator+(const shared_ptr<CNPoint2D>& right)
	{
		auto ret = make_shared<CNPoint2D>(this->x, this->y);
		ret->x += right->x;
		ret->y += right->y;
		return ret;
	}

	shared_ptr<CNVelocity2D> operator+(const shared_ptr<CNVelocity2D>& left, const shared_ptr<CNVelocity2D>& right)
	{
		auto ret = make_shared<CNVelocity2D>(left->x, left->y);
		ret->x += right->x;
		ret->y += right->y;
		return ret;
	}

	shared_ptr<CNPoint2D> operator+(const shared_ptr<CNVelocity2D>& left, const shared_ptr<CNPoint2D>& right)
		{
			auto ret = make_shared<CNPoint2D>(left->x, left->y);
			ret->x += right->x;
			ret->y += right->y;
			return ret;
		}

	string CNVelocity2D::toString()
	{
		stringstream ss;
		ss << "CNVelocity2D: x: " << this->x << " y: " << this->y << endl;
		return ss.str();
	}
}

