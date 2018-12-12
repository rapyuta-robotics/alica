/*
 * CNPosition.cpp
 *
 *  Created on: 19.11.2014
 *      Author: tobi
 */

#include "container/CNPosition.h"
#include <sstream>
namespace geometry
{

	CNPosition::CNPosition(double x, double y, double theta)
	{
		this->x = x;
		this->y = y;
		this->theta = theta;
	}

	CNPosition::~CNPosition()
	{
	}

	string CNPosition::toString()
	{
		stringstream ss;
		ss << "CNPosition: x: " << this->x << " y: " << this->y << " theta: " << this->theta << endl;
		return ss.str();
	}

	double CNPosition::distanceTo(shared_ptr<CNPoint2D> point)
	{
		return sqrt(pow(this->x - point->x, 2) + pow(this->y - point->y, 2));
	}

	shared_ptr<CNPoint2D> CNPosition::alloToEgo(CNPosition& me)
	{
		shared_ptr<CNPoint2D> ego = make_shared<CNPoint2D>();

		double x = this->x - me.x;
		double y = this->y - me.y;

		double angle = atan2(y, x) - me.theta;
		double dist = sqrt(x * x + y * y);

		ego->x = cos(angle) * dist;
		ego->y = sin(angle) * dist;

		return ego;
	}

	shared_ptr<CNPoint2D> CNPosition::egoToAllo(CNPosition& me)
	{
		shared_ptr<CNPoint2D> allo = make_shared<CNPoint2D>();
		allo->x = me.x;
		allo->y = me.y;

		allo->x += cos(me.theta) * x - sin(me.theta) * y;
		allo->y += sin(me.theta) * x + cos(me.theta) * y;

		return allo;
	}

	shared_ptr<CNPoint2D> CNPosition::operator +(const shared_ptr<CNPoint2D>& right)
	{
		auto ret = make_shared<CNPoint2D>(this->x, this->y);
		ret->x += right->x;
		ret->y += right->y;
		return ret;
	}

	shared_ptr<CNPoint2D> CNPosition::operator -(const shared_ptr<CNPoint2D>& right)
	{
		auto ret = make_shared<CNPoint2D>(this->x, this->y);
		ret->x += right->x;
		ret->y += right->y;
		return ret;
	}

	shared_ptr<CNPoint2D> CNPosition::operator +(const shared_ptr<CNPosition>& right)
	{
		auto ret = make_shared<CNPoint2D>(this->x, this->y);
		ret->x += right->x;
		ret->y += right->y;
		return ret;
	}

	shared_ptr<CNPoint2D> CNPosition::operator -(const shared_ptr<CNPosition>& right)
	{
		auto ret = make_shared<CNPoint2D>(this->x, this->y);
		ret->x += right->x;
		ret->y += right->y;
		return ret;
	}

	shared_ptr<CNPoint2D> CNPosition::getPoint()
	{
		return make_shared<CNPoint2D>(this->x, this->y);
	}

	shared_ptr<CNPoint2D> operator +(const shared_ptr<CNPosition>& left, const shared_ptr<CNPosition>& right)
	{
		auto ret = make_shared<CNPoint2D>(left->x, left->y);
		ret->x += right->x;
		ret->y += right->y;
		return ret;
	}

	shared_ptr<CNPoint2D> operator -(const shared_ptr<CNPosition>& left, const shared_ptr<CNPosition>& right)
	{
		auto ret = make_shared<CNPoint2D>(left->x, left->y);
		ret->x -= right->x;
		ret->y -= right->y;
		return ret;
	}
}

