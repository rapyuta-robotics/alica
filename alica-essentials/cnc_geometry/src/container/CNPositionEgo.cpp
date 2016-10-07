#include "container/CNPositionEgo.h"

#include "container/CNPositionAllo.h"
#include <sstream>

namespace geometry
{
    using namespace std;

	CNPositionEgo::CNPositionEgo(double x, double y, double theta)
	{
		this->x = x;
		this->y = y;
        this->theta = theta;
	}

	CNPositionEgo::~CNPositionEgo() { }

	string CNPositionEgo::toString()
	{
		stringstream ss;
		ss << "CNPositionAllo: x: " << this->x << " y: " << this->y << " theta: " << this->theta << endl;
		return ss.str();
	}

	double CNPositionEgo::distanceTo(shared_ptr<CNPositionEgo> egoPos)
	{
		return sqrt(pow(this->x - egoPos->x, 2) + pow(this->y - egoPos->y, 2)); //TODO
	}

    shared_ptr<CNPositionAllo> CNPositionEgo::toAllo(CNPositionAllo& me)
    {
        // TODO: rotate ego
        shared_ptr<CNPositionAllo> allo = make_shared<CNPositionAllo>();
        allo->x = me.x;
        allo->y = me.y;

        allo->x += cos(me.theta) * x - sin(me.theta) * y;
        allo->y += sin(me.theta) * x + cos(me.theta) * y;

        return allo;
    }

	shared_ptr<CNPositionEgo> CNPositionEgo::operator +(const shared_ptr<CNPositionEgo>& right)
	{
		auto ret = make_shared<CNPositionEgo>(this->x, this->y);
		ret->x += right->x;
		ret->y += right->y;
		return ret;
	}

	shared_ptr<CNPositionEgo> CNPositionEgo::operator -(const shared_ptr<CNPositionEgo>& right)
	{
		auto ret = make_shared<CNPositionEgo>(this->x, this->y);
		ret->x += right->x;
		ret->y += right->y;
		return ret;
	}

	shared_ptr<CNPositionEgo> CNPositionEgo::getEgoPoint()
	{
		return make_shared<CNPositionEgo>(this->x, this->y);
	}

	shared_ptr<CNPositionEgo> operator +(const shared_ptr<CNPositionEgo>& left, const shared_ptr<CNPositionEgo>& right)
	{
		auto ret = make_shared<CNPositionEgo>(left->x, left->y);
		ret->x += right->x;
		ret->y += right->y;
		return ret;
	}

	shared_ptr<CNPositionEgo> operator -(const shared_ptr<CNPositionEgo>& left, const shared_ptr<CNPositionEgo>& right)
	{
		auto ret = make_shared<CNPositionEgo>(left->x, left->y);
		ret->x -= right->x;
		ret->y -= right->y;
		return ret;
	}
}
