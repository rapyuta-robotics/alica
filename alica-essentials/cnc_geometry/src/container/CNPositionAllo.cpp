#include "container/CNPositionAllo.h"

#include <sstream>
#include "container/CNPositionEgo.h"

namespace geometry
{
    using namespace std;

	CNPositionAllo::CNPositionAllo(double x, double y, double theta)
	{
		this->x = x;
		this->y = y;
        this->theta = theta;
	}

	CNPositionAllo::~CNPositionAllo() { }

    string CNPositionAllo::toString()
    {
        stringstream ss;
        ss << "CNPositionAllo: x: " << this->x << " y: " << this->y << " theta: " << this->theta << endl;
        return ss.str();
    }

    double CNPositionAllo::distanceTo(shared_ptr<CNPositionAllo> alloPos)
	{
		return sqrt(pow(this->x - alloPos->x, 2) + pow(this->y - alloPos->y, 2)); //TODO
	}

    shared_ptr<CNPositionEgo> CNPosition::toEgo(CNPositionAllo& me)
    {
        //TODO: rotate ego
        shared_ptr<CNPositionEgo> ego = make_shared<CNPositionEgo>();

        double x = this->x - me.x;
        double y = this->y - me.y;

        double angle = atan2(y, x) - me.theta;
        double dist = sqrt(x * x + y * y);

        ego->x = cos(angle) * dist;
        ego->y = sin(angle) * dist;

        return ego;
    }

    shared_ptr<CNPositionAllo> CNPositionAllo::operator +(const shared_ptr<CNPoint2D>& right)
    {
        auto ret = make_shared<CNPositionAllo>(this->x, this->y, this->theta);
        ret->x += right->x;
        ret->y += right->y;
        return ret;
    }

    shared_ptr<CNPositionAllo> CNPositionAllo::operator -(const shared_ptr<CNPoint2D>& right)
    {
        auto ret = make_shared<CNPositionAllo>(this->x, this->y, this->theta);
        ret->x += right->x;
        ret->y += right->y;
        return ret;
    }

    shared_ptr<CNPositionAllo> operator +(const shared_ptr<CNPositionAllo>& left, const shared_ptr<CNPoint2D>& right)
    {
        auto ret = make_shared<CNPositionAllo>(left->x, left->y, this->theta);
        ret->x += right->x;
        ret->y += right->y;
        return ret;
    }

    shared_ptr<CNPositionAllo> operator -(const shared_ptr<CNPositionAllo>& left, const shared_ptr<CNPoint2D>& right)
    {
        auto ret = make_shared<CNPositionAllo>(left->x, left->y, this->theta);
        ret->x -= right->x;
        ret->y -= right->y;
        return ret;
    }
}
