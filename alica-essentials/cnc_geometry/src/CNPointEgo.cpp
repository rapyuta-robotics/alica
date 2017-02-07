#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPointEgo.h>
#include <cnc_geometry/CNVecEgo.h>
#include <sstream>

#include "cnc_geometry/CNPositionAllo.h"

using std::string;
using std::shared_ptr;

namespace geometry
{

CNPointEgo::CNPointEgo(double x, double y, double z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

CNPointEgo::~CNPointEgo() {}

string CNPointEgo::toString()
{
    std::stringstream ss;
    ss << "CNPointEgo: X: " << x << " Y: " << y << " z: " << z << std::endl;
    return ss.str();
}

shared_ptr<CNPointAllo> CNPointEgo::toAllo(CNPositionAllo &me)
{
	// TODO: fix
    shared_ptr<CNPointAllo> allo = std::make_shared<CNPointAllo>();

    // rotate rel point around origin -> rel point with allo orientation
    double s = sin(me.theta);
    double c = cos(me.theta);

    double x = c * this->x - s * this->y;
    double y = s * this->x - c * this->y;

    // sum me pos and rel pos -> allo pos with allo rotaion
    allo->x = x + me.x;
    allo->y = y + me.y;

    return allo;
}

shared_ptr<CNPointEgo> operator+(const shared_ptr<CNPointEgo> &left, const shared_ptr<CNVecEgo> &right)
{
    return std::make_shared<CNPointEgo>(
    		left->x + right->x,
			left->y + right->y,
			left->z + right->z);
}

shared_ptr<CNPointEgo> operator-(const shared_ptr<CNPointEgo> &left, const shared_ptr<CNVecEgo> &right)
{
    return std::make_shared<CNPointEgo>(
    		left->x - right->x,
			left->y - right->y,
			left->z - right->z);
}

} /* namespace geometry */
