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

CNPointEgo::CNPointEgo(const CNPointEgo &obj)
{
	this->x = obj.x;
	this->y = obj.y;
	this->z = obj.z;
}

CNPointEgo::~CNPointEgo() {}

string CNPointEgo::toString() const
{
    std::stringstream ss;
    ss << "CNPointEgo: X: " << x << " Y: " << y << " z: " << z << std::endl;
    return ss.str();
}

CNPointAllo CNPointEgo::toAllo(const CNPositionAllo &me) const
{
    auto allo = CNPointAllo();

    // rotate rel point around origin -> rel point with allo orientation
    double s = sin(me.theta);
    double c = cos(me.theta);

    double x = c * this->x - s * this->y;
    double y = s * this->x - c * this->y; // TODO: fix

    // sum me pos and rel pos -> allo pos with allo rotaion
    allo.x = x + me.x;
    allo.y = y + me.y;
    allo.z = this->z;

    return allo;
}

double CNPointEgo::distanceTo(const CNPointEgo &other) const
{
	return (*this - other).length();
}

CNPointEgo CNPointEgo::operator+(const CNPointEgo &right) const
{
    return CNPointEgo(
    		this->x + right.x,
			this->y + right.y,
			this->z + right.z);
}

CNPointEgo CNPointEgo::operator+(const CNVecEgo &right) const
{
    return CNPointEgo(
    		this->x + right.x,
			this->y + right.y,
			this->z + right.z);
}

CNPointEgo CNPointEgo::operator-(const CNVecEgo &right) const
{
    return CNPointEgo(
    		this->x - right.x,
			this->y - right.y,
			this->z - right.z);
}

CNVecEgo CNPointEgo::operator-(const CNPointEgo &right) const
{
    return CNVecEgo(
    		this->x - right.x,
			this->y - right.y,
			this->z - right.z);
}

} /* namespace geometry */

