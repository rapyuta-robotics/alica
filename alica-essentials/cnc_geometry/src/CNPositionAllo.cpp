#include <cnc_geometry/CNVecAllo.h>
#include "cnc_geometry/CNPositionAllo.h"

#include <sstream>

#include "cnc_geometry/CNPositionEgo.h"

using std::string;
using std::shared_ptr;

namespace geometry
{

CNPositionAllo::CNPositionAllo(double x, double y, double theta)
{
	this->x = x;
	this->y = y;
	this->theta = theta;
}

CNPositionAllo::CNPositionAllo(const CNPositionAllo &obj)
{
	this->x = obj.x;
	this->y = obj.y;
	this->theta = obj.theta;
}

CNPositionAllo::~CNPositionAllo() {}

string CNPositionAllo::toString()
{
    std::stringstream ss;
    ss << "CNPositionAllo: X: " << this->x << " Y: " << this->y << " Orientation: " << this->theta << std::endl;
    return ss.str();
}

CNPositionEgo CNPositionAllo::toEgo(CNPositionAllo &me)
{
    auto ego = CNPositionEgo();

    // sub me pos from allo pos -> rel pos with allo orientation
    double relX = this->x - me.x;
    double relY = this->y - me.y;

    // rotate rel point around origin -> rel point with ego orientation
    double s = sin(-me.theta);
    double c = cos(-me.theta);

    ego.x = c * relX - s * relY;
    ego.y = s * relX - c * relY; // TODO: fix

    // rotate theta
    ego.theta = this->theta - me.theta;

    return ego;
}

CNPointAllo CNPositionAllo::getPoint()
{
	return CNPointAllo(this->x, this->y, 0);
}

CNPositionAllo CNPositionAllo::operator+(const CNVecAllo &right)
{
	return CNPositionAllo(
			this->x + right.x,
			this->y + right.y,
			this->theta);
}

CNPositionAllo CNPositionAllo::operator-(const CNVecAllo &right)
{
	return CNPositionAllo(
			this->x - right.x,
			this->y - right.y,
			this->theta);
}

CNVecAllo CNPositionAllo::operator-(const CNPositionAllo &right)
{
	return CNVecAllo(
			this->x - right.x,
			this->y - right.y,
			this->theta);
}

} /* namespace geometry */
