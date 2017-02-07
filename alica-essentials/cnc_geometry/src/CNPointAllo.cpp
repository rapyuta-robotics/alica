#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPointEgo.h>
#include <cnc_geometry/CNVecAllo.h>
#include <sstream>

#include "cnc_geometry/CNPositionAllo.h"

using std::string;
using std::shared_ptr;

namespace geometry
{

CNPointAllo::CNPointAllo(double x, double y, double z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

CNPointAllo::~CNPointAllo() {}

string CNPointAllo::toString()
{
    std::stringstream ss;
    ss << "CNPointAllo: x: " << x << " y: " << y << " z: " << z << std::endl;
    return ss.str();
}

/**
 * Converts this allocentric 2d point into an egocentric 2d point with respect to
 * the given allocentric position.
 * @param alloPos the allocentric reference position
 * @return an egocentric 2d point with alloPos as origin of ordinates
 */
shared_ptr<CNPointEgo> CNPointAllo::toEgo(CNPositionAllo &alloPos)
{
	// TODO: fix
    shared_ptr<CNPointEgo> ego = std::make_shared<CNPointEgo>();

    // sub me pos from allo pos -> rel pos with allo orientation
    double relX = this->x - alloPos.x;
    double relY = this->y - alloPos.y;

    // rotate rel point around origin -> rel point with ego orientation
    double s = sin(-alloPos.theta);
    double c = cos(-alloPos.theta);

    ego->x = c * relX - s * relY;
    ego->y = s * relX - c * relY;

    return ego;
}

shared_ptr<CNPointAllo> operator+(const shared_ptr<CNPointAllo> &left, const shared_ptr<CNVecAllo> &right)
{
    return std::make_shared<CNPointAllo>(
    		left->x + right->x,
			left->y + right->y,
			left->z + right->z);
}

shared_ptr<CNPointAllo> operator-(const shared_ptr<CNPointAllo> &left, const shared_ptr<CNVecAllo> &right)
{
    return std::make_shared<CNPointAllo>(
    		left->x - right->x,
			left->y - right->y,
			left->z - right->z);
}

} /* namespace geometry */
