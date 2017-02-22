#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPointEgo.h>
#include <cnc_geometry/CNVecAllo.h>
#include <sstream>

#include "cnc_geometry/CNPositionAllo.h"

namespace geometry
{

using std::string;
using std::shared_ptr;

CNPointAllo::CNPointAllo(double x, double y, double z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

CNPointAllo::CNPointAllo(const CNPointAllo &obj)
{
    this->x = obj.x;
    this->y = obj.y;
    this->z = obj.z;
}

CNPointAllo::~CNPointAllo()
{
}

string CNPointAllo::toString() const
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
CNPointEgo CNPointAllo::toEgo(CNPositionAllo &alloPos) const
{
    CNPointEgo ego = CNPointEgo();

    // sub me pos from allo pos -> rel pos with allo orientation
    double relX = this->x - alloPos.x;
    double relY = this->y - alloPos.y;

    // rotate rel point around origin -> rel point with ego orientation
    double s = sin(-alloPos.theta);
    double c = cos(-alloPos.theta);

    ego.x = c * relX - s * relY;
    ego.y = s * relX - c * relY; // TODO: fix
    ego.z = this->z;

    return ego;
}

double CNPointAllo::distanceTo(const CNPointAllo &other) const
{
    return (*this - other).length();
}

CNPointAllo CNPointAllo::operator+(const CNVecAllo &right) const
{
    return CNPointAllo(this->x + right.x, this->y + right.y, this->z + right.z);
}

CNPointAllo CNPointAllo::operator-(const CNVecAllo &right) const
{
    return CNPointAllo(this->x - right.x, this->y - right.y, this->z - right.z);
}

CNVecAllo CNPointAllo::operator-(const CNPointAllo &right) const
{
    return CNVecAllo(this->x - right.x, this->y - right.y, this->z - right.z);
}

} /* namespace geometry */
