#include "cnc_geometry/CNPositionAllo.h"
#include <cnc_geometry/CNVecAllo.h>

#include <sstream>

#include "cnc_geometry/CNPositionEgo.h"
#include "cnc_geometry/Calculator.h"

using std::shared_ptr;
using std::string;

namespace geometry
{

CNPositionAllo::CNPositionAllo(double x, double y, double theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
}

CNPositionAllo::CNPositionAllo(const CNPositionAllo& obj)
{
    this->x = obj.x;
    this->y = obj.y;
    this->theta = obj.theta;
}

CNPositionAllo::~CNPositionAllo() {}

string CNPositionAllo::toString() const
{
    std::stringstream ss;
    ss << "CNPositionAllo: X: " << this->x << " Y: " << this->y << " Orientation: " << this->theta << std::endl;
    return ss.str();
}

double CNPositionAllo::distanceTo(CNPointAllo& pos) const
{
    CNVecAllo delta = *this - pos;
    return delta.length();
}

CNPositionEgo CNPositionAllo::toEgo(CNPositionAllo& me) const
{
    auto ego = CNPositionEgo();

    double x = this->x - me.x;
    double y = this->y - me.y;

    double angle = atan2(y, x) - me.theta;
    double dist = sqrt(x * x + y * y);

    ego.x = cos(angle) * dist;
    ego.y = sin(angle) * dist;
    ego.theta = normalizeAngle(this->theta - me.theta);

    return ego;
    //
    //    // sub me pos from allo pos -> rel pos with allo orientation
    //    double relX = this->x - me.x;
    //    double relY = this->y - me.y;
    //
    //    // rotate rel point around origin -> rel point with ego orientation
    //    double s = sin(-me.theta);
    //    double c = cos(-me.theta);
    //
    //    ego.x = c * relX - s * relY;
    //    ego.y = s * relX + c * relY; // TODO: fix experimentals
    //
    //    // rotate theta
    //    ego.theta = this->theta - me.theta;
    //
    //    return ego;
}

CNPointAllo CNPositionAllo::getPoint() const
{
    return CNPointAllo(this->x, this->y);
}

CNPositionAllo CNPositionAllo::operator+(const CNVecAllo& right) const
{
    return CNPositionAllo(this->x + right.x, this->y + right.y, this->theta);
}

CNPositionAllo CNPositionAllo::operator-(const CNVecAllo& right) const
{
    return CNPositionAllo(this->x - right.x, this->y - right.y, this->theta);
}

CNVecAllo CNPositionAllo::operator-(const CNPositionAllo& right) const
{
    return CNVecAllo(this->x - right.x, this->y - right.y);
}

CNVecAllo CNPositionAllo::operator-(const CNPointAllo& right) const
{
    return CNVecAllo(this->x - right.x, this->y - right.y);
}

} /* namespace geometry */
