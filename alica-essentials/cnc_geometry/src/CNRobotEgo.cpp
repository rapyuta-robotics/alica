#include "cnc_geometry/CNRobotEgo.h"

#include "cnc_geometry/CNPositionAllo.h"
#include "cnc_geometry/CNRobotAllo.h"

using std::make_shared;
using std::string;
using std::vector;

namespace geometry
{

CNRobotEgo::CNRobotEgo()
    : velocity()
{
    this->radius = 0;
    this->id = 0;
    this->certainty = 0;
    this->rotationVel = 0;
    this->opposer = std::make_shared<std::vector<int>>();
    this->supporter = std::make_shared<std::vector<int>>();
}

CNRobotEgo::~CNRobotEgo()
{
}

string CNRobotEgo::toString() const
{
    std::stringstream ss;
    ss << "CNRobotEgo: ID: " << this->id << " Pos: " << this->position << " Velocity: " << this->velocity << std::endl;
    return ss.str();
}

CNRobotAllo CNRobotEgo::toAllo(CNPositionAllo ownPos)
{
    auto robotAllo = CNRobotAllo();

    robotAllo.radius = this->radius;
    robotAllo.id = this->id;
    robotAllo.certainty = this->certainty;
    robotAllo.rotationVel = this->rotationVel;
    robotAllo.opposer = make_shared<vector<int>>(*this->opposer);
    robotAllo.supporter = make_shared<vector<int>>(*this->supporter);
    robotAllo.position = this->position.toAllo(ownPos);
    robotAllo.velocity = this->velocity.toAllo(ownPos);

    return robotAllo;
}

} /* namespace geometry */
