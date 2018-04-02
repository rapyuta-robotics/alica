#include "cnc_geometry/CNRobotAllo.h"

#include "cnc_geometry/CNPositionAllo.h"
#include "cnc_geometry/CNRobotEgo.h"

using std::endl;
using std::make_shared;
using std::string;
using std::vector;

namespace geometry {

CNRobotAllo::CNRobotAllo() : velocity() {
    this->radius = 0;
    this->id = 0;
    this->certainty = 0;
    this->rotationVel = 0;
    this->opposer = make_shared<vector<int>>();
    this->supporter = make_shared<vector<int>>();
}

CNRobotAllo::~CNRobotAllo() {}

string CNRobotAllo::toString() const {
    std::stringstream ss;
    ss << "CNRobot: ID: " << this->id << " Pos: " << this->position << " Velocity: " << this->velocity << endl;
    return ss.str();
}

CNRobotEgo CNRobotAllo::toEgo(CNPositionAllo ownPos) {
    auto robotEgo = CNRobotEgo();

    robotEgo.radius = this->radius;
    robotEgo.id = this->id;
    robotEgo.certainty = this->certainty;
    robotEgo.rotationVel = this->rotationVel;
    robotEgo.opposer = make_shared<vector<int>>(*this->opposer);
    robotEgo.supporter = make_shared<vector<int>>(*this->supporter);
    robotEgo.position = this->position.toEgo(ownPos);
    robotEgo.velocity = this->velocity.toEgo(ownPos);

    return robotEgo;
}

} /* namespace geometry */
