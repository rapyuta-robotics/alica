#pragma once

#include "container/CNPosition.h"

namespace supplementary
{
class IAgentID;
}

namespace geometry
{

class CNRobot : public CNPosition
{
  public:
    CNRobot();
    virtual ~CNRobot();
    double radius;
    double velocityX;
    double velocityY;
    const supplementary::IAgentID* id;
    shared_ptr<vector<int>> opposer;
    shared_ptr<vector<int>> supporter;
    double certainty;
    double rotation;
    string toString();
};
} // namespace geometry
