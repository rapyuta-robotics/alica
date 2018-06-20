#pragma once

#include <engine/USummand.h>

#include <memory>
#include <string>
#include <vector>

namespace supplementary
{
class AgentID;
class AgentIDManager;
} // namespace supplementary

namespace alica
{

class IAssignment;

class DistBallRobot : public USummand
{
public:
    DistBallRobot(double weight);
    virtual ~DistBallRobot();
    UtilityInterval eval(IAssignment ass) const override;
    AgentIDConstPtr robotId;

protected:
    double sb;
    double angleBallOpp;
    double velAngle;
    supplementary::AgentIDManager* manager;
};

} /* namespace alica */
