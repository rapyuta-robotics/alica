#pragma once

#include <engine/USummand.h>

#include <memory>
#include <string>
#include <vector>

namespace alica
{

class IAssignment;

class DistBallRobot : public USummand
{
public:
    DistBallRobot(double weight);
    virtual ~DistBallRobot();
    UtilityInterval eval(IAssignment ass, const Assignment* oldAss) const override;
    AgentId robotId;

protected:
    double sb;
    double angleBallOpp;
    double velAngle;
};

} /* namespace alica */
