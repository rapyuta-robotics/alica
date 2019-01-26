#pragma once

#include <engine/USummand.h>

#include <memory>
#include <string>
#include <vector>

#include "engine/AgentIDConstPtr.h"
#include "essentials/AgentIDManager.h"

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
    essentials::AgentIDManager* manager;
};

} /* namespace alica */
