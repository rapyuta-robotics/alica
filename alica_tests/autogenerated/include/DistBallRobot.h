#pragma once

#include <engine/USummand.h>
#include <essentials/IDManager.h>
#include <essentials/IdentifierConstPtr.h>

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
    UtilityInterval eval(IAssignment ass) const override;
    essentials::IdentifierConstPtr robotId;

protected:
    double sb;
    double angleBallOpp;
    double velAngle;
    essentials::IDManager* manager;
};

} /* namespace alica */
