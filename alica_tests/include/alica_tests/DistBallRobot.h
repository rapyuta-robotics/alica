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
class IAlicaWorldModel;

class DistBallRobot : public USummand
{
public:
    DistBallRobot(double weight);
    virtual ~DistBallRobot();
    UtilityInterval eval(IAssignment ass, const Assignment* oldAss, const IAlicaWorldModel& wm) const override;
    essentials::IdentifierConstPtr robotId;

protected:
    double sb;
    double angleBallOpp;
    double velAngle;
    essentials::IDManager* manager;
};

} /* namespace alica */
