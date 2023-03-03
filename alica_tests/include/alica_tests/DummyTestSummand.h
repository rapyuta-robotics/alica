#pragma once

#include <engine/USummand.h>
#include <string>
#include <vector>

namespace alica
{

class DummyTestSummand : public USummand
{
public:
    DummyTestSummand(double weight);
    virtual ~DummyTestSummand();
    UtilityInterval eval(IAssignment ass, const Assignment* oldAss, const Blackboard* globalBlackboard) const override;
    AgentId robotId;

protected:
    double sb;
    double angleBallOpp;
    double velAngle;
};

} /* namespace alica */
