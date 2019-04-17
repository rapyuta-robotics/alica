#pragma once

#include <essentials/AgentIDConstPtr.h>
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
    UtilityInterval eval(IAssignment ass) const override;
    essentials::AgentIDConstPtr robotId;

protected:
    double sb;
    double angleBallOpp;
    double velAngle;
};

} /* namespace alica */
