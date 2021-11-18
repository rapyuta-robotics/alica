#pragma once

#include <engine/USummand.h>
#include <essentials/IdentifierConstPtr.h>
#include <string>
#include <vector>

namespace alica
{

class DummyTestSummand : public USummand
{
public:
    DummyTestSummand(double weight);
    virtual ~DummyTestSummand();
    UtilityInterval eval(IAssignment ass, const Assignment* oldAss, const IAlicaWorldModel& wm) const override;
    essentials::IdentifierConstPtr robotId;

protected:
    double sb;
    double angleBallOpp;
    double velAngle;
};

} /* namespace alica */
