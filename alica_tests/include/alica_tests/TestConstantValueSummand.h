#pragma once

#include <engine/USummand.h>
#include <essentials/IdentifierConstPtr.h>

namespace alica
{

class TestConstantValueSummand : public USummand
{
public:
    TestConstantValueSummand(double weight, double val);
    virtual ~TestConstantValueSummand();

    UtilityInterval eval(IAssignment ass, const Assignment* oldAss, const IAlicaWorldModel& wm) const override;
    essentials::IdentifierConstPtr robotId;

protected:
    double val;
};

} /* namespace alica */