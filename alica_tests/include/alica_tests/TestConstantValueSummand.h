#pragma once

#include <engine/USummand.h>

namespace alica
{

class TestConstantValueSummand : public USummand
{
public:
    TestConstantValueSummand(double weight, double val);
    virtual ~TestConstantValueSummand();

    UtilityInterval eval(IAssignment ass, const Assignment* oldAss) const override;
    AgentId robotId;

protected:
    double val;
};

} /* namespace alica */