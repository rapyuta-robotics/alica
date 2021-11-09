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
    uint64_t robotId;

protected:
    double val;
};

} /* namespace alica */