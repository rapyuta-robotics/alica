#pragma once

#include <engine/USummand.h>
#include <essentials/IdentifierConstPtr.h>
#include <string>
#include <vector>

namespace alica
{

class DynamicAllocationRuleTestSummand : public USummand
{
public:
    DynamicAllocationRuleTestSummand(double weight);
    virtual ~DynamicAllocationRuleTestSummand();
    UtilityInterval eval(IAssignment ass) const override;
};

} /* namespace alica */
