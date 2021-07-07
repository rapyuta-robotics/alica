#include "DynamicAllocationRuleTestSummand.h"

#include "engine/model/EntryPoint.h"
#include "engine/planselector/IAssignment.h"
#include <TestWorldModel.h>

namespace alica
{
DynamicAllocationRuleTestSummand::DynamicAllocationRuleTestSummand(double weight)
        : USummand(weight)
{
}

DynamicAllocationRuleTestSummand::~DynamicAllocationRuleTestSummand() {}

alica::UtilityInterval DynamicAllocationRuleTestSummand::eval(alica::IAssignment ass) const
{
    return alica::UtilityInterval(0.0, 1.0);
}
} // namespace alica