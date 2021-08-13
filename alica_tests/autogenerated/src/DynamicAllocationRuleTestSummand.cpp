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
    if (!alicaTests::TestWorldModel::getOne()->isSwitchingEntryPoints()) {
        return alica::UtilityInterval(0.0, 1.0);
    }

    if (ass.getAgentsWorking(_relevantEntryPoints[1]).size() > 0) {
        return alica::UtilityInterval(0.0, 1.0);
    }

    return UtilityInterval(-1.0, -1.0);
}
} // namespace alica