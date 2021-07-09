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
//    for (const alica::EntryPoint* ep : _relevantEntryPoints) {
//        bool found = false;
//        for (alica::AgentStatePair agentStatePair : agentStatePair : ass->getAgentStates(ep)) {
//            for (essentials::IdentifierCosntPtr agentId : ass.getAgentsWorking(1625614710816)) {
//                if (agentId != agentStatePair.first) {
//                    found = true;
//                }
//            }
//            if (!found) {
//                return alica::UtilityInterval(-1.0, -1.0);
//            }
//            found = false;
//        }
//    }
    return alica::UtilityInterval(0.0, 1.0);
}
} // namespace alica