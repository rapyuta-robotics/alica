#include <alica_tests/SwitchEntryPointsSummand.h>
#include "engine/planselector/IAssignment.h"

namespace alica
{

SwitchEntryPointsSummand::SwitchEntryPointsSummand(double weight)
        : USummand(weight)
{
}

SwitchEntryPointsSummand::~SwitchEntryPointsSummand() {}

UtilityInterval SwitchEntryPointsSummand::eval(IAssignment ass, const Assignment* oldAss) const
{
    if (!oldAss) {
        UtilityInterval ui(0.0, 1.0);
        return ui;
    }

    const auto& firstAgentStatePairs = oldAss->getAgentStates(_relevantEntryPoints[0]);
    const auto& secondAgentStatePairs = oldAss->getAgentStates(_relevantEntryPoints[1]);

    bool found = false;
    for (const auto& agentStatePair : firstAgentStatePairs) {
        for (essentials::IdentifierConstPtr agentId : ass.getAgentsWorking(_relevantEntryPoints[1])) {
            if (agentId == agentStatePair.first) {
                found = true;
            }
        }
        if (!found) {
            return UtilityInterval(-1.0, -1.0);
        }
        found = false;
    }
    for (const auto& agentStatePair : secondAgentStatePairs) {
        for (essentials::IdentifierConstPtr agentId : ass.getAgentsWorking(_relevantEntryPoints[0])) {
            if (agentId == agentStatePair.first) {
                found = true;
            }
        }
        if (!found) {
            return UtilityInterval(-1.0, -1.0);
        }
        found = false;
    }

    UtilityInterval ui(0.0, 1.0);
    return ui;
}

std::string SwitchEntryPointsSummand::toString(IAssignment ass) const
{
    std::stringstream ss;
    ss << "Total Agent Count: " << ass.getTotalAgentCount() << std::endl;
    ss << "Assigned Agent Count: " << ass.getAssignedAgentCount() << std::endl;
    ss << "Unassigned Agent Count: " << ass.getUnAssignedAgentCount() << std::endl;
    ss << "EP Count: " << ass.getEntryPointCount() << std::endl;
    return ss.str();
}

} /* namespace alica */
