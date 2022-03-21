#include "engine/model/EntryPoint.h"
#include "engine/planselector/IAssignment.h"
#include <alica_tests/AssignPayloadSummand.h>
#include <alica_tests/TaskInstantiationIntegrationWorldModel.h>

#include <math.h>

namespace alica
{

AssignPayloadSummand::AssignPayloadSummand(double weight)
        : USummand(weight)
{
    this->robotId = 8;
}

AssignPayloadSummand::~AssignPayloadSummand() {}

UtilityInterval AssignPayloadSummand::eval(IAssignment ass, const Assignment* oldAss, const IAlicaWorldModel* wm) const
{
    auto worldModel = dynamic_cast<const alicaTests::TaskInstantiationIntegrationWorldModel*>(wm);
    std::vector<uint64_t> nonVisitedEps;
    for (uint64_t dynId = 1; dynId < 9; dynId++) {
        int64_t payloadId = dynId - 1;
        auto agentsInEntryPoint = ass.getAgentsWorking(movePayloadEpId, dynId);

        // dont accept assignments with multiple agents in one entry point
        if (agentsInEntryPoint.size() > 1) {
            return UtilityInterval(-1, -1);
        }

        if (!oldAss) {
            continue;
        }

        // // agents already working on payloads should not switch entry points
        // auto agentsInEntryPointOldAss = oldAss->getAgentsWorking(movePayloadEpId, dynId);
        // if (agentsInEntryPointOldAss.size() == 1 && *(agentsInEntryPoint.begin()) != *(agentsInEntryPointOldAss.begin()) && worldModel->payloads.at(dynId - 1).state != alicaTests::PayloadState::DROPPED) {
        //     return UtilityInterval(-1, -1);
        // }

        // // agents should not be assigned to payloads that have already been moved
        // if (agentsInEntryPoint.size() > 0 && worldModel->payloads.at(dynId - 1).state == alicaTests::PayloadState::DROPPED) {
        //     return UtilityInterval(-1, -1);
        // }

        // // collect payloads that are currently unassigned
        // if (agentsInEntryPointOldAss.size() == 0 && worldModel->payloads.at(dynId - 1).state == alicaTests::PayloadState::READY_FOR_PICKUP) {
        //     nonVisitedEps.push_back(dynId);
        // }

    }

    for (auto it = nonVisitedEps.begin(); it != nonVisitedEps.end(); it++) {
        for (AgentId agentId : ass.getAgentsWorking(movePayloadEpId, *(it))) {
            if (agentId == worldModel->agentId) {
                double x = worldModel->agentLocations.at(agentId).first - worldModel->payloads[*it - 1].pickX;
                double y = worldModel->agentLocations.at(agentId).second - worldModel->payloads[*it - 1].pickY;
                double distance = sqrt(pow(x, 2) + pow(y, 2));
                return UtilityInterval(1 / distance, 1);
            }
        }
    }
    return UtilityInterval(0, 1);
}

} /* namespace alica */
