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
}

AssignPayloadSummand::~AssignPayloadSummand() {}

UtilityInterval AssignPayloadSummand::eval(IAssignment ass, const Assignment* oldAss, const IAlicaWorldModel* wm) const
{
    auto worldModel = dynamic_cast<const alicaTests::TaskInstantiationIntegrationWorldModel*>(wm);
    std::vector<uint64_t> payloadsReadyForPickup;

    // agents that have already been assigned to a payload should not be reassigned
    for (auto wm : *(worldModel->wms)) {
        if (!wm->currentPayloadId.has_value()) {
            continue;
        }
        auto agentId = wm->agentId;
        auto agentsAssignedToPayload = ass.getAgentsWorking(movePayloadEpId, wm->currentPayloadId.value() + 1);
        if (agentsAssignedToPayload.size() != 1 || agentId != *(agentsAssignedToPayload.begin())) {
            return UtilityInterval(-1, -1);
        }
    }

    // for (uint64_t dynId = 1; dynId < 9; dynId++) {
    for (uint64_t payloadId = 0; payloadId < worldModel->payloads.size(); payloadId++) {
        uint64_t dynId = payloadId + 1;
        auto agentsInEntryPoint = ass.getAgentsWorking(movePayloadEpId, payloadId + 1);

        // dont accept assignments with multiple agents in one entry point
        if (agentsInEntryPoint.size() > 1) {
            return UtilityInterval(-1, -1);
        }

        // agents should not be assigned to payloads that have already been moved
        if (agentsInEntryPoint.size() > 0 && worldModel->payloads.at(payloadId).state == alicaTests::PayloadState::DROPPED) {
            return UtilityInterval(-1, -1);
        }

        // collect payloads have not been picked yet
        if (worldModel->payloads.at(payloadId).state == alicaTests::PayloadState::READY_FOR_PICKUP) {
            payloadsReadyForPickup.push_back(payloadId);
        }
    }
    std::vector<double> distances;
    for (auto it = payloadsReadyForPickup.begin(); it != payloadsReadyForPickup.end(); it++) {
        uint64_t dynId = *it - 1;
        for (AgentId agentId : ass.getAgentsWorking(movePayloadEpId, dynId)) {
            int x = worldModel->agentLocations.at(agentId).first - worldModel->payloads[*it].pickX;
            int y = worldModel->agentLocations.at(agentId).second - worldModel->payloads[*it].pickY;
            double distance = sqrt(pow(x, 2) + pow(y, 2));
            if (distance < 1) {
                distance = 1;
            }
            distances.push_back(distance);
        }
    }
    double utilityValue = 0;
    for (auto it = distances.begin(); it != distances.end(); it++) {
        utilityValue += 1 / *it;
    }
    return UtilityInterval(0, utilityValue);
}

} /* namespace alica */
