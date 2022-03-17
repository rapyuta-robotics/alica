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
    // for (uint64_t dynId = 1; dynId < 9; dynId++) {
    //     std::cerr << "only one robot per dyn ep" << std::endl;
    //     for (AgentId agentId : ass.getAgentsWorking(movePayloadEpId, dynId)) {
    //         std::cerr << "working agent: " << agentId << " in " << dynId << std::endl;
    //     }
    //     if (ass.getAgentsWorking(movePayloadEpId, dynId).size() > 1) {
    //         return UtilityInterval(-1, -1);
    //     }
    // }
    // for (int64_t dynId = 1; dynId < 5; dynId++) {
    //     if (ass.getAgentsWorking(movePayloadEpId, dynId).size() > 1) {
    //         return UtilityInterval(-1, -1);
    //     }
    //     // for (AgentId agentId : ass.getAgentsWorking(movePayloadEpId, dynId)) {
    //     //     if (agentId == this->robotId) {        
    //     //         const alicaTests::TaskInstantiationIntegrationWorldModel* worldModel = dynamic_cast<const alicaTests::TaskInstantiationIntegrationWorldModel*>(wm);
    //     //         double x = worldModel->agentLocations.at(agentId).first - worldModel->payloads[dynId - 1].pickX;
    //     //         double y = worldModel->agentLocations.at(agentId).second - worldModel->payloads[dynId - 1].pickY;
    //     //         double distance = pow(x, 2) + pow(y, 2);
    //     //         return UtilityInterval(1 / distance, 1);
    //     //     }
    //     // }
    // }

    UtilityInterval ui(0, 1);
    const alicaTests::TaskInstantiationIntegrationWorldModel* worldModel = dynamic_cast<const alicaTests::TaskInstantiationIntegrationWorldModel*>(wm);

    for (uint64_t agentId = 8; agentId < 12; agentId++) {
        for (AgentId id : ass.getAgentsWorking(movePayloadEpId, worldModel->currentPayloadId + 1)) {
            if (worldModel->agentId == id) {
                return UtilityInterval(0, 1);
            }
        }
    }

    return UtilityInterval(0, 1);
}

} /* namespace alica */
