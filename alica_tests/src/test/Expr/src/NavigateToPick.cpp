#include "NavigateToPick.h"
#include <memory>

/*PROTECTED REGION ID(inccpp4505472195947429717) ENABLED START*/
// Add additional includes here
#include <engine/AlicaEngine.h>
#include <engine/PlanInterface.h>
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars4505472195947429717) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

NavigateToPick::NavigateToPick(IAlicaWorldModel* wm)
        : DomainBehaviour(wm, "NavigateToPick")
{
    /*PROTECTED REGION ID(con4505472195947429717) ENABLED START*/
    // Add additional options here
    _worldModel = dynamic_cast<alicaTests::TaskInstantiationIntegrationWorldModel*>(wm);
    /*PROTECTED REGION END*/
}
NavigateToPick::~NavigateToPick()
{
    /*PROTECTED REGION ID(dcon4505472195947429717) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void NavigateToPick::run(void* msg)
{
    /*PROTECTED REGION ID(run4505472195947429717) ENABLED START*/
    // Add additional options here
    if (isSuccess()) {
        return;
    }

    std::lock_guard<std::mutex> guard(_worldModel->sharedWorldModel->mtx);
    int64_t assignedPayload = _worldModel->sharedWorldModel->payloadAssignments[_worldModel->agentId].value();
    _worldModel->sharedWorldModel->agentLocations[_worldModel->agentId].first = _worldModel->sharedWorldModel->payloads[assignedPayload].pickX;
    _worldModel->sharedWorldModel->agentLocations[_worldModel->agentId].second = _worldModel->sharedWorldModel->payloads[assignedPayload].pickY;

    std::cout << "[MOVE TO PICK] agent " << _agentId << " to position " << _worldModel->sharedWorldModel->agentLocations[_agentId].first << " | "
              << _worldModel->sharedWorldModel->agentLocations[_agentId].second << " (payload " << assignedPayload << ")" << std::endl;
    setSuccess();
    /*PROTECTED REGION END*/
}
void NavigateToPick::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters4505472195947429717) ENABLED START*/
    // Add additional options here
    _agentId = getEngine()->getTeamManager().getLocalAgentID();
    auto context = getPlanContext();
    if (context.isValid()) {
        auto rp = context.getRunningPlan();

        std::lock_guard<std::mutex> guard(_worldModel->sharedWorldModel->mtx);
        int64_t assignedPayload = rp->getParent()->getActiveEntryPoint()->getDynamicId() - 1;
        _worldModel->sharedWorldModel->payloadAssignments[_agentId] = assignedPayload;
        std::cout << "[ASSIGN] payload " << assignedPayload << " to agent " << _agentId << std::endl;
    }

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods4505472195947429717) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
