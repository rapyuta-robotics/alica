#include "NavigateToDrop.h"
#include <memory>

/*PROTECTED REGION ID(inccpp4459885370764933844) ENABLED START*/
// Add additional includes here
#include <engine/AlicaEngine.h>
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars4459885370764933844) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

NavigateToDrop::NavigateToDrop(IAlicaWorldModel* wm)
        : DomainBehaviour(wm, "NavigateToDrop")
{
    /*PROTECTED REGION ID(con4459885370764933844) ENABLED START*/
    // Add additional options here
    _worldModel = dynamic_cast<alicaTests::TaskInstantiationIntegrationWorldModel*>(wm);
    /*PROTECTED REGION END*/
}
NavigateToDrop::~NavigateToDrop()
{
    /*PROTECTED REGION ID(dcon4459885370764933844) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void NavigateToDrop::run(void* msg)
{
    /*PROTECTED REGION ID(run4459885370764933844) ENABLED START*/
    // Add additional options here
    if (isSuccess()) {
        return;
    }

    std::lock_guard<std::mutex> guard(_worldModel->sharedWorldModel->mtx);
    int64_t assignedPayload = _worldModel->sharedWorldModel->payloadAssignments[_worldModel->agentId].value();
    _worldModel->sharedWorldModel->agentLocations[_worldModel->agentId].first = _worldModel->sharedWorldModel->payloads[assignedPayload].dropX;
    _worldModel->sharedWorldModel->agentLocations[_worldModel->agentId].second = _worldModel->sharedWorldModel->payloads[assignedPayload].dropY;

    std::cout << "[MOVE TO DROP] agent " << _agentId << " to position " << _worldModel->sharedWorldModel->agentLocations[_agentId].first << " | "
              << _worldModel->sharedWorldModel->agentLocations[_agentId].second << " (payload " << assignedPayload << ")" << std::endl;
    setSuccess();
    /*PROTECTED REGION END*/
}
void NavigateToDrop::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters4459885370764933844) ENABLED START*/
    // Add additional options here
    _agentId = getEngine()->getTeamManager().getLocalAgentID();

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods4459885370764933844) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
