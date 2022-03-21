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
    _worldModel->agentLocations[_agentId].first = _worldModel->payloads[_worldModel->currentPayloadId].dropX;
    _worldModel->agentLocations[_agentId].second = _worldModel->payloads[_worldModel->currentPayloadId].dropY;

    std::cout << "Move robot " << _agentId << " to position " << _worldModel->agentLocations[_agentId].first << " | "
              << _worldModel->agentLocations[_agentId].second << " payload " << _worldModel->currentPayloadId << std::endl;
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
