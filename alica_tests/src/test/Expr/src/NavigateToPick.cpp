#include "NavigateToPick.h"
#include <memory>

/*PROTECTED REGION ID(inccpp4505472195947429717) ENABLED START*/
// Add additional includes here
#include <engine/AlicaEngine.h>
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

    _worldModel->agentLocations[_agentId].first = _worldModel->payloads[_worldModel->currentPayloadId].pickX;
    _worldModel->agentLocations[_agentId].second = _worldModel->payloads[_worldModel->currentPayloadId].pickY;

    std::cout << "Move robot " << _agentId << " to position " << _worldModel->agentLocations[_agentId].first << " | "
              << _worldModel->agentLocations[_agentId].second << " payload " << _worldModel->currentPayloadId << std::endl;
    setSuccess();
    /*PROTECTED REGION END*/
}
void NavigateToPick::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters4505472195947429717) ENABLED START*/
    // Add additional options here
    _agentId = getEngine()->getTeamManager().getLocalAgentID();
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods4505472195947429717) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
