#include "Navigate.h"
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

Navigate::Navigate(IAlicaWorldModel* wm)
        : DomainBehaviour(wm, "Navigate")
{
    /*PROTECTED REGION ID(con4505472195947429717) ENABLED START*/
    // Add additional options here
    _worldModel = dynamic_cast<alicaTests::TaskInstantiationIntegrationWorldModel*>(wm);
    /*PROTECTED REGION END*/
}
Navigate::~Navigate()
{
    /*PROTECTED REGION ID(dcon4505472195947429717) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void Navigate::run(void* msg)
{
    /*PROTECTED REGION ID(run4505472195947429717) ENABLED START*/
    // Add additional options here
    if (isSuccess()) {
        return;
    }

    if (_action == "PICK") {
        _worldModel->agentLocations[_agentId].first = _worldModel->payloads[_worldModel->currentPayloadId].pickX;
        _worldModel->agentLocations[_agentId].second = _worldModel->payloads[_worldModel->currentPayloadId].pickY;
        
    } else if (_action == "DROP") {
        _worldModel->agentLocations[_agentId].first = _worldModel->payloads[_worldModel->currentPayloadId].dropX;
        _worldModel->agentLocations[_agentId].second = _worldModel->payloads[_worldModel->currentPayloadId].dropY;
    }
    std::cout << "Move robot " << _agentId << " to position " << _worldModel->agentLocations[_agentId].first << " | "
            << _worldModel->agentLocations[_agentId].second << std::endl;   
    setSuccess();
    /*PROTECTED REGION END*/
}
void Navigate::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters4505472195947429717) ENABLED START*/
    // Add additional options here
    LockedBlackboardRW bb = LockedBlackboardRW(*(getBlackboard()));
    _action = bb.get<std::string>("nav_action");
    _agentId = getEngine()->getTeamManager().getLocalAgentID();
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods4505472195947429717) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
