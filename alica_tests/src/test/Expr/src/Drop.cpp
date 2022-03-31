#include "Drop.h"
#include <memory>

/*PROTECTED REGION ID(inccpp3009473645416620380) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars3009473645416620380) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

Drop::Drop(IAlicaWorldModel* wm)
        : DomainBehaviour(wm, "Drop")
{
    /*PROTECTED REGION ID(con3009473645416620380) ENABLED START*/
    // Add additional options here
    _worldModel = dynamic_cast<alicaTests::TaskInstantiationIntegrationWorldModel*>(wm);
    /*PROTECTED REGION END*/
}
Drop::~Drop()
{
    /*PROTECTED REGION ID(dcon3009473645416620380) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void Drop::run(void* msg)
{
    /*PROTECTED REGION ID(run3009473645416620380) ENABLED START*/
    // Add additional options here
    if (isSuccess()) {
        return;
    }

    std::lock_guard<std::mutex> guard(_worldModel->sharedWorldModel->mtx);
    int64_t assignedPayload = _worldModel->sharedWorldModel->payloadAssignments[_worldModel->agentId].value();
    _worldModel->sharedWorldModel->payloads[assignedPayload].state = alicaTests::PayloadState::DROPPED;
    std::cout << "[DROP] payload " << assignedPayload << std::endl;

    std::cout << "[UNASSIGN] payload " << _worldModel->sharedWorldModel->payloadAssignments[_worldModel->agentId].value() << std::endl;
    _worldModel->sharedWorldModel->payloadAssignments[_worldModel->agentId] = std::nullopt;

    setSuccess();
    /*PROTECTED REGION END*/
}
void Drop::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters3009473645416620380) ENABLED START*/
    // Add additional options here

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods3009473645416620380) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
