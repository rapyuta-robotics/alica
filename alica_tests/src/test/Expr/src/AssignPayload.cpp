#include "AssignPayload.h"
#include <memory>

/*PROTECTED REGION ID(inccpp3826644292150922713) ENABLED START*/
// Add additional includes here
#include <engine/PlanInterface.h>
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars3826644292150922713) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

AssignPayload::AssignPayload(IAlicaWorldModel* wm)
        : DomainBehaviour(wm, "AssignPayload")
{
    /*PROTECTED REGION ID(con3826644292150922713) ENABLED START*/
    // Add additional options here
    _worldModel = dynamic_cast<alicaTests::TaskInstantiationIntegrationWorldModel*>(wm);
    /*PROTECTED REGION END*/
}
AssignPayload::~AssignPayload()
{
    /*PROTECTED REGION ID(dcon3826644292150922713) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void AssignPayload::run(void* msg)
{
    /*PROTECTED REGION ID(run3826644292150922713) ENABLED START*/
    // Add additional options here
    if (isSuccess()) {
        return;
    }
    auto context = getPlanContext();
    if (context.isValid()) {
        auto rp = context.getRunningPlan();
        _worldModel->currentPayloadId = rp->getParent()->getActiveEntryPoint()->getDynamicId() - 1;
        std::cout << "[ASSIGN] payload " << _worldModel->currentPayloadId.value() << " to agent " << _worldModel->agentId << std::endl;
        setSuccess();
    }
    /*PROTECTED REGION END*/
}
void AssignPayload::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters3826644292150922713) ENABLED START*/
    // Add additional options here

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods3826644292150922713) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
