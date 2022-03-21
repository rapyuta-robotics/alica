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
        std::cerr << "assigning payload " << _worldModel->currentPayloadId << " to " << _worldModel->agentId << " with dynId " << rp->getParent()->getActiveEntryPoint()->getDynamicId() << std::endl;
        for (uint64_t dynId = 1; dynId < 9; dynId++) {
            int64_t payloadId = dynId - 1;
            auto agentsInEntryPoint = rp->getParent()->getAssignment().getAgentsWorking(rp->getParent()->getActiveEntryPoint()->getId(), dynId);
            if (agentsInEntryPoint.size() > 0) {
                std::cerr << "agents assigned to " << dynId << ": " << *(agentsInEntryPoint.begin()) << std::endl;
            }
        }
    }

    

    setSuccess();
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
