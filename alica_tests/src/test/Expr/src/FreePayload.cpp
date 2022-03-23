#include "FreePayload.h"
#include <memory>

/*PROTECTED REGION ID(inccpp422054015709952219) ENABLED START*/
// Add additional includes here
#include <optional>
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars422054015709952219) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

FreePayload::FreePayload(IAlicaWorldModel* wm)
        : DomainBehaviour(wm, "FreePayload")
{
    /*PROTECTED REGION ID(con422054015709952219) ENABLED START*/
    // Add additional options here
    _worldModel = dynamic_cast<alicaTests::TaskInstantiationIntegrationWorldModel*>(wm);
    /*PROTECTED REGION END*/
}
FreePayload::~FreePayload()
{
    /*PROTECTED REGION ID(dcon422054015709952219) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void FreePayload::run(void* msg)
{
    /*PROTECTED REGION ID(run422054015709952219) ENABLED START*/
    // Add additional options here
    if (isSuccess()) {
        return;
    }
    std::cout << "[UNASSIGN] payload " << _worldModel->currentPayloadId.value() << std::endl;
    _worldModel->currentPayloadId = std::nullopt;
    setSuccess();
    /*PROTECTED REGION END*/
}
void FreePayload::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters422054015709952219) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods422054015709952219) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
