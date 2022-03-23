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
    for (auto* wm : *(_worldModel->wms.get())) {
        wm->payloads[_worldModel->currentPayloadId.value()].state = alicaTests::PayloadState::DROPPED;
    }
    std::cout << "[DROP] payload " << _worldModel->currentPayloadId.value() << std::endl;
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
