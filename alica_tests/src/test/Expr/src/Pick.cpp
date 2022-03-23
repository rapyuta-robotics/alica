#include "Pick.h"
#include <memory>

/*PROTECTED REGION ID(inccpp2580816776008671737) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars2580816776008671737) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

Pick::Pick(IAlicaWorldModel* wm)
        : DomainBehaviour(wm, "Pick")
{
    /*PROTECTED REGION ID(con2580816776008671737) ENABLED START*/
    // Add additional options here
    _worldModel = dynamic_cast<alicaTests::TaskInstantiationIntegrationWorldModel*>(wm);
    // TODO: get local agent ID
    _agentId = 0;
    /*PROTECTED REGION END*/
}
Pick::~Pick()
{
    /*PROTECTED REGION ID(dcon2580816776008671737) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void Pick::run(void* msg)
{
    /*PROTECTED REGION ID(run2580816776008671737) ENABLED START*/
    // Add additional options here
    if (isSuccess()) {
        return;
    }
    for (auto* wm : *(_worldModel->wms.get())) {
        wm->payloads[_worldModel->currentPayloadId.value()].state = alicaTests::PayloadState::PICKED;
    }
    std::cout << "[PICK] payload " << _worldModel->currentPayloadId.value() << std::endl;
    setSuccess();
    /*PROTECTED REGION END*/
}
void Pick::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters2580816776008671737) ENABLED START*/
    // Add additional options here

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods2580816776008671737) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
