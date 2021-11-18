#include "Behaviour/TriggerA.h"
#include <memory>

/*PROTECTED REGION ID(inccpp1428508297492) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars1428508297492) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

TriggerA::TriggerA(IAlicaWorldModel* wm)
        : DomainBehaviour("TriggerA", wm)
{
    /*PROTECTED REGION ID(con1428508297492) ENABLED START*/
    // Add additional options here
    this->callCounter = 0;
    this->initCounter = 0;
    /*PROTECTED REGION END*/
}
TriggerA::~TriggerA()
{
    /*PROTECTED REGION ID(dcon1428508297492) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void TriggerA::run(void* msg)
{
    /*PROTECTED REGION ID(run1428508297492) ENABLED START*/
    // Add additional options here
    callCounter++;
    /*PROTECTED REGION END*/
}
void TriggerA::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1428508297492) ENABLED START*/
    // Add additional options here
    callCounter = 0;
    initCounter++;

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1428508297492) ENABLED START*/
// Add additional methods here
/*PROTECTED REGION END*/

} /* namespace alica */
