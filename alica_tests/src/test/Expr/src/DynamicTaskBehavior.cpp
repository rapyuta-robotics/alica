#include "DynamicTaskBehavior.h"
#include <memory>

/*PROTECTED REGION ID(inccpp4044546549214673470) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars4044546549214673470) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

DynamicTaskBehavior::DynamicTaskBehavior(IAlicaWorldModel* wm)
        : DomainBehaviour(wm, "DynamicTaskBehavior")
{
    /*PROTECTED REGION ID(con4044546549214673470) ENABLED START*/
    // Add additional options here
    this->callCounter = 0;
    this->initCounter = 0;
    /*PROTECTED REGION END*/
}
DynamicTaskBehavior::~DynamicTaskBehavior()
{
    /*PROTECTED REGION ID(dcon4044546549214673470) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void DynamicTaskBehavior::run(void* msg)
{
    /*PROTECTED REGION ID(run4044546549214673470) ENABLED START*/
    // Add additional options here
    callCounter++;
    /*PROTECTED REGION END*/
}
void DynamicTaskBehavior::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters4044546549214673470) ENABLED START*/
    // Add additional options here
    callCounter = 0;
    initCounter++;

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods4044546549214673470) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
