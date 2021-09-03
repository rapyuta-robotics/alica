#include "Behaviour/CountIndefinitely.h"
#include <memory>

/*PROTECTED REGION ID(inccpp1529456643148) ENABLED START*/
// Add additional includes here
#include <alica_tests/CounterClass.h>
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars1529456643148) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

CountIndefinitely::CountIndefinitely()
        : DomainBehaviour("CountIndefinitely")
{
    /*PROTECTED REGION ID(con1529456643148) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
CountIndefinitely::~CountIndefinitely()
{
    /*PROTECTED REGION ID(dcon1529456643148) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void CountIndefinitely::run(void* msg)
{
    /*PROTECTED REGION ID(run1529456643148) ENABLED START*/
    // Add additional options here
    ++CounterClass::called;
    /*PROTECTED REGION END*/
}
void CountIndefinitely::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1529456643148) ENABLED START*/
    // Add additional options here

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1529456643148) ENABLED START*/
// Add additional methods here
/*PROTECTED REGION END*/

} /* namespace alica */
