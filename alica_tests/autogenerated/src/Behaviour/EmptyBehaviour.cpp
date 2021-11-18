#include "Behaviour/EmptyBehaviour.h"
#include <memory>

/*PROTECTED REGION ID(inccpp1625610857563) ENABLED START*/
// Add additional includes here
#include <alica_tests/CounterClass.h>
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars1625610857563) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

EmptyBehaviour::EmptyBehaviour(IAlicaWorldModel* wm)
        : DomainBehaviour("EmptyBehaviour", wm)
{
    /*PROTECTED REGION ID(con1625610857563) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
EmptyBehaviour::~EmptyBehaviour()
{
    /*PROTECTED REGION ID(dcon1625610857563) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void EmptyBehaviour::run(void* msg)
{
    /*PROTECTED REGION ID(run1625610857563) ENABLED START*/
    // Add additional options here
    ++CounterClass::called;
    /*PROTECTED REGION END*/
}
void EmptyBehaviour::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1625610857563) ENABLED START*/
    // Add additional options here
    CounterClass::called = 0;

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1625610857563) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
