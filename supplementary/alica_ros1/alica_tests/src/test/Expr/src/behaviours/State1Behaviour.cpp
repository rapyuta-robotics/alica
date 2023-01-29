#include <alica_tests/behaviours/State1Behaviour.h>
#include <memory>

/*PROTECTED REGION ID(inccpp3563417394101512880) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars3563417394101512880) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

State1Behaviour::State1Behaviour(BehaviourContext& context)
        : DomainBehaviour(context)
{
    /*PROTECTED REGION ID(con3563417394101512880) ENABLED START*/
    // Add additional options here
    callCounter = 0;
    /*PROTECTED REGION END*/
}
State1Behaviour::~State1Behaviour()
{
    /*PROTECTED REGION ID(dcon3563417394101512880) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void State1Behaviour::run()
{
    /*PROTECTED REGION ID(run3563417394101512880) ENABLED START*/
    // Add additional options here
    callCounter++;
    if (callCounter > 10) {
        setSuccess();
    }
    /*PROTECTED REGION END*/
}
void State1Behaviour::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters3563417394101512880) ENABLED START*/
    // Add additional options here
    callCounter = 0;

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods3563417394101512880) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
