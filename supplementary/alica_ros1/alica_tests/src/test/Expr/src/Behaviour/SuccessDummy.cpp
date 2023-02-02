#include <alica_tests/Behaviour/SuccessDummy.h>
#include <memory>

/*PROTECTED REGION ID(inccpp3505111757300078074) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars3505111757300078074) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

SuccessDummy::SuccessDummy(BehaviourContext& context)
        : DomainBehaviour(context)
{
    /*PROTECTED REGION ID(con3505111757300078074) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SuccessDummy::~SuccessDummy()
{
    /*PROTECTED REGION ID(dcon3505111757300078074) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void SuccessDummy::run()
{
    /*PROTECTED REGION ID(run3505111757300078074) ENABLED START*/
    // Add additional options here
    setSuccess();
    /*PROTECTED REGION END*/
}
void SuccessDummy::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters3505111757300078074) ENABLED START*/
    // Add additional options here

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods3505111757300078074) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
