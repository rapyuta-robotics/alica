#include "Behaviour/AlwaysFail.h"
#include <memory>

/*PROTECTED REGION ID(inccpp1532424188199) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars1532424188199) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

AlwaysFail::AlwaysFail(IAlicaWorldModel* wm)
        : DomainBehaviour("AlwaysFail", wm)
{
    /*PROTECTED REGION ID(con1532424188199) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
AlwaysFail::~AlwaysFail()
{
    /*PROTECTED REGION ID(dcon1532424188199) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void AlwaysFail::run(void* msg)
{
    /*PROTECTED REGION ID(run1532424188199) ENABLED START*/
    // Add additional options here
    setFailure();
    /*PROTECTED REGION END*/
}
void AlwaysFail::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1532424188199) ENABLED START*/
    // Add additional options here

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1532424188199) ENABLED START*/
// Add additional methods here
/*PROTECTED REGION END*/

} /* namespace alica */
