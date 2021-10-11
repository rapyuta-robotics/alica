#include "DisabledTracing.h"
#include <memory>

/*PROTECTED REGION ID(inccpp863651328966767832) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars863651328966767832) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

DisabledTracing::DisabledTracing()
        : DomainBehaviour("DisabledTracing")
{
    /*PROTECTED REGION ID(con863651328966767832) ENABLED START*/
    // Add additional options here
    disableTracing();
    /*PROTECTED REGION END*/
}
DisabledTracing::~DisabledTracing()
{
    /*PROTECTED REGION ID(dcon863651328966767832) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void DisabledTracing::run(void* msg)
{
    /*PROTECTED REGION ID(run863651328966767832) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void DisabledTracing::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters863651328966767832) ENABLED START*/
    // Add additional options here

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods863651328966767832) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
