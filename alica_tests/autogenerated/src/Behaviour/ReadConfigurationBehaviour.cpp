#include "Behaviour/ReadConfigurationBehaviour.h"
#include <memory>

/*PROTECTED REGION ID(inccpp1588061129360) ENABLED START*/
#include "engine/model/Configuration.h"
#include "engine/model/Parameter.h"
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars1588061129360) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

ReadConfigurationBehaviour::ReadConfigurationBehaviour()
        : DomainBehaviour("ReadConfigurationBehaviour")
{
    /*PROTECTED REGION ID(con1588061129360) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
ReadConfigurationBehaviour::~ReadConfigurationBehaviour()
{
    /*PROTECTED REGION ID(dcon1588061129360) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void ReadConfigurationBehaviour::run(void* msg)
{
    /*PROTECTED REGION ID(run1588061129360) ENABLED START*/
    std::cout << this->getName() << ": TestValue is " << testValue << std::endl;
    /*PROTECTED REGION END*/
}
void ReadConfigurationBehaviour::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1588061129360) ENABLED START*/
    std::cout << this->getName() << ": initParams1: TestValue is " << testValue << std::endl;
    getParameter("TestValue", testValue);
    std::cout << this->getName() << ": initParams2: TestValue is " << testValue << std::endl;
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1588061129360) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
