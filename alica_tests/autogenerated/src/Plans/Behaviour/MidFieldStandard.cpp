using namespace std;
#include "Plans/Behaviour/MidFieldStandard.h"

/*PROTECTED REGION ID(inccpp1402488696205) ENABLED START*/ // Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1402488696205) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
MidFieldStandard::MidFieldStandard()
        : DomainBehaviour("MidFieldStandard")
{
    /*PROTECTED REGION ID(con1402488696205) ENABLED START*/ // Add additional options here
    this->callCounter = 0;
    /*PROTECTED REGION END*/
}
MidFieldStandard::~MidFieldStandard()
{
    /*PROTECTED REGION ID(dcon1402488696205) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void MidFieldStandard::run(void* msg)
{
    /*PROTECTED REGION ID(run1402488696205) ENABLED START*/ // Add additional options here
    callCounter++;
    if (callCounter > 10) {
        this->setSuccess();
    }
    /*PROTECTED REGION END*/
}
void MidFieldStandard::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1402488696205) ENABLED START*/ // Add additional options here
    this->callCounter = 0;
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1402488696205) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
