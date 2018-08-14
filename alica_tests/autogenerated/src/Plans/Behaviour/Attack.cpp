using namespace std;
#include "Plans/Behaviour/Attack.h"

/*PROTECTED REGION ID(inccpp1402488848841) ENABLED START*/ // Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1402488848841) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
Attack::Attack()
        : DomainBehaviour("Attack")
{
    /*PROTECTED REGION ID(con1402488848841) ENABLED START*/ // Add additional options here
    this->callCounter = 0;
    this->initCounter = 0;
    /*PROTECTED REGION END*/
}
Attack::~Attack()
{
    /*PROTECTED REGION ID(dcon1402488848841) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void Attack::run(void* msg)
{
    /*PROTECTED REGION ID(run1402488848841) ENABLED START*/ // Add additional options here
    callCounter++;
    /*PROTECTED REGION END*/
}
void Attack::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1402488848841) ENABLED START*/ // Add additional options here
    callCounter = 0;
    initCounter++;
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1402488848841) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
