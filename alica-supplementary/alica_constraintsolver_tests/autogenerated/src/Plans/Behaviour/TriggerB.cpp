using namespace std;
#include "Plans/Behaviour/TriggerB.h"

/*PROTECTED REGION ID(inccpp1428508316905) ENABLED START*/ // Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1428508316905) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
TriggerB::TriggerB()
        : DomainBehaviour("TriggerB")
{
    /*PROTECTED REGION ID(con1428508316905) ENABLED START*/ // Add additional options here
    this->callCounter = 0;
    this->initCounter = 0;
    setTrigger(alicaTests::TestWorldModel::getOne()->trigger1);
    /*PROTECTED REGION END*/
}
TriggerB::~TriggerB()
{
    /*PROTECTED REGION ID(dcon1428508316905) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void TriggerB::run(void* msg)
{
    /*PROTECTED REGION ID(run1428508316905) ENABLED START*/ // Add additional options here
    callCounter++;
    /*PROTECTED REGION END*/
}
void TriggerB::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1428508316905) ENABLED START*/ // Add additional options here
    callCounter = 0;
    initCounter++;
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1428508316905) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
