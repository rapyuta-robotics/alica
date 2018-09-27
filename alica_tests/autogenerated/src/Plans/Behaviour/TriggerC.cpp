using namespace std;
#include "Plans/Behaviour/TriggerC.h"

/*PROTECTED REGION ID(inccpp1428508355209) ENABLED START*/ // Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1428508355209) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
TriggerC::TriggerC()
    : DomainBehaviour("TriggerC")
{
    /*PROTECTED REGION ID(con1428508355209) ENABLED START*/ // Add additional options here
    this->callCounter = 0;
    this->initCounter = 0;
    this->behaviourTrigger = alicaTests::TestWorldModel::getOne()->trigger2;
    /*PROTECTED REGION END*/
}
TriggerC::~TriggerC()
{
    /*PROTECTED REGION ID(dcon1428508355209) ENABLED START*/ // Add additional options here
    this->callCounter = 0;
    this->initCounter = 0;
    this->behaviourTrigger = alicaTests::TestWorldModel::getOne()->trigger1;
    /*PROTECTED REGION END*/
}
void TriggerC::run(void* msg)
{
    /*PROTECTED REGION ID(run1428508355209) ENABLED START*/ // Add additional options here
    callCounter++;

    /*PROTECTED REGION END*/
}
void TriggerC::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1428508355209) ENABLED START*/ // Add additional options here
    callCounter = 0;
    initCounter++;
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1428508355209) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
