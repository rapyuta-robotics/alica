#Behavoiur Code

**---> explanation**
``` cpp
using namespace std;
#include "Plans/Behaviour/StartDrive.h"

/*PROTECTED REGION ID(inccpp1427727877859) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1427727877859) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    StartDrive::StartDrive() :
            DomainBehaviour("StartDrive")
    {
        /*PROTECTED REGION ID(con1427727877859) ENABLED START*/ //Add additional options here
	**---> Will call once before start**
	**---> c = 0;**
        /*PROTECTED REGION END*/
    }
    StartDrive::~StartDrive()
    {
        /*PROTECTED REGION ID(dcon1427727877859) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void StartDrive::run(void* msg)
    {
        /*PROTECTED REGION ID(run1427727877859) ENABLED START*/ //Add additional options here
	**---> the engine will call this method with 30 Hz**
	**---> c++;**
	**---> x++;**
        /*PROTECTED REGION END*/
    }
    void StartDrive::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1427727877859) ENABLED START*/ //Add additional options here
	**---> Is called by each start meant here is that it is only called if we change the behavior**
	**---> Example initialize variables here or contructor**
	**---> x = 0;**
	**---> after 30 times the runmethod get called and we leave the state**
	**---> c will stay at 30**
	**---> x will be 0 if we start this behaviour again**
	**---> so c will be counting from 30 and x from 0 if we call this behaviour after one minute again**
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1427727877859) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */


```
