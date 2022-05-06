#include <memory>
#include <test/Behaviour/BehBAA.h>

/*PROTECTED REGION ID(inccpp1629895911592) ENABLED START*/
// Add additional includes here
#include "engine/PlanInterface.h"
#include <alica_tests/test_sched_world_model.h>
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars1629895911592) ENABLED START*/
// initialise static variables here
int BehBAA::runCount;
/*PROTECTED REGION END*/

BehBAA::BehBAA(BehaviourContext& context)
        : DomainBehaviour(context)
{
    /*PROTECTED REGION ID(con1629895911592) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
BehBAA::~BehBAA()
{
    /*PROTECTED REGION ID(dcon1629895911592) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void BehBAA::run(void* msg)
{
    /*PROTECTED REGION ID(run1629895911592) ENABLED START*/
    // Add additional options here
    ++runCount;
    /*PROTECTED REGION END*/
}
void BehBAA::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1629895911592) ENABLED START*/
    // Add additional options here
    _wm = dynamic_cast<alica_test::SchedWM*>(getWorldModel());
    _wm->execOrder += "BehBAA::Init\n";
    runCount = 0;

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1629895911592) ENABLED START*/
// Add additional options here
void BehBAA::onTermination()
{
    runCount = 0;
    _wm->execOrder += "BehBAA::Term\n";
}
/*PROTECTED REGION END*/

} /* namespace alica */
