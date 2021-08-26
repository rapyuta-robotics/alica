#include "Behaviour/BehAAA.h"
#include <memory>

/*PROTECTED REGION ID(inccpp1629895901559) ENABLED START*/
// Add additional includes here
#include <alica_tests/test_sched_world_model.h>
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars1629895901559) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

BehAAA::BehAAA()
        : DomainBehaviour("BehAAA")
{
    /*PROTECTED REGION ID(con1629895901559) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
BehAAA::~BehAAA()
{
    /*PROTECTED REGION ID(dcon1629895901559) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void BehAAA::run(void* msg)
{
    /*PROTECTED REGION ID(run1629895901559) ENABLED START*/
    // Add additional options here
    auto& wm = alica_test::SchedWM::instance();
    wm.behAAARunCalled = true;
    if (!_inRunContext) {
        wm.behAAARunOutOfOrder = true;
    }
    /*PROTECTED REGION END*/
}
void BehAAA::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1629895901559) ENABLED START*/
    // Add additional options here
    alica_test::SchedWM::instance().execOrder += "BehAAA::Init\n";
    _inRunContext = true;
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1629895901559) ENABLED START*/
// Add additional options here
void BehAAA::onTermination()
{
    _inRunContext = false;
    alica_test::SchedWM::instance().execOrder += "BehAAA::Term\n";
}
/*PROTECTED REGION END*/

} /* namespace alica */
