#include "Behaviour/BehAAA.h"
#include <memory>

/*PROTECTED REGION ID(inccpp1629895901559) ENABLED START*/
// Add additional includes here
#include <alica_tests/test_sched_world_model.h>
#include "engine/PlanInterface.h"
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
    ++runCount;
    _wm->behAAARunCalled = true;
    if (!_inRunContext) {
        _wm->behAAARunOutOfOrder = true;
    }

    if (_wm->behAAASetFailure) {
        setFailure();
        if (!isFailure()) {
            _wm->behAAASetFailureFailed = true;
        }
    }
    if (_wm->behAAASetSuccess) {
        setSuccess();
        if (!isSuccess()) {
            _wm->behAAASetSuccessFailed = true;
        }
    }

    while (_wm->behAAABlockRun) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    /*PROTECTED REGION END*/
}
void BehAAA::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1629895901559) ENABLED START*/
    // Add additional options here
    runCount = 0;
    _wm = dynamic_cast<alica_test::SchedWM*>(getPlanContext().getRunningPlan()->getWorldModel());
    _wm->execOrder += "BehAAA::Init\n";
    _inRunContext = true;

    if (isSuccess()) {
        _wm->behAAASuccessInInit = true;
    }
    if (isFailure()) {
        _wm->behAAAFailureInInit = true;
    }

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1629895901559) ENABLED START*/
// Add additional options here
void BehAAA::onTermination()
{
    runCount = 0;
    _inRunContext = false;
    _wm->execOrder += "BehAAA::Term\n";

    if (isSuccess()) {
        _wm->behAAASuccessInTerminate = true;
    }
    setSuccess();
    if (isSuccess()) {
        _wm->behAAASuccessInTerminate = true;
    }

    if (isFailure()) {
        _wm->behAAAFailureInTerminate = true;
    }
    setFailure();
    if (isFailure()) {
        _wm->behAAAFailureInTerminate = true;
    }
}
/*PROTECTED REGION END*/

} /* namespace alica */
