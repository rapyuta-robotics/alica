#include <libalica-tests/behaviours/BehAAA.h>
#include <memory>

#include <alica_tests/test_sched_world_model.h>

namespace alica
{
int BehAAA::runCount;

BehAAA::BehAAA(BehaviourContext& context)
        : BasicBehaviour(context)
{
}
BehAAA::~BehAAA() {}
void BehAAA::run()
{
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
}
void BehAAA::initialiseParameters()
{
    runCount = 0;
    _wm = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    _wm->execOrder += "BehAAA::Init\n";
    _inRunContext = true;

    if (isSuccess()) {
        _wm->behAAASuccessInInit = true;
    }
    if (isFailure()) {
        _wm->behAAAFailureInInit = true;
    }
}

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

std::unique_ptr<BehAAA> BehAAA::create(alica::BehaviourContext& context)
{
    return std::make_unique<BehAAA>(context);
}

} /* namespace alica */
