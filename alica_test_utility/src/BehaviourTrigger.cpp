#include "alica/test/BehaviourTrigger.h"

namespace alica::test
{
void BehaviourTrigger::trigger()
{
    this->notifyEveryCV(false);
}

bool BehaviourTrigger::behaviourFinishedRun()
{
    if (isAnyCVRegistered()) {
        return !this->isNotifyCalled(_cv);
    }
    return false;
}

void BehaviourTrigger::registerCV(std::condition_variable* cv)
{
    ITrigger::registerCV(cv);
    _cv = cv;
}
} // namespace alica::test
