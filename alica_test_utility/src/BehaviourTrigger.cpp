#include "alica/test/BehaviourTrigger.h"

namespace alica::test
{
void BehaviourTrigger::trigger()
{
    this->notifyAll(false);
}

bool BehaviourTrigger::behaviourFinishedRun()
{
    if (registeredCVs.size() > 0) {
        return !this->isNotifyCalled(registeredCVs.begin()->first);
    }
    return false;
}
} // namespace alica
