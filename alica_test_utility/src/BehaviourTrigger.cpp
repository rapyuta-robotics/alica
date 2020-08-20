#include "alica/test/BehaviourTrigger.h"

namespace alica::test
{
void BehaviourTrigger::trigger()
{
    std::lock_guard<std::mutex>lockGuard(this->cvVec_mtx);
    this->notifyAll(false);
}

bool BehaviourTrigger::behaviourFinishedRun()
{
    std::lock_guard<std::mutex>lockGuard(this->cvVec_mtx);
    if (registeredCVs.size() > 0) {
        return !(registeredCVs.begin()->second);
    }
    return false;
}
} // namespace alica
