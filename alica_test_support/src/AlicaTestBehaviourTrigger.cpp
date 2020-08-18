#include "alica/AlicaTestBehaviourTrigger.h"

namespace alica
{
void AlicaTestBehaviourTrigger::trigger()
{
    this->notifyAll(false);
}

bool AlicaTestBehaviourTrigger::behaviourFinishedRun()
{
    return registeredCVs.size() > 1 && !registeredCVs.begin()->second;
}
} // namespace alica
