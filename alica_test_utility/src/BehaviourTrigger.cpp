#include "alica/test/BehaviourTrigger.h"

namespace alica::test
{
void BehaviourTrigger::run(bool notifyAll)
{
    this->notifyEveryCV(false);
}
} // namespace alica::test
