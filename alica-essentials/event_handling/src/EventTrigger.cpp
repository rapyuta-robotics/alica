#include "essentials/EventTrigger.h"

namespace essentials
{
void EventTrigger::run(bool notifyAll)
{
    this->notifyEveryCV(notifyAll);
}
} // namespace essentials
