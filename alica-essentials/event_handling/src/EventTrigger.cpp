#include "essentials/EventTrigger.h"

namespace essentials
{
void EventTrigger::run(bool notifyAll)
{
    notifyEveryCV(notifyAll);
}
} // namespace essentials
