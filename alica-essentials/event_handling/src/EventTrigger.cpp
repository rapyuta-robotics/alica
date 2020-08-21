#include "essentials/EventTrigger.h"

namespace essentials
{
EventTrigger::EventTrigger() {}

EventTrigger::~EventTrigger() {}

void EventTrigger::run(bool notifyAll)
{
    this->notifyAll(notifyAll);
}
} // namespace essentials
