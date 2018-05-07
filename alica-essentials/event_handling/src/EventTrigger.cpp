#include "supplementary/EventTrigger.h"

namespace supplementary {
EventTrigger::EventTrigger() {}

EventTrigger::~EventTrigger() {}

void EventTrigger::run(bool notifyAll) {
    std::lock_guard<std::mutex> lock(cv_mtx);
    this->notifyAll(notifyAll);
}
}  // namespace supplementary
