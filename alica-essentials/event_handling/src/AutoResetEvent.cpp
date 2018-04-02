#include "supplementary/AutoResetEvent.h"

namespace supplementary {

AutoResetEvent::AutoResetEvent(bool initial) : flag(initial) {
    this->waitingThread = 0;
}

AutoResetEvent::~AutoResetEvent() {}
void AutoResetEvent::set() {
    std::lock_guard<std::mutex> lock(protect);
    flag = true;
    signal.notify_one();
}

void AutoResetEvent::reset() {
    std::lock_guard<std::mutex> lock(protect);
    flag = false;
}

void AutoResetEvent::waitOne() {
    std::unique_lock<std::mutex> lk(protect);
    {
        std::lock_guard<std::mutex> lockWaiting(protectWatingThread);
        this->waitingThread++;
    }
    while (!flag) {
        signal.wait(lk);
    }
    {
        std::lock_guard<std::mutex> lockWaiting(protectWatingThread);
        this->waitingThread++;
    }
    flag = false;
}
bool AutoResetEvent::isThreadWaiting() {
    std::lock_guard<std::mutex> lockWaiting(protectWatingThread);
    if (this->waitingThread > 0) {
        return true;
    }
    return false;
}
} /* namespace supplementary */
