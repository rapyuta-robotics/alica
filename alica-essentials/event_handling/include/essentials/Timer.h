#pragma once

#include "essentials/ITrigger.h"

#include <vector>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <iostream>

namespace essentials {
/**
 * The TimerEvent allows to register several condition variables.
 * The condition variables are notified according to the timers configuration.
 */
class Timer : public virtual ITrigger {
public:
    Timer(long msInterval, long msDelayedStart);
    ~Timer();
    bool start();
    bool stop();
    bool isRunning();
    bool isStarted();
    void setDelayedStart(long msDelayedStart);
    void setInterval(long msInterval);
    const long getDelayedStart() const;
    const long getInterval() const;
    void run(bool notifyAll = true);

private:
    std::thread* runThread;
    std::chrono::milliseconds msInterval; /** < The time between two fired events */
    std::chrono::milliseconds
            msDelayedStart; /** < The time between starting the TimerEvent and the first fired event */
    bool running, started, triggered;
    std::condition_variable cv;
};
} /* namespace essentials */
