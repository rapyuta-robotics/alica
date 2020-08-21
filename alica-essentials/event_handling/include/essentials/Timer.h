#pragma once

#include "essentials/ITrigger.hpp"

#include <chrono>
#include <condition_variable>

namespace std
{
class thread;
}

namespace essentials
{
/**
 * The Timer allows to register several condition variables.
 * The condition variables are notified according to the timers configuration.
 */
class Timer : public ITrigger
{
public:
    Timer(std::chrono::milliseconds msInterval, std::chrono::milliseconds msDelayedStart, bool notifyAllThreads);
    ~Timer() override;
    /**
     * Sets the timer to run.
     * @return True, if the timer will run after this call. False, otherwise.
     */
    bool start();
    /**
     * Sets the timer to stop.
     * @return True, if the timer will anyways run after this call. False, otherwise.
     */
    bool stop();
    /**
     * Allows to get the running state of the timer.
     * @return True, if the timer is set to run. False, otherwise.
     */
    bool isStarted() const;
    std::chrono::milliseconds getDelayedStart() const;
    std::chrono::milliseconds getInterval() const;
    void run(bool notifyAllThreads) override;

private:
    bool _running; /** < Is always true except when the timer is shutting down. */
    bool _started;
    bool _justStartedAgain;
    std::thread* _runThread;
    std::chrono::milliseconds _msInterval;     /** < The time between two fired events */
    std::chrono::milliseconds _msDelayedStart; /** < The time between starting the TimerEvent and the first fired event */
    std::mutex _cv_mtx;
    std::condition_variable _cv;
};
} /* namespace essentials */
