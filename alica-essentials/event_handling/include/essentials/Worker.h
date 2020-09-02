#pragma once

#include "essentials/NotifyTimer.hpp"

#include <chrono>
#include <condition_variable>
#include <string>

namespace std
{
class thread;
}

namespace essentials
{
class Timer;
class Worker
{
public:
    Worker(std::string name, std::chrono::milliseconds msInterval, std::chrono::milliseconds msDelayedStart);
    virtual ~Worker();
    virtual void run() = 0; /** < Meant to be overwritten by derived classes. */
    bool stop();
    bool start();
    bool isRunning() const;
    std::string name() const;

private:
    void runInternal();

    std::string _name;                                    /** < The name of this worker. */
    bool _started;                                        /** < Is always true except when the worker is shutting down. */
    std::unique_ptr<essentials::NotifyTimer<Worker>> _timer;      /** < Triggers the condition_variable of the runThread. */
    std::mutex _runCV_mtx;                                /** < For creating the unique_lock, passed to _runCV.wait(..) */
    std::condition_variable _runCV;                       /** < ConditionVariable which is registered at the timer object */
    std::unique_ptr<std::thread> _runThread;              /** < Executes the runInternal and thereby the abstract run method. */
};

} // namespace essentials
