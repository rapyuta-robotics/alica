#pragma once

#include "engine/AlicaClock.h"

#include <functional>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>

namespace alica
{

class SyncStopTimer
{
public:
    using TimerCb = std::function<void()>;

    SyncStopTimer(const SyncStopTimer&) = delete;
    SyncStopTimer(SyncStopTimer&&) = default;
    SyncStopTimer& operator=(const SyncStopTimer&) = delete;
    SyncStopTimer& operator=(SyncStopTimer&&) = default;

    ~SyncStopTimer()
    {
        stop();
    }

    void start()
    {
        _timer.start();
    }

    void stop()
    {
        // TODO: get rid of mutex lock for checking if userCb is in progress
        _timer.stop();
        std::unique_lock<std::mutex> lock(_stopMutex);
        _stop = true;
        if (!_userCbInProgress) {
            return;
        }
        _stopCv.wait(lock, [this](){ return !_userCbInProgress; });
    }

private:
    SyncStopTimer(TimerCb&& userCb, ros::NodeHandle& nh, ros::Duration period)
        : _userCb(std::move(userCb))
        , _timer(nh.createTimer(period, timerCb, this, false, false))
        , _stopMutex()
        , _stopCv()
        , _userCbInProgress(false)
        , _stop(false)
    {
        start();
    }

    void timerCb(const ros::TimerEvent&)
    {
        if (!_userCb) {
            return;
        }

        {
            std::unique_lock<std::mutex> lock(_stopMutex);
            if (_stop) {
                return;
            }
            _userCbInProgress = true;
        }

        _userCb();

        std::unique_lock<std::mutex> lock(_stopMutex);
        _userCbInProgress = false;
        if (_stop) {
            _stopCv.notify_one();
        }
    }

    TimerCb _userCb;
    ros::Timer _timer;
    mutable std::mutex _stopMutex;
    std::condition_variable _stopCv;
    bool _stop;
    bool _userCbInProgress;
};

// TODO: implement a async stop timer i.e. the stop() method should take a callback as parameter which will be called when the timer is successfully stopped

// TODO: this is a policy based design which only takes the timer policy for now. Can extend to take more policy classes like threadpool & callback queues if need be
template <class Timer>
class AlicaTimerManager
{
    friend Timer;
    using TimerCb = typename Timer::TimerCb;

public:
    AlicaTimerManager(int numThreads);
    AlicaTimerManager(const AlicaTimerManager&) = delete;
    AlicaTimerManager(AlicaTimerManager&&) = delete;
    AlicaTimerManager& operator=(const AlicaTimerManager&) = delete;
    AlicaTimerManager&& operator=(AlicaTimerManager&&) = delete;
    ~AlicaTimerManager() = default;

    Timer createTimer(TimerCb timerCb, AlicaTime period)
    {
        auto sec = period.inSeconds();
        auto nanosec = period.inNanoseconds();
        nanosec -= sec * 1000000000LL;
        return Timer(timerCb, _nh, ros::Duration{sec, nanosec});
    }

    // We may require a shared_ptr to the implementation so that this class stays alive as long as a timer is alive
    // Is probably not required here since the ros takes care of it

private:
    ros::NodeHandle _nh;
};

}