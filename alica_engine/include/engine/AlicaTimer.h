#pragma once

#include "engine/AlicaClock.h"

#include <condition_variable>
#include <functional>
#include <mutex>
#include <ros/ros.h>

#include <memory>

namespace alica
{

template <class CallbackQ>
class SyncStopTimerRos
{
public:
    using TimerCb = std::function<void()>;

    SyncStopTimerRos(CallbackQ& cbQ, TimerCb&& userCb, AlicaTime period)
            : _userCb(std::move(userCb))
            , _nh()
            , _timer()
            , _stopMutex(std::make_shared<std::mutex>())
            , _stopCv(std::make_unique<std::condition_variable>())
            , _userCbInProgress(false)
            , _stop(false)
            , _period(period)
    {
        _nh.setCallbackQueue(std::addressof(cbQ));
        initTimer();
    }

    SyncStopTimerRos(const SyncStopTimerRos&) = default;
    SyncStopTimerRos(SyncStopTimerRos&& other)
            : _userCb(std::move(other._userCb))
            , _nh(other._nh)
            , _stopMutex(std::move(other._stopMutex))
            , _stopCv(std::move(other._stopCv))
            , _stop(other._stop)
            , _period(other._period)
    {
        other.stop();
        initTimer();
    }
    SyncStopTimerRos& operator=(const SyncStopTimerRos&) = default;
    SyncStopTimerRos& operator=(SyncStopTimerRos&&) = default;

    ~SyncStopTimerRos() { stop(); }

    void start() { _timer.start(); }

    void stop()
    {
        // TODO: get rid of mutex lock for checking if userCb is in progress
        if (!_stopMutex) {
            return; // AlicaTimer object has been copied and should not stop the timer or do any clean up
        }
        _timer.stop();
        std::unique_lock<std::mutex> lock(*_stopMutex);
        _stop = true;
        if (!_userCbInProgress) {
            return;
        }
        (*_stopCv).wait(lock, [this]() { return !_userCbInProgress; });
    }

private:
    void timerCb(const ros::TimerEvent&)
    {
        if (!_userCb) {
            return;
        }

        {
            std::unique_lock<std::mutex> lock(*_stopMutex);
            if (_stop) {
                return;
            }
            _userCbInProgress = true;
        }

        _userCb();

        std::unique_lock<std::mutex> lock(*_stopMutex);
        _userCbInProgress = false;
        if (_stop) {
            (*_stopCv).notify_one();
        }
    }

    void initTimer()
    {
        auto sec = _period.inSeconds();
        auto nanosec = _period.inNanoseconds();
        nanosec -= sec * 1000000000LL;
        _timer = _nh.createTimer(ros::Duration(sec, nanosec), &SyncStopTimerRos::timerCb, this, false, false);
    }

    TimerCb _userCb;
    ros::NodeHandle _nh;
    ros::Timer _timer;
    std::shared_ptr<std::mutex> _stopMutex;
    std::unique_ptr<std::condition_variable> _stopCv;
    bool _stop;
    bool _userCbInProgress;
    AlicaTime _period;
};

// TODO: implement a AsyncStopTimerRos i.e. the stop() method should take a callback as parameter which will be called asynchronously when the timer is
// successfully stopped

template <class CallbackQ>
class ThreadPoolRos
{
public:
    ThreadPoolRos(CallbackQ& cbQ, uint32_t numThreads)
            : _asyncSpinner(numThreads, std::addressof(cbQ))
    {
    }

    void start() { _asyncSpinner.start(); }

    void stop() { _asyncSpinner.stop(); }

private:
    ros::AsyncSpinner _asyncSpinner;
};

template <class Timer, class CallbackQ, class ThreadPool>
class AlicaTimerFactory
{
    using TimerCb = typename Timer::TimerCb;

public:
    AlicaTimerFactory(uint32_t numThreads)
            : _cbQ()
            , _threadPool(_cbQ, numThreads)
    {
        _threadPool.start();
    }

    AlicaTimerFactory(const AlicaTimerFactory&) = delete;
    AlicaTimerFactory(AlicaTimerFactory&&) = delete;
    AlicaTimerFactory& operator=(const AlicaTimerFactory&) = delete;
    AlicaTimerFactory&& operator=(AlicaTimerFactory&&) = delete;
    ~AlicaTimerFactory() = default;

    Timer createTimer(TimerCb timerCb, AlicaTime period) { return Timer(_cbQ, std::move(timerCb), period); }

private:
    // TODO: pass shared_ptr's to the CallbackQ to the timers & threadpool
    CallbackQ _cbQ;
    ThreadPool _threadPool;
};

using TimerFactoryRos = AlicaTimerFactory<SyncStopTimerRos<ros::CallbackQueue>, ros::CallbackQueue, ThreadPoolRos<ros::CallbackQueue>>;
} // namespace alica