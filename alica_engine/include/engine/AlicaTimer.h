#pragma once

#include "engine/AlicaClock.h"

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <ros/ros.h>

namespace alica
{

template <class CallbackQ>
class SyncStopTimerRos
{
public:
    using TimerCb = std::function<void()>;

    SyncStopTimerRos(std::shared_ptr<CallbackQ> cbQ, TimerCb&& userCb, AlicaTime period)
            : _userCb(std::move(userCb))
            , _nh()
            , _timer()
            , _cbQ(std::move(cbQ))
            , _stopMutex()
            , _stopCv()
            , _userCbInProgress(false)
            , _stop(false)
            , _period(period)
    {
        _nh.setCallbackQueue(_cbQ.get());
        initTimer();
    }

    ~SyncStopTimerRos() { stop(); }
    void start() { _timer.start(); }

    void stop()
    {
        // TODO: get rid of mutex lock for checking if userCb is in progress
        _timer.stop();
        std::unique_lock<std::mutex> lock(_stopMutex);
        _stop = true;
        if (!_userCbInProgress) {
            return;
        }
        _stopCv.wait(lock, [this]() { return !_userCbInProgress; });
    }

private:
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
    std::shared_ptr<CallbackQ> _cbQ;
    std::mutex _stopMutex;
    std::condition_variable _stopCv;
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
    ThreadPoolRos(std::shared_ptr<CallbackQ> cbQ, uint32_t numThreads)
            : _asyncSpinner(numThreads, cbQ.get())
            , _cbQ(std::move(cbQ))
    {
    }

    void start() { _asyncSpinner.start(); }
    void stop() { _asyncSpinner.stop(); }

private:
    ros::AsyncSpinner _asyncSpinner;
    std::shared_ptr<CallbackQ> _cbQ;
};

template <template <class> class Timer, class CallbackQ, template <class> class ThreadPool>
class AlicaTimerFactory
{
    using TimerCb = typename Timer<CallbackQ>::TimerCb;

public:
    using TimerType = Timer<CallbackQ>;

    AlicaTimerFactory(uint32_t numThreads)
            : _cbQ(std::make_shared<CallbackQ>())
            , _threadPool(_cbQ, numThreads)
    {
        _threadPool.start();
    }

    AlicaTimerFactory(const AlicaTimerFactory&) = delete;
    AlicaTimerFactory(AlicaTimerFactory&&) = delete;
    AlicaTimerFactory& operator=(const AlicaTimerFactory&) = delete;
    AlicaTimerFactory&& operator=(AlicaTimerFactory&&) = delete;
    ~AlicaTimerFactory() = default;

    std::unique_ptr<TimerType> createTimer(TimerCb timerCb, AlicaTime period) { return std::make_unique<TimerType>(_cbQ, std::move(timerCb), period); }

private:
    std::shared_ptr<CallbackQ> _cbQ;
    ThreadPool<CallbackQ> _threadPool;
};

using TimerFactoryRos = AlicaTimerFactory<SyncStopTimerRos, ros::CallbackQueue, ThreadPoolRos>;

} // namespace alica