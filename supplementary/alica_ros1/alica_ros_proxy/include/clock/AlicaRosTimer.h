#pragma once

#include "engine/AlicaClock.h"
#include "engine/IAlicaTimer.h"

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <ros/ros.h>
#include <stdexcept>

namespace alicaRosTimer
{

template <class CallbackQ>
class SyncStopTimerRosImpl : public std::enable_shared_from_this<SyncStopTimerRosImpl<CallbackQ>>
{
    using Base = std::enable_shared_from_this<SyncStopTimerRosImpl<CallbackQ>>;

public:
    using TimerCb = std::function<void()>;

    SyncStopTimerRosImpl(CallbackQ& cbQ, TimerCb&& userCb, const alica::AlicaTime& period)
            : _userCb(std::move(userCb))
            , _period(toRosDuration(period))
            , _nh()
            , _timer()
            , _stopMutex()
            , _stopCv()
            , _stop(false)
            , _userCbInProgress(false)
    {
        _nh.setCallbackQueue(std::addressof(cbQ));
    }

    void start()
    {
        // Call the userCb once immediately. Required since ROS timers don't fire immediately. This means the first callback
        // will be called from the scheduler thread which is fine since stop() anyway blocks on userCb()
        _userCb();

        // Grab a weak ptr to this object (grabbing a shared_ptr will result in a cycle)
        _timer = _nh.createTimer(_period, [weak_ptr_self = Base::weak_from_this()](auto&&...) {
            // Ensure impl object for this timer is not destroyed
            if (auto self = weak_ptr_self.lock()) {
                self->timerCb();
            }
        });
    }

    void stop()
    {
        _timer.stop();
        // Block if the timerCb is in progress
        // TODO: use atomics to check if timerCb is in progress & acquire the mutex only if we need to block
        std::unique_lock<std::mutex> lock(_stopMutex);
        _stop = true;
        _stopCv.wait(lock, [this]() { return !_userCbInProgress; });
    }

    void timerCb()
    {
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
            lock.unlock();
            _stopCv.notify_all();
        }
    }

    static ros::Duration toRosDuration(const alica::AlicaTime& period)
    {
        auto sec = period.inSeconds();
        auto nanosec = period.inNanoseconds();
        nanosec -= sec * 1000000000LL;
        return ros::Duration(sec, nanosec);
    }

    TimerCb _userCb;
    ros::Duration _period;
    ros::NodeHandle _nh;
    ros::Timer _timer;
    std::mutex _stopMutex;
    std::condition_variable _stopCv;
    bool _stop;
    bool _userCbInProgress;
};

template <class CallbackQ>
class SyncStopTimerRos : public alica::IAlicaTimer
{
    using Impl = SyncStopTimerRosImpl<CallbackQ>;

public:
    using TimerCb = std::function<void()>;

    SyncStopTimerRos(CallbackQ& cbQ, TimerCb&& userCb, alica::AlicaTime period)
            : _impl(std::make_shared<Impl>(cbQ, std::move(userCb), period))
    {
        _impl->start();
    }

    ~SyncStopTimerRos() { _impl->stop(); }

private:
    // ROS timer's can be destroyed while the timerCb is still in progress so we use shared & weak ptr to
    // ensure this does not happen
    std::shared_ptr<Impl> _impl;
};

template <template <class> class Timer, class CallbackQ>
class TimerFactory : public alica::IAlicaTimerFactory
{
    using TimerCb = typename Timer<CallbackQ>::TimerCb;

public:
    using TimerType = Timer<CallbackQ>;

    TimerFactory(CallbackQ& callback_queue = *ros::getGlobalCallbackQueue())
            : _cbQ(callback_queue)
    {
    }

    TimerFactory(const TimerFactory&) = delete;
    TimerFactory(TimerFactory&&) = delete;
    TimerFactory& operator=(const TimerFactory&) = delete;
    TimerFactory&& operator=(TimerFactory&&) = delete;
    ~TimerFactory() = default;

    std::unique_ptr<alica::IAlicaTimer> createTimer(TimerCb timerCb, alica::AlicaTime period) const override
    {
        if (!timerCb) {
            throw std::invalid_argument("no timerCb specified!");
        }
        return std::make_unique<TimerType>(_cbQ, std::move(timerCb), period);
    }

private:
    CallbackQ& _cbQ;
};

using AlicaRosTimerFactory = TimerFactory<SyncStopTimerRos, ros::CallbackQueue>;

} // namespace alicaRosTimer
