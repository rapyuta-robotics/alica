#pragma once

#include "engine/AlicaClock.h"
#include "engine/IAlicaTimer.h"

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>

#include <rclcpp/duration.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

namespace alicaRosTimer
{

class SyncStopTimerRosImpl : public std::enable_shared_from_this<SyncStopTimerRosImpl>
{
    using Base = std::enable_shared_from_this<SyncStopTimerRosImpl>;

public:
    using TimerCb = std::function<void()>;

    SyncStopTimerRosImpl(TimerCb&& userCb, alica::AlicaTime period)
            : _userCb(std::move(userCb))
            , _period(toRosDuration(period))
            , _nh(rclcpp::Node::make_shared("AlicaROS2Timer"))
            , _timer()
            , _stopMutex()
            , _stopCv()
            , _userCbInProgress(false)
            , _stop(false)
    {

        _myexec.add_node(_nh);
        _clock = std::thread([this]() { _myexec.spin(); });
    }

    ~SyncStopTimerRosImpl()
    {
        _myexec.cancel();
        _clock.join();
    }

    void start()
    {
        // Call the userCb once immediately. Required since ROS timers don't fire immediately. This means the first callback
        // will be called from the scheduler thread which is fine since stop() anyway blocks on userCb()
        _userCb();

        _timer = rclcpp::create_timer(_nh, _nh->get_clock(), _period, std::bind(&SyncStopTimerRosImpl::timerCb, this));
    }

    void stop()
    {
        _timer->cancel();
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
            _stopCv.notify_one();
        }
    }

    static rclcpp::Duration toRosDuration(alica::AlicaTime period)
    {
        auto sec = period.inSeconds();
        auto nanosec = period.inNanoseconds();
        nanosec -= sec * 1000000000LL;
        return rclcpp::Duration(sec, nanosec);
    }

    TimerCb _userCb;
    rclcpp::Duration _period;
    rclcpp::Node::SharedPtr _nh;
    std::shared_ptr<rclcpp::TimerBase> _timer;
    std::mutex _stopMutex;
    std::condition_variable _stopCv;
    bool _stop;
    bool _userCbInProgress;
    std::thread _clock;
    rclcpp::executors::MultiThreadedExecutor _myexec;
};

class SyncStopTimerRos : public alica::IAlicaTimer
{
    using Impl = SyncStopTimerRosImpl;

public:
    using TimerCb = std::function<void()>;

    SyncStopTimerRos(TimerCb&& userCb, alica::AlicaTime period)
            : _impl(std::make_shared<Impl>(std::move(userCb), period))
    {
        _impl->start();
    }

    ~SyncStopTimerRos() { _impl->stop(); }

private:
    // ROS timer's can be destroyed while the timerCb is still in progress so we use shared & weak ptr to
    // ensure this does not happen
    std::shared_ptr<Impl> _impl;
};

template <class Timer>
class TimerFactory : public alica::IAlicaTimerFactory
{
    using TimerCb = typename Timer::TimerCb;

public:
    using TimerType = Timer;

    TimerFactory() {}

    TimerFactory(const TimerFactory&) = delete;
    TimerFactory& operator=(const TimerFactory&) = delete;
    ~TimerFactory() = default;

    std::unique_ptr<alica::IAlicaTimer> createTimer(TimerCb timerCb, alica::AlicaTime period) const override
    {
        if (!timerCb) {
            throw std::invalid_argument("no timerCb specified!");
        }
        return std::make_unique<TimerType>(std::move(timerCb), period);
    }

private:
};

using AlicaRosTimerFactory = TimerFactory<SyncStopTimerRos>;

} // namespace alicaRosTimer
