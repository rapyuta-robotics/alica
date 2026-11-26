#pragma once

#include "engine/AlicaClock.h"
#include "engine/IAlicaTimer.h"

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

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

    // Runs the timer on a node, executor and thread of its own
    SyncStopTimerRosImpl(TimerCb&& userCb, alica::AlicaTime period)
            : SyncStopTimerRosImpl(rclcpp::Node::make_shared("AlicaROS2Timer"), nullptr, std::move(userCb), period)
    {
        _myexec = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
        _myexec->add_node(_nh);
        _clock = std::thread([this]() { _myexec->spin(); });
    }

    // Runs the timer on node, in group (nullptr: the node's default callback group). The executor that serves group runs the callbacks.
    SyncStopTimerRosImpl(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, TimerCb&& userCb, alica::AlicaTime period)
            : _userCb(std::move(userCb))
            , _period(toRosDuration(period))
            , _nh(std::move(node))
            , _group(std::move(group))
            , _timer()
            , _stopMutex()
            , _stopCv()
            , _stop(false)
            , _userCbInProgress(false)
    {
    }

    ~SyncStopTimerRosImpl()
    {
        if (_myexec) {
            _myexec->cancel();
            _clock.join();
        }
    }

    void start()
    {
        // Call the userCb once immediately. Required since ROS timers don't fire immediately. This means the first callback
        // will be called from the scheduler thread which is fine since stop() anyway blocks on userCb()
        _userCb();

        if (_myexec) {
            // The destructor joins the thread that runs this callback, so the callback cannot outlive this object
            _timer = rclcpp::create_timer(
                    _nh, _nh->get_clock(), _period, [this]() { timerCb(); }, _group);
        } else {
            // The caller's executor can still be inside timerCb() when the last owner releases this object, so hold it through a weak ptr
            _timer = rclcpp::create_timer(
                    _nh, _nh->get_clock(), _period,
                    [weakSelf = Base::weak_from_this()]() {
                        if (auto self = weakSelf.lock()) {
                            self->timerCb();
                        }
                    },
                    _group);
        }
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
    rclcpp::CallbackGroup::SharedPtr _group;
    std::shared_ptr<rclcpp::TimerBase> _timer;
    std::mutex _stopMutex;
    std::condition_variable _stopCv;
    bool _stop;
    bool _userCbInProgress;
    std::thread _clock;
    // Only set when the timer runs on an executor of its own
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> _myexec;
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

    SyncStopTimerRos(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, TimerCb&& userCb, alica::AlicaTime period)
            : _impl(std::make_shared<Impl>(std::move(node), std::move(group), std::move(userCb), period))
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

    // Each timer runs on a node, executor and thread of its own
    TimerFactory() {}

    /**
     * The timers run on node, in group (nullptr: the node's default callback group), so the executor that serves group runs
     * the timer callbacks.
     * dedicatedThread: pass true only if a single-threaded executor serves group and nothing else, so that blocking inside a
     * timer callback delays no other callback. The default, false, fits the usual setup: the node's executor, or an executor
     * that the application's main loop spins.
     */
    TimerFactory(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group = nullptr, bool dedicatedThread = false)
            : _node(std::move(node))
            , _group(std::move(group))
            , _dedicatedThread(dedicatedThread)
    {
        if (!_node) {
            throw std::invalid_argument("no node specified!");
        }
    }

    TimerFactory(const TimerFactory&) = delete;
    TimerFactory& operator=(const TimerFactory&) = delete;
    ~TimerFactory() = default;

    std::unique_ptr<alica::IAlicaTimer> createTimer(TimerCb timerCb, alica::AlicaTime period) const override
    {
        if (!timerCb) {
            throw std::invalid_argument("no timerCb specified!");
        }
        if (_node) {
            return std::make_unique<TimerType>(_node, _group, std::move(timerCb), period);
        }
        return std::make_unique<TimerType>(std::move(timerCb), period);
    }

    bool dedicatedThread() const override { return _dedicatedThread; }

private:
    rclcpp::Node::SharedPtr _node;
    rclcpp::CallbackGroup::SharedPtr _group;
    // True without a node: every timer then has a thread of its own
    bool _dedicatedThread{true};
};

using AlicaRosTimerFactory = TimerFactory<SyncStopTimerRos>;

} // namespace alicaRosTimer
