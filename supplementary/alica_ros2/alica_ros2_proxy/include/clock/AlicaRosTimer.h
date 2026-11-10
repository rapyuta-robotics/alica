#pragma once

#include "engine/AlicaClock.h"
#include "engine/IAlicaTimer.h"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/create_timer.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

#include <unistd.h>

namespace alicaRosTimer
{

class SyncStopTimerRosImpl : public std::enable_shared_from_this<SyncStopTimerRosImpl>
{
    using Base = std::enable_shared_from_this<SyncStopTimerRosImpl>;

public:
    using TimerCb = std::function<void()>;

    SyncStopTimerRosImpl(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr callbackGroup, TimerCb&& userCb, const alica::AlicaTime& period)
            : _userCb(std::move(userCb))
            , _period(toRosDuration(period))
            , _node(std::move(node))
            , _callbackGroup(std::move(callbackGroup))
            , _timer()
            , _stopMutex()
            , _stopCv()
            , _stop(false)
            , _userCbInProgress(false)
    {
        if (!_callbackGroup) {
            // A mutually exclusive group for each timer: the callbacks of one timer never overlap, as with a ROS1 timer,
            // but the callbacks of different timers can run in parallel on a multi threaded executor
            _callbackGroup = _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        }
    }

    void start()
    {
        // Call the userCb once immediately. Required since ROS timers don't fire immediately. This means the first callback
        // will be called from the scheduler thread which is fine since stop() anyway blocks on userCb()
        _userCb();

        // Grab a weak ptr to this object (grabbing a shared_ptr will result in a cycle)
        _timer = rclcpp::create_timer(
                _node, _node->get_clock(), _period,
                [weak_ptr_self = Base::weak_from_this()]() {
                    // Ensure impl object for this timer is not destroyed
                    if (auto self = weak_ptr_self.lock()) {
                        self->timerCb();
                    }
                },
                _callbackGroup);
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
            lock.unlock();
            _stopCv.notify_all();
        }
    }

    static rclcpp::Duration toRosDuration(const alica::AlicaTime& period)
    {
        auto sec = period.inSeconds();
        auto nanosec = period.inNanoseconds();
        nanosec -= sec * 1000000000LL;
        return rclcpp::Duration(sec, nanosec);
    }

    TimerCb _userCb;
    rclcpp::Duration _period;
    rclcpp::Node::SharedPtr _node;
    rclcpp::CallbackGroup::SharedPtr _callbackGroup;
    rclcpp::TimerBase::SharedPtr _timer;
    std::mutex _stopMutex;
    std::condition_variable _stopCv;
    bool _stop;
    bool _userCbInProgress;
};

class SyncStopTimerRos : public alica::IAlicaTimer
{
    using Impl = SyncStopTimerRosImpl;

public:
    using TimerCb = std::function<void()>;

    SyncStopTimerRos(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr callbackGroup, TimerCb&& userCb, alica::AlicaTime period)
            : _impl(std::make_shared<Impl>(std::move(node), std::move(callbackGroup), std::move(userCb), period))
    {
        _impl->start();
    }

    ~SyncStopTimerRos() { _impl->stop(); }

private:
    // ROS timer's can be destroyed while the timerCb is still in progress so we use shared & weak ptr to
    // ensure this does not happen
    std::shared_ptr<Impl> _impl;
};

/**
 * Creates timers on one node.
 *
 * If a node is given, the caller owns the executor that spins it. Use a multi threaded executor, so that a long timer
 * callback does not block the others. If a callback group is given, all timers use it; else each timer gets its own
 * mutually exclusive group. If no node is given, the factory creates one node and spins it on an internal multi
 * threaded executor.
 */
template <class Timer>
class TimerFactory : public alica::IAlicaTimerFactory
{
    using TimerCb = typename Timer::TimerCb;

public:
    using TimerType = Timer;

    TimerFactory(rclcpp::Node::SharedPtr node = nullptr, rclcpp::CallbackGroup::SharedPtr callbackGroup = nullptr)
            : _node(std::move(node))
            , _callbackGroup(std::move(callbackGroup))
    {
        if (!_node) {
            static std::atomic<int> instanceCount(0);
            _node = rclcpp::Node::make_shared("alica_timer_" + std::to_string(getpid()) + "_" + std::to_string(instanceCount++));
            _executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
            _executor->add_node(_node);
            _spinThread = std::thread([this]() { _executor->spin(); });
        }
    }

    TimerFactory(const TimerFactory&) = delete;
    TimerFactory(TimerFactory&&) = delete;
    TimerFactory& operator=(const TimerFactory&) = delete;
    TimerFactory&& operator=(TimerFactory&&) = delete;

    ~TimerFactory()
    {
        if (_spinThread.joinable()) {
            _executor->cancel();
            _spinThread.join();
        }
    }

    std::unique_ptr<alica::IAlicaTimer> createTimer(TimerCb timerCb, alica::AlicaTime period) const override
    {
        if (!timerCb) {
            throw std::invalid_argument("no timerCb specified!");
        }
        return std::make_unique<TimerType>(_node, _callbackGroup, std::move(timerCb), period);
    }

private:
    rclcpp::Node::SharedPtr _node;
    rclcpp::CallbackGroup::SharedPtr _callbackGroup;
    // Only used if no node is given
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> _executor;
    std::thread _spinThread;
};

using AlicaRosTimerFactory = TimerFactory<SyncStopTimerRos>;

} // namespace alicaRosTimer
