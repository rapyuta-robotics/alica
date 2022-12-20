#pragma once

#include "engine/AlicaClock.h"
#include "engine/IAlicaTimer.h"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <stdexcept>
#include <thread>

namespace alicaTimer
{

class SyncStopTimerImpl : public std::enable_shared_from_this<SyncStopTimerImpl>
{
    using Base = std::enable_shared_from_this<SyncStopTimerImpl>;

public:
    using TimerCb = std::function<void()>;

    SyncStopTimerImpl(TimerCb&& userCb, alica::AlicaTime period)
            : _userCb(std::move(userCb))
            , _period(period.inMilliseconds())
            , _active(false)
    {
    }

    ~SyncStopTimerImpl()
    {
        if (_active) {
            stop();
        }
    }

    void start()
    {

        if (_active) {
            stop();
        }
        _active = true;
        _thread = std::thread([this]() {
            while (_active) {
                _userCb();
                int64_t sleep_duration = _period;
                while ((sleep_duration > 0) && _active) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(std::min(int64_t(500), _period)));
                    sleep_duration -= std::min(int64_t(500), _period);
                }
            }
        });
    }

    void stop()
    {
        _active = false;
        if (_thread.joinable()) {
            _thread.join();
        }
    }

    TimerCb _userCb;
    int64_t _period;
    std::thread _thread;
    std::atomic<bool> _active;
};

class SyncStopTimerTest : public alica::IAlicaTimer
{
    using Impl = SyncStopTimerImpl;

public:
    using TimerCb = std::function<void()>;

    SyncStopTimerTest(TimerCb&& userCb, alica::AlicaTime period)
            : _impl(std::make_shared<Impl>(std::move(userCb), period))
    {
        _impl->start();
    }

    ~SyncStopTimerTest() { _impl->stop(); }

private:
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

using AlicaTestTimerFactory = TimerFactory<SyncStopTimerTest>;

} // namespace alicaTimer
