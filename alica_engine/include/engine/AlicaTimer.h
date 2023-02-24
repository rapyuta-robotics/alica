#pragma once

#include "engine/AlicaClock.h"
#include "engine/IAlicaTimer.h"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <stdexcept>
#include <thread>

namespace alica
{

class AlicaSystemTimerImpl
{
public:
    using TimerCb = std::function<void()>;

    AlicaSystemTimerImpl(TimerCb&& userCb, alica::AlicaTime period)
            : _userCb(std::move(userCb))
            , _period(period.inMilliseconds())
            , _isActive(false)
    {
    }

    ~AlicaSystemTimerImpl()
    {
        if (_isActive) {
            stop();
        }
    }

    void start()
    {

        if (_isActive) {
            stop();
        }
        _isActive = true;
        _thread = std::thread([this]() {
            while (_isActive) {
                _userCb();
                int64_t sleep_duration = _period;
                while ((sleep_duration > 0) && _isActive) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(std::min(int64_t(500), _period)));
                    sleep_duration -= std::min(int64_t(500), _period);
                }
            }
        });
    }

    void stop()
    {
        _isActive = false;
        if (_thread.joinable()) {
            _thread.join();
        }
    }

    TimerCb _userCb;
    int64_t _period;
    std::thread _thread;
    std::atomic<bool> _isActive;
};

class AlicaSystemTimer : public alica::IAlicaTimer
{
    using Impl = AlicaSystemTimerImpl;

public:
    using TimerCb = std::function<void()>;

    AlicaSystemTimer(TimerCb&& userCb, alica::AlicaTime period)
            : _impl(std::make_shared<Impl>(std::move(userCb), period))
    {
        _impl->start();
    }

    ~AlicaSystemTimer() { _impl->stop(); }

private:
    std::shared_ptr<Impl> _impl;
};

template <class Timer>
class AlicaTimerFactory : public alica::IAlicaTimerFactory
{
    using TimerCb = typename Timer::TimerCb;

public:
    using TimerType = Timer;

    AlicaTimerFactory() {}

    AlicaTimerFactory(const AlicaTimerFactory&) = delete;
    AlicaTimerFactory& operator=(const AlicaTimerFactory&) = delete;
    ~AlicaTimerFactory() = default;

    std::unique_ptr<alica::IAlicaTimer> createTimer(TimerCb timerCb, alica::AlicaTime period) const override
    {
        if (!timerCb) {
            throw std::invalid_argument("no timerCb specified!");
        }
        return std::make_unique<TimerType>(std::move(timerCb), period);
    }

private:
};

using AlicaSystemTimerFactory = AlicaTimerFactory<AlicaSystemTimer>;

} // namespace alica
