#pragma once

#include "engine/AlicaClock.h"
#include "engine/IAlicaTimer.h"

#include <atomic>
#include <functional>
#include <memory>
#include <stdexcept>

namespace alica
{

class AlicaSystemTimer : public alica::IAlicaTimer
{
public:
    using TimerCb = std::function<void()>;

    AlicaSystemTimer(TimerCb&& userCb, alica::AlicaTime period);
    ~AlicaSystemTimer();

private:
    class AlicaSystemTimerImpl;
    std::unique_ptr<AlicaSystemTimerImpl> _impl;
};

template <class Timer>
class AlicaTimerFactory : public alica::IAlicaTimerFactory
{
    using TimerCb = typename Timer::TimerCb;

public:
    AlicaTimerFactory() = default;

    AlicaTimerFactory(const AlicaTimerFactory&) = delete;
    AlicaTimerFactory& operator=(const AlicaTimerFactory&) = delete;
    ~AlicaTimerFactory() = default;

    std::unique_ptr<alica::IAlicaTimer> createTimer(TimerCb timerCb, alica::AlicaTime period) const override;

private:
};

using AlicaSystemTimerFactory = AlicaTimerFactory<AlicaSystemTimer>;

template <class TimerType>
std::unique_ptr<alica::IAlicaTimer> AlicaTimerFactory<TimerType>::createTimer(TimerCb timerCb, alica::AlicaTime period) const
{
    if (!timerCb) {
        throw std::invalid_argument("no timerCb specified!");
    }
    return std::make_unique<TimerType>(std::move(timerCb), period);
}

} // namespace alica
