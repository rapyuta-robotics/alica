#pragma once

#include "engine/AlicaClock.h"

#include <functional>
#include <memory>

namespace alica
{

class IAlicaTimer
{
public:
    using TimerCb = std::function<void()>;

    virtual void start() = 0;
    virtual void stop() = 0;
    virtual ~IAlicaTimer() = default;
};

class IAlicaTimerFactory
{
public:
    virtual std::unique_ptr<IAlicaTimer> createTimer(IAlicaTimer::TimerCb timerCb, AlicaTime period) = 0;
    virtual ~IAlicaTimerFactory() = default;
};

} // namespace alica