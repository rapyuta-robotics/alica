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

    /**
     * Timer is expected to start on construction & TimerCb should be fired immediately
     * after this start.
     */

    /**
     * The destructor should stop the repeated execution of the TimerCb and needs to block if any
     * TimerCb is already in progress.
     */
    virtual ~IAlicaTimer() = default;
};

class IAlicaTimerFactory
{
public:
    virtual std::unique_ptr<IAlicaTimer> createTimer(IAlicaTimer::TimerCb timerCb, AlicaTime period) = 0;
    virtual ~IAlicaTimerFactory() = default;
};

} // namespace alica