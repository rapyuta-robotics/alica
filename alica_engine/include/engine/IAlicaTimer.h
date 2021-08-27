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
     * Timer is expected to stop on destruction.
     * The implementation of stop should stop the repeated execution of the TimerCb.
     * Stopping the timer is expected to be synchronized. After leaving the stop method,
     * the timer should no longer execute any TimerCb. If TimerCb is already invoked stop
     * waits until TimerCb returns, therefore destruction is blocking.
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