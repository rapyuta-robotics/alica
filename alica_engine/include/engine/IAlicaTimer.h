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
     * Starts the AlicaTimer.
     * The implementation of start should start the repeated execution of the TimerCb.
     */
    virtual void start() = 0;
    /**
     * Stops the AlicaTimer.
     * The implementation of stop should stop the repeated execution of the TimerCb.
     * Stopping the timer is expected to be synchronized. After leaving the stop method,
     * the timer should no longer execute any TimerCb.
     */
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