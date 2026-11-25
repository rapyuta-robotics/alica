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
    virtual std::unique_ptr<IAlicaTimer> createTimer(IAlicaTimer::TimerCb timerCb, AlicaTime period) const = 0;

    /**
     * True if every timer this factory creates runs its callback on a thread that only that timer uses,
     * so that blocking inside the callback delays nobody else. The engine waits for fast-path events
     * inside its timer callback only when its factory returns true. The default keeps the behaviour
     * of factories written before this function existed.
     */
    virtual bool dedicatedThread() const { return true; }

    virtual ~IAlicaTimerFactory() = default;
};

} // namespace alica
