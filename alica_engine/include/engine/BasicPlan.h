#pragma once

#include "engine/AlicaClock.h"
#include "engine/RunnableObject.h"
#include <atomic>
#include <string>

namespace alica
{

class RunningPlan;
class ThreadSafePlanInterface;

class BasicPlan : public RunnableObject
{
public:
    BasicPlan();
    virtual ~BasicPlan() = default;

    void start(RunningPlan* rp);
    void stop();

    // TODO: get the name in the constructor
    void setName(const std::string& name) { _name = name; }

    AlicaTime getInterval() const { return _msInterval; }
    void setInterval(int32_t msInterval) { _msInterval = AlicaTime::milliseconds(msInterval); }

protected:
    ThreadSafePlanInterface getPlanContext() const;

    virtual void onInit(){};
    virtual void run(void* msg){};
    virtual void onTerminate(){};

    void disableTracing() { clearFlags(Flags::TRACING_ENABLED); };

private:
    using Counter = uint64_t;

    void doInit();
    void doRun(void* msg);
    void doTerminate();

    void sendLogMessage(int level, const std::string& message) const;

    static constexpr bool isActive(Counter cnt) { return !(cnt & 1); }
    bool isExecutingInContext() const
    {
        Counter sc = _signalState.load(), ec = _execState.load();
        return sc == ec && isActive(sc);
    }

    static constexpr int DEFAULT_MS_INTERVAL = 100;

    AlicaTime _msInterval;
    int64_t _activeRunJobId;
    std::string _name;

    std::atomic<RunningPlan*> _signalContext;
    std::atomic<RunningPlan*> _execContext;
    std::atomic<Counter> _signalState;
    std::atomic<Counter> _execState;
};
} // namespace alica
