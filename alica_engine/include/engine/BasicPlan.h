#pragma once

#include "engine/AlicaClock.h"
#include <string>
#include <atomic>

namespace alica
{

class RunningPlan;
class Configuration;
class AlicaEngine;
class ThreadSafePlanInterface;

class BasicPlan
{
public:
    BasicPlan();
    virtual ~BasicPlan() = default;

    void start(RunningPlan* rp);
    void stop();

    void setEngine(AlicaEngine* engine) { _ae = engine; }
    void setRunningPlan(RunningPlan* rp) { _context = rp; }
    void setConfiguration(const Configuration* conf) { _configuration = conf; }

    AlicaTime getInterval() { return _msInterval; }
    void setInterval(int32_t msInterval) { _msInterval = AlicaTime::milliseconds(msInterval); }

protected:
    ThreadSafePlanInterface getPlanContext() const;

    virtual void onInit(){};
    virtual void run(void* msg){};
    virtual void onTerminate(){};

private:
    using Counter = uint64_t;

    void doInit();
    void doRun(void* msg);
    void doTerminate();

    void sendLogMessage(int level, const std::string& message) const;

    // See BasicBehaviour.h for explanation of the logic used
    static constexpr bool isActive(Counter cnt) { return !(cnt & 1); }
    bool isExecutingInContext() const
    {
        Counter sc = _signalState.load(), ec = _execState.load();
        return sc == ec && isActive(sc);
    }

    static constexpr int DEFAULT_MS_INTERVAL = 100;

    alica::AlicaEngine* _ae;
    const Configuration* _configuration;
    AlicaTime _msInterval;
    int64_t _activeRunJobId;

    std::atomic<RunningPlan*> _context;
    std::atomic<Counter> _signalState;
    std::atomic<Counter> _execState;
};
} // namespace alica
