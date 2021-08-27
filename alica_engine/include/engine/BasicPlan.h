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

    void doInit();
    void doRun(void* msg);
    void doTerminate();

    void start(RunningPlan* rp);
    void stop();
    void setEngine(AlicaEngine* engine) { _ae = engine; }

    ThreadSafePlanInterface getPlanContext() const;
    void setRunningPlan(RunningPlan* rp) { _context = rp; }

    bool isPlanStarted() const { return _planStarted; }

    void setConfiguration(const Configuration* conf);

    AlicaTime getInterval() { return _msInterval; }
    void setInterval(int32_t msInterval);

private:
    using PlanState = uint64_t;

    virtual void onInit(){};
    virtual void run(void* msg){};
    virtual void onTerminate(){};
    void sendLogMessage(int level, const std::string& message) const;

    static constexpr bool isActive(PlanState planState) { return !(planState & 1); }
    bool isExecutingInContext() const
    {
        PlanState signalState = _signalState.load(), execState = _execState.load();
        return signalState == execState && isActive(signalState);
    }

    static constexpr int DEFAULT_MS_INTERVAL = 100;

    alica::AlicaEngine* _ae;
    const Configuration* _configuration;
    AlicaTime _msInterval;
    std::atomic<bool> _planStarted;
    int64_t _activeRunJobId;

    std::atomic<RunningPlan*> _context;
    std::atomic<PlanState> _signalState;
    std::atomic<PlanState> _execState;
};
} // namespace alica
