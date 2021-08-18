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

    virtual void init(){};
    virtual void run(void* msg){};
    virtual void onTermination(){};

    void doInit();
    void doRun(void* msg);
    void doTerminate();

    void start();
    void stop();
    void setEngine(AlicaEngine* engine) { _ae = engine; }

    ThreadSafePlanInterface getPlanContext() const;
    void setRunningPlan(RunningPlan* rp) { _context = rp; }

    bool isPlanStarted() const { return _planStarted; }

    void setConfiguration(const Configuration* conf);

    AlicaTime getInterval() { return _msInterval; }

private:
    static constexpr int DEFAULT_MS_INTERVAL = 100;

    alica::AlicaEngine* _ae;
    const Configuration* _configuration;
    AlicaTime _msInterval;
    RunningPlan* _context;
    std::atomic<bool> _planStarted;
    int64_t _activeRunJobId;
};
} // namespace alica
