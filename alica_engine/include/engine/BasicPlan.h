#pragma once

#include "engine/PlanInterface.h"

#include <string>
#include <atomic>

namespace alica
{

class RunningPlan;
class Configuration;
class AlicaEngine;

class BasicPlan
{
public:
    BasicPlan();
    virtual ~BasicPlan() = default;

    virtual void init(){};
    virtual void onTermination(){};

    void doInit()
    {
        init();
        _planStarted = true;
    }

    void doTerminate()
    {
        onTermination();
        _planStarted = false;
    }

    void start();
    void stop();
    void setEngine(AlicaEngine* engine) { _ae = engine; }

    ThreadSafePlanInterface getPlanContext() { return ThreadSafePlanInterface(isPlanStarted() ? _context : nullptr); }
    void setRunningPlan(RunningPlan* rp) { _context = rp; }

    bool isPlanStarted() const { return _planStarted; }

    void setConfiguration(const Configuration* conf);

private:
    alica::AlicaEngine* _ae;
    RunningPlan* _context;
    std::atomic<bool> _planStarted;
    const Configuration* _configuration;
};
} // namespace alica
