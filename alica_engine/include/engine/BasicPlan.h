#pragma once

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
    virtual void onTermination(){};

    void doInit();
    void doTerminate();

    void start();
    void stop();
    void setEngine(AlicaEngine* engine) { _ae = engine; }

    ThreadSafePlanInterface getPlanContext() const;
    void setRunningPlan(RunningPlan* rp) { _context = rp; }

    bool isPlanStarted() const { return _planStarted; }

    void setConfiguration(const Configuration* conf);

private:
    alica::AlicaEngine* _ae;
    RunningPlan* _context;
    std::atomic<bool> _planStarted;
    Configuration* _configuration;
};
} // namespace alica
