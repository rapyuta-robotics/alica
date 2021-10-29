#pragma once

#include "engine/AlicaClock.h"
#include "engine/IAlicaTrace.h"
#include <string>
#include <atomic>
#include <functional>

namespace alica
{

class RunningPlan;
class Configuration;
class AlicaEngine;
class ThreadSafePlanInterface;

// For comments, have a look at BasicBehaviour. This class essentially mimics its behaviour.
// TODO: This indicates that these classes should probably be combined
class BasicPlan
{
public:
    BasicPlan();
    virtual ~BasicPlan() = default;

    void start(RunningPlan* rp);
    void stop();

    void setEngine(AlicaEngine* engine) { _ae = engine; }
    void setConfiguration(const Configuration* conf) { _configuration = conf; }
    // TODO: get the name in the constructor
    void setName(const std::string& name) { _name = name; }
    void setAsMasterPlan() { _isMasterPlan = true; };

    AlicaTime getInterval() const { return _msInterval; }
    void setInterval(int32_t msInterval) { _msInterval = AlicaTime::milliseconds(msInterval); }

    std::optional<std::string> getTraceContext() const
    {
        return _trace ? std::optional<std::string>(_trace->context()) : std::nullopt;
    }

protected:
    ThreadSafePlanInterface getPlanContext() const;

    virtual void onInit(){};
    virtual void run(void* msg){};
    virtual void onTerminate(){};

    enum class TracingType : uint8_t
    {
        DEFAULT,
        SKIP,
        ROOT,
        CUSTOM
    };

    void setTracing(TracingType type, std::function<std::string(BasicPlan*)> customTraceContextGetter = {})
    {
        _tracingType = type;
        _customTraceContextGetter = std::move(customTraceContextGetter);
    }

    IAlicaTrace* getTrace() const
    {
        return _trace ? _trace.get() : nullptr;
    }

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

    void startTrace();

    static constexpr int DEFAULT_MS_INTERVAL = 100;

    alica::AlicaEngine* _ae;
    const Configuration* _configuration;
    AlicaTime _msInterval;
    int64_t _activeRunJobId;
    std::string _name;
    bool _isMasterPlan;

    std::atomic<RunningPlan*> _signalContext;
    std::atomic<RunningPlan*> _execContext;
    std::atomic<Counter> _signalState;
    std::atomic<Counter> _execState;

    TracingType _tracingType;
    std::function<std::string(BasicPlan*)> _customTraceContextGetter;
    std::unique_ptr<IAlicaTrace> _trace;
    bool _runTraced;

    bool _initExecuted;
};
} // namespace alica
