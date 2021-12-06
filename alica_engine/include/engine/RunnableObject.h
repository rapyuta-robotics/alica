#pragma once

#include "engine/AlicaClock.h"
#include "engine/IAlicaTrace.h"

#include <atomic>
#include <memory>
#include <string>
#include <functional>

namespace alica
{
namespace test
{
class TestContext;
}
class Configuration;
class RunningPlan;
class ThreadSafePlanInterface;
class AlicaEngine;
class IAlicaWorldModel;

/**
 * The base class for BasicBehaviour and BasicPlan
 */
class RunnableObject
{
protected:
    RunnableObject(IAlicaWorldModel* wm, const std::string& name = "");
    virtual ~RunnableObject() = default;
    void setEngine(AlicaEngine* engine) { _engine = engine; };
    void setConfiguration(const Configuration* conf) { _configuration = conf; };
    void setName(const std::string& name) { _name = name; };
    const std::string& getName() { return _name; }
    AlicaTime getInterval() const { return _msInterval; };
    void setInterval(int32_t msInterval) { _msInterval = AlicaTime::milliseconds(msInterval); };

    void sendLogMessage(int level, const std::string& message) const;

    // This is not thread safe. Should only be called by the scheduler thread. TODO: make this private
    std::optional<std::string> getTraceContext() const { return _trace ? std::optional<std::string>(_trace->context()) : std::nullopt; };

    using Counter = uint64_t;

    static constexpr int DEFAULT_MS_INTERVAL = 100;

    void doStop();
    void doStart(RunningPlan* rp);
    void doInit();
    void doRun();
    void doTerminate();

    virtual void onInit_() = 0;
    virtual void onRun_() = 0;
    virtual void onTerminate_() = 0;

    enum class TracingType : uint8_t
    {
        // Use the trace context of the lowest ancestor plan for which tracing is not skipped
        DEFAULT,
        // Skip tracing
        SKIP,
        // Create a root trace
        ROOT,
        // Provide a custom trace context
        CUSTOM
    };

    IAlicaTrace* getTrace() const { return _trace.get(); };

    bool isExecutingInContext() const
    {
        Counter sc = _signalState.load();
        Counter ec = _execState.load();
        return sc == ec && isActive(sc);
    };

    // If the counter is even then it indicates the behaviour is active i.e it is started, but not stopped yet
    constexpr static bool isActive(Counter cnt) { return !(cnt & 1); };

    ThreadSafePlanInterface getPlanContext() const;

    void startTrace();
    IAlicaWorldModel* getWorldModel() { return _wm; };

    IAlicaWorldModel* _wm;
    std::string _name;
    AlicaEngine* _engine;
    const Configuration* _configuration;
    AlicaTime _msInterval;
    TracingType _tracingType;
    std::function<std::optional<std::string>()> _customTraceContextGetter;
    std::unique_ptr<IAlicaTrace> _trace;
    std::atomic<Counter> _execState;          // Tracks the actual executate state of the behaviour by the scheduler thread
    std::atomic<Counter> _signalState;        // Tracks the signal state from the alica main engine thread i.e. tracks start() & stop() calls
    std::atomic<RunningPlan*> _signalContext; // The running plan context when start() is called
    std::atomic<RunningPlan*> _execContext;   // The running plan context under which the behaviour is executing
    bool _initExecuted;
    int64_t _activeRunJobId;
    bool _runTraced;
};
} /* namespace alica */
