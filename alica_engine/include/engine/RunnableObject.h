#pragma once

#include "engine/AlicaClock.h"
#include "engine/IAlicaTrace.h"

#include "engine/IAlicaTimer.h"
#include "engine/blackboard/Blackboard.h"
#include "engine/blackboard/BlackboardBlueprint.h"
#include "engine/blackboard/KeyMapping.h"

#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace alica
{
namespace test
{
class TestContext;
}
class RunningPlan;
class Blackboard;
class TeamManager;
class PlanBase;
class IAlicaCommunication;
class IAlicaTraceFactory;
class IAlicaTimerFactory;

static constexpr const char* LOGNAME = "RunnableObject";

class TraceRunnableObject
{
public:
    TraceRunnableObject(const IAlicaTraceFactory* tf, const std::string& name)
            : _tracingType(TracingType::DEFAULT)
            , _runTraced(false)
            , _tf(tf)
            , _name(name)
    {
    }

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

    IAlicaTrace* getTrace() const { return _trace ? _trace.get() : nullptr; };
    const IAlicaTraceFactory* getTraceFactory() const { return _tf; }

    // Set the tracing type for this runnable object. customTraceContextGetter is required for custom tracing
    // & this method will be called to get the parent trace context before initialiseParameters is called
    void setTracing(TracingType type, std::function<tracing::SpanStartOptions()> customTraceContextGetter = {});
    void setupTraceContext(const std::string& name, RunningPlan* rp);
    void cleanupTraceContext();
    void traceInitCall();
    void traceRunCall();
    void traceTerminateCall();
    void traceException(const std::string& exceptionOriginMethod, const std::string& details);
    void finishTrace() { _trace.reset(); }
    const std::string& getName() const { return _name; }

private:
    TracingType _tracingType;
    bool _runTraced; // True if the behaviour/plan's run method has already been logged in the trace
    const IAlicaTraceFactory* _tf;
    std::function<tracing::SpanStartOptions()> _customTraceContextGetter;
    std::unique_ptr<IAlicaTrace> _trace;
    const std::string& _name;
};

/**
 * The base class for BasicBehaviour and BasicPlan
 */
class RunnableObject
{
protected:
    using TracingType = TraceRunnableObject::TracingType;

    RunnableObject(std::shared_ptr<Blackboard> globalBlackboard, const IAlicaTraceFactory* tf, const std::string& name = "");
    virtual ~RunnableObject() = default;

    static constexpr int DEFAULT_MS_INTERVAL = 100;

    virtual void doInit() = 0;
    virtual void doRun() = 0;
    virtual void doTerminate() = 0;

    void setTracing(TracingType type, std::function<tracing::SpanStartOptions()> customTraceContextGetter = {})
    {
        _runnableObjectTracer.setTracing(type, customTraceContextGetter);
    }

    const std::string& getName() const { return _name; };
    IAlicaTrace* getTrace() const { return _runnableObjectTracer.getTrace(); };
    // Helper to allow applications to generate their own trace.
    const IAlicaTraceFactory* getTraceFactory() const { return _runnableObjectTracer.getTraceFactory(); }

    void sendLogMessage(int level, const std::string& message) const;
    RunningPlan* getPlanContext() const { return _runningplanContext; }
    const std::shared_ptr<Blackboard> getBlackboard() { return _blackboard; }
    const BlackboardBlueprint* getBlackboardBlueprint() const { return _blackboardBlueprint; }
    const std::shared_ptr<Blackboard> getGlobalBlackboard() { return _globalBlackboard; };

    void start(RunningPlan* rp);
    void stop();

    // Only plan will have these
    void addKeyMapping(int64_t wrapperId, const KeyMapping* keyMapping);
    void setInterval(int32_t msInterval) { _msInterval = AlicaTime::milliseconds(msInterval); };
    bool getInheritBlackboard() const { return _blackboardBlueprint == nullptr; };
    void setBlackboardBlueprint(const BlackboardBlueprint* blackboard) { _blackboardBlueprint = blackboard; }
    const KeyMapping* getKeyMapping(int64_t id) const { return _keyMappings.at(id); }

    const TeamManager& getTeamManager() const;
    void setTeamManager(const TeamManager* teamManager);
    void setAlicaTraceFactory(const IAlicaTraceFactory* traceFactory);
    void setPlanBase(PlanBase* planBase);
    void setAlicaCommunication(const IAlicaCommunication* communication);
    void setAlicaTimerFactory(const IAlicaTimerFactory* timerFactory);

    void handleException(const std::string& exceptionOriginMethod, std::exception_ptr eptr);

    TraceRunnableObject _runnableObjectTracer;
    const TeamManager* _teamManager{nullptr};
    PlanBase* _planBase{nullptr};
    const IAlicaCommunication* _communication{nullptr};
    const IAlicaTraceFactory* _traceFactory{nullptr};
    const IAlicaTimerFactory* _timerFactory{nullptr};

private:
    void setupBlackboard();
    void scheduleRunCalls();
    void stopRunCalls();
    void cleanupBlackboard();
    void runJob();

    RunningPlan* _runningplanContext;
    std::string _name;
    AlicaTime _msInterval;
    std::unique_ptr<IAlicaTimer> _activeRunTimer;
    const BlackboardBlueprint* _blackboardBlueprint;
    std::shared_ptr<Blackboard> _blackboard;
    std::shared_ptr<Blackboard> _globalBlackboard;

    // Map from ConfAbstractPlanWrapper id to associated attachment
    // Only plan will have these
    std::unordered_map<int64_t, const KeyMapping*> _keyMappings;

    bool _started;
};
} /* namespace alica */
