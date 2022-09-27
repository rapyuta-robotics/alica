#pragma once

#include "engine/AlicaClock.h"
#include "engine/IAlicaTrace.h"

#include <alica_common_config/debug_output.h>

#include "engine/IAlicaTimer.h"
#include "engine/blackboard/Blackboard.h"
#include "engine/blackboard/BlackboardBlueprint.h"
#include "engine/blackboard/KeyMapping.h"

#include <atomic>
#include <functional>
#include <memory>
#include <string>

namespace alica
{
namespace test
{
class TestContext;
}
class RunningPlan;
class IAlicaWorldModel;
class TeamManager;
class PlanBase;
class IAlicaCommunication;
class IAlicaTraceFactory;
class IAlicaTimerFactory;

class TraceRunnableObject
{
public:
    TraceRunnableObject(const IAlicaTraceFactory* tf)
            : _tracingType(TracingType::DEFAULT)
            , _runTraced(false)
            , _tf(tf)
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
    void setTracing(TracingType type, std::function<std::optional<std::string>()> customTraceContextGetter = {});
    void setupTraceContext(const std::string& name, RunningPlan* rp);
    void cleanupTraceContext();
    void traceRunCall();

private:
    TracingType _tracingType;
    const IAlicaTraceFactory* _tf;
    std::function<std::optional<std::string>()> _customTraceContextGetter;
    std::unique_ptr<IAlicaTrace> _trace;
    // True if the behaviour/plan's run method has already been logged in the trace
    bool _runTraced;
};

/**
 * The base class for BasicBehaviour and BasicPlan
 */
class RunnableObject
{
protected:
    using TracingType = TraceRunnableObject::TracingType;

    RunnableObject(IAlicaWorldModel* wm, const IAlicaTraceFactory* tf, const std::string& name = "");
    virtual ~RunnableObject() = default;

    static constexpr int DEFAULT_MS_INTERVAL = 100;

    virtual void doInit() = 0;
    virtual void doRun() = 0;
    virtual void doTerminate() = 0;

    void setTracing(TracingType type, std::function<std::optional<std::string>()> customTraceContextGetter = {})
    {
        _runnableObjectTracer.setTracing(type, customTraceContextGetter);
    }
    const std::string& getName() { return _name; };
    IAlicaTrace* getTrace() const { return _runnableObjectTracer.getTrace(); };
    // Helper to allow applications to generate their own trace.
    const IAlicaTraceFactory* getTraceFactory() const { return _runnableObjectTracer.getTraceFactory(); }

    void sendLogMessage(int level, const std::string& message) const;
    RunningPlan* getPlanContext() const { return _runningplanContext; }
    const std::shared_ptr<Blackboard> getBlackboard() { return _blackboard; }
    IAlicaWorldModel* getWorldModel() { return _wm; };

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
    IAlicaWorldModel* _wm;

    // Map from ConfAbstractPlanWrapper id to associated attachment
    // Only plan will have these
    std::unordered_map<int64_t, const KeyMapping*> _keyMappings;

    bool _started;
};
} /* namespace alica */
