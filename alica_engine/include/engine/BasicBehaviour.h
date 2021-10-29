#pragma once

#include "engine/Assignment.h"
#include "engine/PlanInterface.h"
#include "engine/Types.h"
#include "engine/model/Behaviour.h"
#include "engine/IAlicaTrace.h"

#include <essentials/ITrigger.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace alica
{
namespace test
{
class TestContext;
}
class Variable;
class RunningPlan;
class Configuration;
class EntryPoint;
class AlicaEngine;

/**
 * The base class for all behaviours. All Behaviours must inherit from this class.
 */
class BasicBehaviour
{
public:
    BasicBehaviour(const std::string& name);
    virtual ~BasicBehaviour(){};
    virtual void run(void* msg) = 0;

    // This method returns true when it is safe to delete the RunningPlan instance that is passed to it
    // Note that for things to work correctly it is assumed that this method is called after start() has finished execution
    // i.e. either on the same thread or via some other synchronization mechanism
    bool isRunningInContext(const RunningPlan* rp) const { return rp == _execContext.load() || rp == _signalContext.load(); };
    void setEngine(AlicaEngine* engine) { _engine = engine; }
    const std::string& getName() const { return _name; }

    void setBehaviour(const Behaviour* beh) { _behaviour = beh; };
    void setConfiguration(const Configuration* conf) { _configuration = conf; };

    const VariableGrp& getVariables() const { return _behaviour->getVariables(); }
    const Variable* getVariable(const std::string& name) const { return _behaviour->getVariable(name); };

    bool stop();
    bool start(RunningPlan* rp);
    void terminate() { stop(); };

    void setDelayedStart(int32_t msDelayedStart) { _msDelayedStart = AlicaTime::milliseconds(msDelayedStart); }

    void setInterval(int32_t msInterval) { _msInterval = AlicaTime::milliseconds(msInterval); }

    bool isSuccess() const { return isExecutingInContext() && _behResult.load() == BehResult::SUCCESS; };
    bool isFailure() const { return isExecutingInContext() && _behResult.load() == BehResult::FAILURE; };

    bool getParameter(const std::string& key, std::string& valueOut) const;

    void doTrigger();
    bool isTriggeredRunFinished();

    /**
     * Called after construction.
     * Override in case custom initialization has to happen after the behavior has been integrated into the engine.
     */
    virtual void init() {}

    AlicaTime getInterval() { return _msInterval; }

    bool isEventDriven() const { return _behaviour->isEventDriven(); }

    // This is not thread safe. Should only be called by the scheduler thread. TODO: make this private
    std::optional<std::string> getTraceContext() const
    {
        return _trace ? std::optional<std::string>(_trace->context()) : std::nullopt;
    }

protected:
    essentials::IdentifierConstPtr getOwnId() const;
    const AlicaEngine* getEngine() const { return _engine; }

    ThreadSafePlanInterface getPlanContext() const { return ThreadSafePlanInterface(isExecutingInContext() ? _execContext.load() : nullptr); }
    void setSuccess();
    void setFailure();

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

    // Set the tracing type for this behaviour. customTraceContextGetter is required for custom tracing
    // & this method will be called to get the parent trace context before initialiseParameters is called
    void setTracing(TracingType type, std::function<std::string(BasicBehaviour*)> customTraceContextGetter = {})
    {
        _tracingType = type;
        _customTraceContextGetter = std::move(customTraceContextGetter);
    }

    std::optional<IAlicaTrace*> getTrace() const
    {
        return _trace ? std::optional<IAlicaTrace*>(_trace.get()) : std::nullopt;
    }

    /**
     * Called whenever a basic behaviour is started, i.e., when the corresponding state is entered.
     * Override for behaviour specific initialisation. Guaranteed to be executed on the behavior's thread.
     */
    virtual void initialiseParameters() {}

    /**
     * Called whenever a basic behavior is stopped, i.e., when the corresponding state is left.
     * Override for behaviour specific termination. Guaranteed to be executed on the behavior's thread.
     */
    virtual void onTermination() {}

private:
    friend alica::test::TestContext;
    using Counter = uint64_t;
    enum class BehResult : uint8_t
    {
        UNKNOWN,
        SUCCESS,
        FAILURE
    };

    void runJob(void* msg);
    void initJob();
    void terminateJob();

    void sendLogMessage(int level, const std::string& message) const;

    /*
     * The Alica main engine thread calls start() & stop() whenever the current running plan corresponds to this behaviour
     * & _signalState is used to track this context [signal context].
     *
     * The scheduler thread is the one that actually executes the initialiseParameters(), run() & onTermination() methods
     * of this behaviour & _execState is used to track this context [execution context].
     *
     * The states are tracked using simple counters: if the counter is even then the behaviour is active within that context
     * i.e. behaviour is started [equivalently: not stopped] & the counter is incremented whenever the behaviour is started
     * or stopped within its context.
     *
     * Therefore the execution of the behaviour by the scheduler thread is in the context of the running plan only if
     * _signalState == _execState && _signalState is even.
     *
     * The behaviour also maintains the current RunningPlan context under which it is executing & this is used by the alica
     * main thread to know when it is safe to destroy the RunningPlan object.
     *
     */

    // If the counter is even then it indicates the behaviour is active i.e it is started, but not stopped yet
    static constexpr bool isActive(Counter cnt) { return !(cnt & 1); }

    // Returns true if the behaviour is executing in the context of the running plan
    bool isExecutingInContext() const
    {
        Counter sc = _signalState.load(), ec = _execState.load();
        return sc == ec && isActive(sc);
    }

    void startTrace();

    /**
     * The name of this behaviour.
     */
    std::string _name;

    const Behaviour* _behaviour;
    AlicaEngine* _engine;

    /**
     * The configuration, that is set in the behaviour pool, associated with
     * this basic behaviour through its corresponding ConfAbstractPlanWrapper.
     */
    const Configuration* _configuration;

    AlicaTime _msInterval;
    AlicaTime _msDelayedStart;

    // TODO: Optimization: It should be okay (confirm it) for all accesses to atomic variables to be done using acquire-release semantics instead
    // of the currently used sequentially-consistent ordering which can be a performance bottleneck because of the necessiated memory fence instruction
    // (see https://en.cppreference.com/w/cpp/atomic/memory_order#Sequentially-consistent_ordering)
    std::atomic<RunningPlan*> _signalContext; // The running plan context when start() is called
    std::atomic<RunningPlan*> _execContext; // The running plan context under which the behaviour is executing
    std::atomic<Counter> _signalState; // Tracks the signal state from the alica main engine thread i.e. tracks start() & stop() calls
    std::atomic<Counter> _execState; // Tracks the actual executate state of the behaviour by the scheduler thread
    std::atomic<BehResult> _behResult;

    int64_t _activeRunJobId;
    std::atomic<bool> _triggeredJobRunning;

    TracingType _tracingType;
    std::function<std::string(BasicBehaviour*)> _customTraceContextGetter;
    std::unique_ptr<IAlicaTrace> _trace;
    bool _runTraced;

    bool _initExecuted;
};
} /* namespace alica */
