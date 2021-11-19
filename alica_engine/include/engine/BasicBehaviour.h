#pragma once

#include "engine/Assignment.h"
#include "engine/IAlicaTrace.h"
#include "engine/RunnableObject.h"
#include "engine/Types.h"
#include "engine/model/Behaviour.h"

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
class EntryPoint;

/**
 * The base class for all behaviours. All Behaviours must inherit from this class.
 */
class BasicBehaviour : private RunnableObject
{
public:
    BasicBehaviour(const std::string& name);
    virtual ~BasicBehaviour(){};

    // Use of private inheritance and explciltly making members public
    // to share code between BasicBehaviour and Runnable object but not expose internals to further derived classes
    using RunnableObject::getPlanContext;
    using RunnableObject::getTraceContext;
    using RunnableObject::setConfiguration;
    using RunnableObject::setEngine;
    using RunnableObject::setInterval;
    using RunnableObject::setRequiresParameters;
    using RunnableObject::setName;
    using RunnableObject::start;
    using RunnableObject::stop;
    using RunnableObject::getBlackBoard;

    virtual void run(void* msg) = 0;

    // This method returns true when it is safe to delete the RunningPlan instance that is passed to it
    // Note that for things to work correctly it is assumed that this method is called after start() has finished execution
    // i.e. either on the same thread or via some other synchronization mechanism
    bool isRunningInContext(const RunningPlan* rp) const { return rp == _execContext.load() || rp == _signalContext.load(); };
    const std::string& getName() const { return _name; }

    void setBehaviour(const Behaviour* beh) { _behaviour = beh; };

    const VariableGrp& getVariables() const { return _behaviour->getVariables(); }
    const Variable* getVariable(const std::string& name) const { return _behaviour->getVariable(name); };

    void terminate() { stop(); };

    void setDelayedStart(int32_t msDelayedStart) { _msDelayedStart = AlicaTime::milliseconds(msDelayedStart); }

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

    bool isEventDriven() const { return _behaviour->isEventDriven(); }

protected:
    essentials::IdentifierConstPtr getOwnId() const;
    const AlicaEngine* getEngine() const { return _engine; }

    void setSuccess();
    void setFailure();

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
    enum class BehResult : uint8_t
    {
        UNKNOWN,
        SUCCESS,
        FAILURE
    };

    void runJob(void* msg);
    void doInit() override;
    void doTerminate() override;

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

    const Behaviour* _behaviour;
    AlicaTime _msDelayedStart;
    std::atomic<BehResult> _behResult;
    std::atomic<bool> _triggeredJobRunning;
};
} /* namespace alica */
