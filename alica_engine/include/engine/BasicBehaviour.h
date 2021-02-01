#pragma once

#include "engine/Assignment.h"
#include "engine/PlanInterface.h"
#include "engine/Types.h"
#include "engine/model/Behaviour.h"

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

    bool isRunningInContext(const RunningPlan* rp) const;
    void setEngine(AlicaEngine* engine) { _engine = engine; }
    const std::string& getName() const { return _name; }

    void setBehaviour(const Behaviour* beh);
    void setConfiguration(const Configuration* conf);

    const VariableGrp& getVariables() const { return _behaviour->getVariables(); }
    const Variable* getVariable(const std::string& name) const { return _behaviour->getVariable(name); };

    bool stop();
    bool start();
    void terminate();

    void setDelayedStart(int32_t msDelayedStart) { _msDelayedStart = AlicaTime::milliseconds(msDelayedStart); }

    void setInterval(int32_t msInterval) { _msInterval = AlicaTime::milliseconds(msInterval); }

    ThreadSafePlanInterface getPlanContext() const { return ThreadSafePlanInterface(isBehaviourStarted() ? _context : nullptr); }
    void setRunningPlan(RunningPlan* rp) { _context = rp; }

    bool isSuccess() const;
    void setSuccess();
    bool isFailure() const;
    void setFailure();

    bool getParameter(const std::string& key, std::string& valueOut) const;
    void setTrigger(essentials::ITrigger* trigger);
    bool isTriggeredRunFinished();

    void sendLogMessage(int level, const std::string& message) const;

    /**
     * Called after construction.
     * Override in case custom initialization has to happen after the behavior has been integrated into the engine.
     */
    virtual void init() {}

    AlicaTime getInterval() { return _msInterval;}

protected:
    essentials::IdentifierConstPtr getOwnId() const;
    const AlicaEngine* getEngine() const { return _engine; }

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
    enum BehaviourResult
    {
        UNKNOWN,
        SUCCESS,
        FAILURE
    };
    enum BehaviourState
    {
        UNINITIALIZED,
        INITIALIZING,
        RUNNING,
        TERMINATING
    };
    enum SignalState
    {
        START,
        STOP,
        TERMINATE
    };

    SignalState getSignalState() const { return _signalState; }
    void setSignalState(SignalState signalState) { _signalState = signalState; }
    bool isTerminated() const { return _signalState == SignalState::TERMINATE; }
    BehaviourState getBehaviourState() const { return _behaviourState.load(); }
    void setBehaviourState(BehaviourState state) { _behaviourState.store(state); }
    BehaviourResult getBehaviourResult() const { return _behaviourResult.load(); }
    void setBehaviourResult(BehaviourResult result) { _behaviourResult.store(result); }
    bool isStopCalled() const { return _stopCalled.load(); }
    void setStopCalled(bool val) { _stopCalled.store(val); }
    bool isBehaviourStarted() const { return _behaviourState.load() != BehaviourState::UNINITIALIZED; }

    void runThread(bool timed);

    // wait, init, run & stop the behaviour
    bool doWait();
    void doInit(bool timed);
    void doRun(bool timed);
    void doStop();

    /**
     * The name of this behaviour.
     */
    std::string _name;

    const Behaviour* _behaviour;
    AlicaEngine* _engine;
    RunningPlan* _context;

    /**
     * The configuration, that is set in the behaviour pool, associated with
     * this basic behaviour through its corresponding ConfAbstractPlanWrapper.
     */
    const Configuration* _configuration;
    std::atomic<RunningPlan*> _contextInRun;
    SignalState _signalState;      // current state of the signal from main thread (start, stop or terminate)
    std::atomic<bool> _stopCalled; // used by behaviour thread to check if stop was signalled while it was running user code

    std::atomic<BehaviourResult> _behaviourResult;
    std::atomic<BehaviourState> _behaviourState;

    std::thread* _runThread; /** < executes the runInternal and thereby the abstract run method */

    essentials::ITrigger* _behaviourTrigger; /** triggers the condition_variable of the runThread, if this behaviour
                                                  is event triggered, alternative to timer */
    std::condition_variable _runCV;
    mutable std::mutex _runLoopMutex;
    AlicaTime _msInterval;
    AlicaTime _msDelayedStart;
};
} /* namespace alica */
