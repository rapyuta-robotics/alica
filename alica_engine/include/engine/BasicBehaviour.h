#pragma once

#include "engine/Assignment.h"
#include "engine/PlanInterface.h"
#include "engine/Types.h"
#include "engine/model/BehaviourConfiguration.h"
#include <essentials/ITrigger.h>
#include <essentials/Timer.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace alica
{
// TODO: get rid of this line once generator templates get an overhaul
using std::string;

class Variable;
class RunningPlan;

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
    const BehaviourParameterMap& getParameters() const { return _configuration->getParameters(); }

    void setConfiguration(const BehaviourConfiguration* beh);

    const VariableGrp& getVariables() const { return _configuration->getVariables(); }
    const Variable* getVariableByName(const std::string& name) const;

    bool stop();
    bool start();
    void terminate();

    void setDelayedStart(int32_t msDelayedStart) { _msDelayedStart = AlicaTime::milliseconds(msDelayedStart); }

    void setInterval(int32_t msInterval) { _msInterval = AlicaTime::milliseconds(msInterval); }

    ThreadSafePlanInterface getPlanContext() const { return ThreadSafePlanInterface(isBehaviourStarted() && !isStopCalled() ? _context : nullptr); }
    void setRunningPlan(RunningPlan* rp) { _context = rp; }

    bool isSuccess() const;
    void setSuccess();
    bool isFailure() const;
    void setFailure();

    bool getParameter(const std::string& key, std::string& valueOut) const;
    void setTrigger(essentials::ITrigger* trigger);

    void sendLogMessage(int level, const std::string& message) const;

    /**
     * Called after construction.
     * Override in case custom initialization has to happen after the behavior has been integrated into the engine.
     */
    virtual void init() {}

protected:
    AgentIDConstPtr getOwnId() const;
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
    enum BehaviourResult { UNKNOWN, SUCCESS, FAILURE };
    enum BehaviourState { UNINITIALIZED, STARTED, RUNNING };
    enum SignalState { START, STOP, TERMINATE };

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

    const BehaviourConfiguration* _configuration;
    AlicaEngine* _engine;
    RunningPlan* _context;

    SignalState _signalState; // current state of the signal from main thread (start, stop or terminate)
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
