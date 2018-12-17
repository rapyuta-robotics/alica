#pragma once

#include "engine/Assignment.h"
#include "engine/PlanInterface.h"
#include "engine/Types.h"
#include "engine/model/BehaviourConfiguration.h"
#include <essentials/Timer.h>
#include <essentials/ITrigger.h>

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
    virtual ~BasicBehaviour();
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

    void setDelayedStart(int32_t msDelayedStart) { _msDelayedStart = std::chrono::milliseconds(msDelayedStart); }

    void setInterval(int32_t msInterval) { _msInterval = std::chrono::milliseconds(msInterval); }

    ThreadSafePlanInterface getPlanContext() const { return ThreadSafePlanInterface(_contextInRun); }
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
    AlicaEngine* getEngine() const { return _engine; }

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
    void runInternalTimed();
    void runInternalTriggered();
    void initInternal();

    /**
     * The name of this behaviour.
     */
    std::string _name;

    const BehaviourConfiguration* _configuration;
    AlicaEngine* _engine;
    RunningPlan* _context;

    /**
     * is always true except when the behaviour is shutting down
     */
    bool _started;
    bool _callInit;
    /**
     * The Success flag. Raised by a behaviour to indicate it reached whatever it meant to reach.
     */
    bool _success;
    /**
     * The Failure flag. Raised by a behaviour to indicate it has failed in some way.
     */
    bool _failure;
    /**
     * Tells us whether the behaviour is currently running (or active)
     */
    bool _running;
    std::atomic<RunningPlan*> _contextInRun;

    std::thread* _runThread; /** < executes the runInternal and thereby the abstract run method */

    essentials::ITrigger* _behaviourTrigger; /** triggers the condition_variable of the runThread, if this behaviour
                                                  is event triggered, alternative to timer */
    std::condition_variable _runCV;
    mutable std::mutex _runLoopMutex;
    std::chrono::milliseconds _msInterval;
    std::chrono::milliseconds _msDelayedStart;
};
} /* namespace alica */
