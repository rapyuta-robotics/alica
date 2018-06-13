#pragma once

#include "engine/Assignment.h"
#include "engine/PlanInterface.h"
#include "engine/Types.h"
#include "engine/model/BehaviourConfiguration.h"
#include <supplementary/AgentID.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace supplementary
{
class Timer;
class ITrigger;
} // namespace supplementary

namespace alica
{
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
    bool isProperlyStopped() const;
    void setEngine(AlicaEngine* engine) { _engine = engine; }
    const std::string& getName() const { return _name; }
    const BehaviourParameterMap& getParameters() const { return _configuration->getParameters(); }

    void setName(const std::string& name);
    void setConfiguration(const BehaviourConfiguration* beh);

    const VariableGrp& getVariables() const { return _configuration->getVariables(); }
    const Variable* getVariableByName(const std::string& name) const;

    bool stop();
    bool start();

    void setDelayedStart(int32_t msDelayedStart) { _msDelayedStart = std::chrono::milliseconds(msDelayedStart); }

    void setInterval(int32_t msInterval) { _msInterval = std::chrono::milliseconds(msInterval); }

    ThreadSafePlanInterface getPlanContext() const { return ThreadSafePlanInterface(_context); }
    void setRunningPlan(RunningPlan* rp) { _context = rp; }

    bool isSuccess() const;
    void setSuccess();
    bool isFailure() const;
    void setFailure();

    bool getParameter(const std::string& key, std::string& valueOut) const;
    void setTrigger(supplementary::ITrigger* trigger);

    void sendLogMessage(int level, const std::string& message) const;

    virtual void init(){};

protected:
    AgentIDConstPtr getOwnId() const;

    /**
     * Called whenever a basic behaviour is started, i.e., when the corresponding state is entered.
     * Override for behaviour specific initialisation.
     */
    virtual void initialiseParameters(){};

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
    std::atomic<bool> _inRun;

    std::thread* _runThread; /** < executes the runInternal and thereby the abstract run method */

    supplementary::ITrigger* _behaviourTrigger; /** triggers the condition_variable of the runThread, if this behaviour
                                                  is event triggered, alternative to timer */
    std::condition_variable _runCV;
    mutable std::mutex _runLoopMutex;
    std::chrono::milliseconds _msInterval;
    std::chrono::milliseconds _msDelayedStart;
};
} /* namespace alica */
