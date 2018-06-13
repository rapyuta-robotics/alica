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
    const std::string& getName() const { return name; }
    const BehaviourParameterMap& getParameters() const { return _configuration->getParameters(); }

    void setName(const std::string& name);
    void setConfiguration(const BehaviourConfiguration* beh);

    const VariableGrp& getVariables() const { return _configuration->getVariables(); }
    const Variable* getVariableByName(const std::string& name) const;

    bool stop();
    bool start();

    int getDelayedStart() const;
    void setDelayedStart(long msDelayedStart);
    int getInterval() const;
    void setInterval(long msInterval);

    PlanInterface getPlanContext() const { return PlanInterface(_context); }
    void setRunningPlan(RunningPlan* rp) { _context = rp; }

    bool isSuccess() const;
    void setSuccess(bool success);
    bool isFailure() const;
    void setFailure(bool failure);

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

    /**
     * The name of this behaviour.
     */
    std::string name;

    const BehaviourConfiguration* _configuration;

    /**
     * The running plan representing this behaviour within the PlanBase.
     */

    std::chrono::milliseconds msInterval;
    std::chrono::milliseconds msDelayedStart;
    /**
     * is always true except when the behaviour is shutting down
     */
    bool started;
    bool callInit;

    /**
     * Tells us whether the behaviour is currently running (or active)
     */
    bool running;
    std::atomic<bool> _inRun;

    std::thread* runThread;                    /** < executes the runInternal and thereby the abstract run method */
    supplementary::Timer* timer;               /** < triggers the condition_variable of the runThread, if this behaviour is timer
                                                  triggered, alternative to behaviourTrigger*/
    supplementary::ITrigger* behaviourTrigger; /** triggers the condition_variable of the runThread, if this behaviour
                                                  is event triggered, alternative to timer */
    std::condition_variable runCV;

    AlicaEngine* _engine;

private:
    RunningPlan* _context;
    void runInternal();
    void initInternal();

    std::mutex runCV_mtx;
    /**
     * The Success flag. Raised by a behaviour to indicate it reached whatever it meant to reach.
     */
    bool success;
    /**
     * The Failure flag. Raised by a behaviour to indicate it has failed in some way.
     */
    bool failure;
};
} /* namespace alica */
