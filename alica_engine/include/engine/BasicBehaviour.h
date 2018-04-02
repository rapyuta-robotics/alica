#pragma once

#include "supplementary/AgentID.h"

#include <string>
#include <iostream>
#include <map>
#include <memory>
#include <list>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <vector>

namespace supplementary {
class Timer;
class ITrigger;
}  // namespace supplementary

namespace alica {
class Variable;
class RunningPlan;
class BehaviourConfiguration;
class EntryPoint;
class AlicaEngine;

/**
 * The base class for all behaviours. All Behaviours must inherit from this class.
 */
class BasicBehaviour {
public:
    BasicBehaviour(std::string name);
    virtual ~BasicBehaviour();
    virtual void run(void* msg) = 0;
    const std::string getName() const;
    void setName(std::string name);
    std::shared_ptr<std::map<std::string, std::string>> getParameters();
    void setParameters(std::shared_ptr<std::map<std::string, std::string>> parameters);
    std::shared_ptr<std::list<Variable*>> getVariables();
    Variable* getVariablesByName(std::string name);
    void setVariables(std::shared_ptr<std::list<Variable*>> variables);
    bool stop();
    bool start();
    int getDelayedStart() const;
    void setDelayedStart(long msDelayedStart);
    int getInterval() const;
    void setInterval(long msInterval);
    std::shared_ptr<RunningPlan> getRunningPlan();
    void setRunningPlan(std::shared_ptr<RunningPlan> runningPlan);
    bool isSuccess() const;
    void setSuccess(bool success);
    void setEngine(AlicaEngine* engine);
    bool isFailure() const;
    void setFailure(bool failure);

    bool getParameter(std::string key, std::string& valueOut);
    void setTrigger(supplementary::ITrigger* trigger);

    void sendLogMessage(int level, std::string& message);

    virtual void init(){

    };

protected:
    /**
     * The name of this behaviour.
     */
    std::string name;
    /**
     * Parameters are behaviour configuration specific fixed values. They are set before the behaviour is activated.
     */
    std::shared_ptr<std::map<std::string, std::string>> parameters;
    /**
     * The set of Variables attached to this behaviours as defined by the BehaviourConfiguration.
     */
    std::shared_ptr<std::list<Variable*>> variables;
    /**
     * The running plan representing this behaviour within the PlanBase.
     */
    std::shared_ptr<RunningPlan> runningPlan;
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

    std::thread* runThread;      /** < executes the runInternal and thereby the abstract run method */
    supplementary::Timer* timer; /** < triggers the condition_variable of the runThread, if this behaviour is timer
                                    triggered, alternative to behaviourTrigger*/
    supplementary::ITrigger* behaviourTrigger; /** triggers the condition_variable of the runThread, if this behaviour
                                                  is event triggered, alternative to timer */
    std::condition_variable runCV;
    const supplementary::AgentID* getOwnId();

    /**
     * Called whenever a basic behaviour is started, i.e., when the corresponding state is entered.
     * Override for behaviour specific initialisation.
     */
    virtual void initialiseParameters(){};

    EntryPoint* getParentEntryPoint(std::string taskName);

    EntryPoint* getHigherEntryPoint(std::string planName, std::string taskName);

    std::shared_ptr<std::vector<const supplementary::AgentID*>> robotsInEntryPointOfHigherPlan(EntryPoint* ep);

    std::shared_ptr<std::vector<const supplementary::AgentID*>> robotsInEntryPoint(EntryPoint* ep);

private:
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

    AlicaEngine* engine;
};
} /* namespace alica */
