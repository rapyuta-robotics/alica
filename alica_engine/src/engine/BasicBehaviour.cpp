#include "engine/BasicBehaviour.h"
#include "engine/Assignment.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanBase.h"
#include "engine/TeamObserver.h"
#include "engine/logging/Logging.h"
#include "engine/model/Behaviour.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Parameter.h"
#include "engine/model/Plan.h"
#include "engine/model/Variable.h"
#include "engine/teammanager/TeamManager.h"

#include <assert.h>
#include <iostream>

namespace alica
{
/**
 * Basic constructor. Initialises the timer. Should only be called from the constructor of inheriting classes.
 * If using eventTrigger set behaviourTrigger
 * @param name The name of the behaviour
 */
BasicBehaviour::BasicBehaviour(BehaviourContext& context)
        : RunnableObject(context.globalBlackboard, context.traceFactory, context.name)
        , _behaviour(context.behaviourModel)
        , _behResult(BehResult::UNKNOWN)
        , _triggeredJobRunning(false)
{
    if (_behaviour->getFrequency() < 1 || _behaviour->isEventDriven()) {
        // TODO: set interval to invalid value like -1 & have the basic behaviour not schedule run jobs for such intervals
        setInterval(0);
    } else {
        setInterval(1000 / _behaviour->getFrequency());
    }
    setBlackboardBlueprint(_behaviour->getBlackboardBlueprint());
}

/**
 * Convenience method to obtain the robot's own id.
 * @return the own robot id
 */
AgentId BasicBehaviour::getOwnId() const
{
    return getTeamManager().getLocalAgentID();
}

bool BasicBehaviour::isTriggeredRunFinished()
{
    return !_triggeredJobRunning.load();
}

void BasicBehaviour::doInit()
{
    try {
        initialiseParameters();
    } catch (...) {
        handleException("initialise", std::current_exception());
    }
}

void BasicBehaviour::doRun()
{
    try {
        run();
    } catch (...) {
        handleException("run", std::current_exception());
    }
    _triggeredJobRunning = false;
}

void BasicBehaviour::doTerminate()
{
    try {
        onTermination();
    } catch (...) {
        handleException("terminate", std::current_exception());
    }
    _behResult.store(BehResult::UNKNOWN);
}

void BasicBehaviour::doTrigger()
{
    if (!_behaviour->isEventDriven()) {
        Logging::logError(LOGNAME) << "Behaviour: " << getName() << ", Error: Trying to trigger a behaviour that is not event driven";
    } else if (!isTriggeredRunFinished()) {
        Logging::logError(LOGNAME) << "Behaviour: " << getName() << ", Error: Cannot trigger behaviour because the previous run is not yet finished";
    } else {
        _triggeredJobRunning = true;
        doRun();
    }
}

void BasicBehaviour::setSuccess()
{
    setResult(BehResult::SUCCESS);
}

void BasicBehaviour::setFailure()
{
    setResult(BehResult::FAILURE);
}

void BasicBehaviour::setResult(BehResult result)
{
    auto prev = _behResult.exchange(result);
    if (prev != result) {
        _planBase->addFastPathEvent(getPlanContext());
        if (getTrace()) {
            const char* resultStr = (result == BehResult::SUCCESS ? "Success" : "Fail");
            Logging::logInfo(LOGNAME) << "Behaviour: " << getName() << ", result: " << resultStr;
            getTrace()->setTag("Result", resultStr);
        }
    }
}

} /* namespace alica */
