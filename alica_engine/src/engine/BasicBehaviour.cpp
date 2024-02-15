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
{
    if (_behaviour->getFrequency() < 1) {
        // TODO: set interval to invalid value like -1 & have the basic behaviour not schedule run jobs for such intervals
        setInterval(0);
    } else {
        setInterval(1000 / _behaviour->getFrequency());
    }
    setBlackboardBlueprint(_behaviour->getBlackboardBlueprint());
    if (_behaviour->getBlackboardBlueprint()) {
        Logging::logDebug(LOGNAME) << "Adding blackboard blueprint for behavior " << _behaviour->getName() << " with " << *_behaviour->getBlackboardBlueprint();
    }
}

/**
 * Convenience method to obtain the robot's own id.
 * @return the own robot id
 */
AgentId BasicBehaviour::getOwnId() const
{
    return getTeamManager().getLocalAgentID();
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
}

std::string BasicBehaviour::resultToString(BehResult result)
{
    switch (result) {
    case BehResult::SUCCESS:
        return "Success";
    case BehResult::FAILURE:
        return "Failure";
    case BehResult::UNKNOWN:
        return "Unknown";
    }
    return ""; // should never reach
}

void BasicBehaviour::doTerminate()
{
    if (getTrace()) {
        const std::string resultStr = resultToString(_behResult.load());
        Logging::logInfo(LOGNAME) << "Behaviour: " << getName() << ", result: " << resultStr;
        getTrace()->setTag("Result", resultStr);
    }
    try {
        onTermination();
    } catch (...) {
        handleException("terminate", std::current_exception());
    }
    _behResult.store(BehResult::UNKNOWN);
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
    }
}

} /* namespace alica */
