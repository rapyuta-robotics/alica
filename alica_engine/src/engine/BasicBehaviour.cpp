#include "engine/BasicBehaviour.h"
#include "engine/Assignment.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanBase.h"
#include "engine/TeamObserver.h"
#include "engine/logging/Logging.h"
#include "engine/model/Behaviour.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include "engine/model/Configuration.h"
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
    } catch (const std::exception& e) {
        Logging::logError("BasicBehaviour") << "Exception in Behaviour-INIT of: " << getName() << ": " << e.what();
    }
}

void BasicBehaviour::doRun()
{
    try {
        run();
    } catch (const std::exception& e) {
        std::string err = std::string("Exception caught:  ") + getName() + std::string(" - ") + std::string(e.what());
        sendLogMessage(4, err);
    }
    _triggeredJobRunning = false;
}

void BasicBehaviour::doTerminate()
{
    try {
        onTermination();
    } catch (const std::exception& e) {
        Logging::logError("BasicBehaviour") << "Exception in Behaviour-TERMINATE of: " << getName() << ": " << e.what();
    }

    _behResult.store(BehResult::UNKNOWN);
}

void BasicBehaviour::doTrigger()
{
    if (!_behaviour->isEventDriven() || !isTriggeredRunFinished()) {
        return;
    }
    _triggeredJobRunning = true;
    doRun();
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
            getTrace()->setTag("Result", (result == BehResult::SUCCESS ? "Success" : "Fail"));
        }
    }
}

} /* namespace alica */
