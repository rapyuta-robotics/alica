#include "engine/BasicBehaviour.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanBase.h"
#include "engine/TeamObserver.h"
#include "engine/model/Behaviour.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include "engine/model/Configuration.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Parameter.h"
#include "engine/model/Plan.h"
#include "engine/model/Variable.h"
#include "engine/scheduler/Scheduler.h"
#include "engine/teammanager/TeamManager.h"

#include <alica_common_config/debug_output.h>

#include <assert.h>
#include <iostream>

namespace alica
{
/**
 * Basic constructor. Initialises the timer. Should only be called from the constructor of inheriting classes.
 * If using eventTrigger set behaviourTrigger
 * @param name The name of the behaviour
 */
BasicBehaviour::BasicBehaviour(IAlicaWorldModel* wm, const std::string& name)
        : RunnableObject(wm, name)
        , _behaviour(nullptr)
        , _msDelayedStart(AlicaTime::milliseconds(0))
        , _behResult(BehResult::UNKNOWN)
        , _triggeredJobRunning(false)
{
}

/**
 * Convenience method to obtain the robot's own id.
 * @return the own robot id
 */
AgentId BasicBehaviour::getOwnId() const
{
    return _engine->getTeamManager().getLocalAgentID();
}

void BasicBehaviour::stop()
{
    doStop();
    _engine->editScheduler().schedule(std::bind(&BasicBehaviour::terminateJob, this));
}

void BasicBehaviour::start(RunningPlan* rp)
{
    doStart(rp);
    _engine->editScheduler().schedule(std::bind(&BasicBehaviour::initJob, this));
}

void BasicBehaviour::setFailure()
{
    if (!isExecutingInContext()) {
        return;
    }
    auto prev = _behResult.exchange(BehResult::FAILURE);
    if (prev != BehResult::FAILURE) {
        _engine->editPlanBase().addFastPathEvent(_execContext.load());
        if (_trace) {
            _trace->setTag("Result", "Fail");
        }
    }
}

bool BasicBehaviour::isTriggeredRunFinished()
{
    return !_triggeredJobRunning.load();
}

void BasicBehaviour::initJob()
{
    assert(_behResult.load() == BehResult::UNKNOWN);

    doInit();

    // Do not schedule repeatable run job when behaviour is event driven or when frequency is 0.
    if (!isEventDriven() && _msInterval > AlicaTime::milliseconds(0)) {
        // TODO: account for delayed start
        _activeRunJobId = _engine->editScheduler().schedule(std::bind(&BasicBehaviour::runJob, this), getInterval());
    }
}

void BasicBehaviour::runJob()
{
    doRun();
    _triggeredJobRunning = false;
}

void BasicBehaviour::doTrigger()
{
    if (!_behaviour->isEventDriven() || !isTriggeredRunFinished()) {
        return;
    }
    _triggeredJobRunning = true;
    _engine->editScheduler().schedule(std::bind(&BasicBehaviour::runJob, this));
}

void BasicBehaviour::terminateJob()
{
    if (_activeRunJobId != -1) {
        _engine->editScheduler().cancelJob(_activeRunJobId);
        _activeRunJobId = -1;
    }
    // Just to be double safe in terms of the correct behaviour of isSuccess() & isFailure() ensure result is reset before incrementing _execState
    _behResult.store(BehResult::UNKNOWN);
    doTerminate();
}

void BasicBehaviour::onInit_()
{
    if (_trace) {
        _trace->setTag("type", "behaviour");
    }
    initialiseParameters();
}

void BasicBehaviour::onRun_()
{
    run(nullptr);
}

void BasicBehaviour::onTerminate_()
{
    onTermination();
}

bool BasicBehaviour::getParameter(const std::string& key, std::string& valueOut) const
{
    if (!_configuration) {
        valueOut.clear();
        return false;
    }

    const auto& parameter = _configuration->getParameters().find(key);
    if (parameter != _configuration->getParameters().end()) {
        valueOut = parameter->second->getValue();
        return true;
    } else {
        valueOut.clear();
        return false;
    }
}

void BasicBehaviour::setSuccess()
{
    if (!isExecutingInContext()) {
        return;
    }
    auto prev = _behResult.exchange(BehResult::SUCCESS);
    if (prev != BehResult::SUCCESS) {
        _engine->editPlanBase().addFastPathEvent(_execContext.load());
        if (_trace) {
            _trace->setTag("Result", "Success");
        }
    }
}

} /* namespace alica */
