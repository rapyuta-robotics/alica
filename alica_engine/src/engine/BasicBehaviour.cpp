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
BasicBehaviour::BasicBehaviour(const std::string& name)
        : _name(name)
        , _behaviour(nullptr)
        , _engine(nullptr)
        , _configuration(nullptr)
        , _msInterval(AlicaTime::milliseconds(100))
        , _msDelayedStart(AlicaTime::milliseconds(0))
        , _context(nullptr)
        , _signalState(1)
        , _execState(1)
        , _behResult(BehResult::UNKNOWN)
        , _activeRunJobId(-1)
        , _triggeredJobRunning(false)
        , _tracingDisabled(false)
{
}

/**
 * Convenience method to obtain the robot's own id.
 * @return the own robot id
 */
essentials::IdentifierConstPtr BasicBehaviour::getOwnId() const
{
    return _engine->getTeamManager().getLocalAgentID();
}

/**
 * Stops the execution of this BasicBehaviour.
 */
bool BasicBehaviour::stop()
{
    if (!isActive(_signalState.load())) {
        return true;
    }
    ++_signalState;
    _engine->editScheduler().schedule(std::bind(&BasicBehaviour::terminateJob, this));
    return true;
}

/**
 * Starts the execution of this BasicBehaviour.
 */
bool BasicBehaviour::start(RunningPlan* rp)
{
    if (isActive(_signalState.load())) {
        return true;
    }
    ++_signalState;
    _context.store(rp);
    startTrace();
    _engine->editScheduler().schedule(std::bind(&BasicBehaviour::initJob, this));
    return true;
}

void BasicBehaviour::setSuccess()
{
    if (!isExecutingInContext()) {
        return;
    }
    auto prev = _behResult.exchange(BehResult::SUCCESS);
    if (prev != BehResult::SUCCESS) {
        _engine->editPlanBase().addFastPathEvent(_context.load());
    }
}

void BasicBehaviour::setFailure()
{
    if (!isExecutingInContext()) {
        return;
    }
    auto prev = _behResult.exchange(BehResult::FAILURE);
    if (prev != BehResult::FAILURE) {
        _engine->editPlanBase().addFastPathEvent(_context.load());
    }
}

bool BasicBehaviour::isTriggeredRunFinished()
{
    return !_triggeredJobRunning.load();
}

void BasicBehaviour::initJob()
{
    assert(_behResult.load() == BehResult::UNKNOWN);
    ++_execState;

    // Todo: Optimization: don't call initialiseParameters if not in context
    try {
        if (_trace) {
            _trace->setTag("Init", "true");
        }
        initialiseParameters();
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("[BasicBehaviour] Exception in Behaviour-INIT of: " << getName() << std::endl << e.what());
    }

    // Do not schedule repeatable run job when behaviour is event driven or when frequency is 0.
    if (!isEventDriven() && _msInterval > AlicaTime::milliseconds(0)) {
        // TODO: account for delayed start
        _activeRunJobId = _engine->editScheduler().schedule(std::bind(&BasicBehaviour::runJob, this, nullptr), getInterval());
    }
}

void BasicBehaviour::runJob(void* msg)
{
    // TODO: get rid of msg
    try {
        if (_trace) {
            _trace->setTag("Run", "true");
        }
        run(msg);
    } catch (const std::exception& e) {
        std::string err = std::string("Exception caught:  ") + getName() + std::string(" - ") + std::string(e.what());
        sendLogMessage(4, err);
    }
    _triggeredJobRunning = false;
}

void BasicBehaviour::doTrigger()
{
    if (!_behaviour->isEventDriven() || !isTriggeredRunFinished()) {
        return;
    }
    _triggeredJobRunning = true;
    _engine->editScheduler().schedule(std::bind(&BasicBehaviour::runJob, this, nullptr));
}

void BasicBehaviour::terminateJob()
{
    if (_activeRunJobId != -1) {
        _engine->editScheduler().cancelJob(_activeRunJobId);
        _activeRunJobId = -1;
    }
    // Just to be double safe in terms of the correct behaviour of isSuccess() & isFailure() ensure result is reset before incrementing _execState
    _behResult.store(BehResult::UNKNOWN);
    ++_execState;
    // Intentionally call onTermination() at the end. This prevents setting success/failure from this method
    // TODO: Optimization: don't call onTermination if initiliaseParameters in not called because we were not in context
    try {
        if (_trace) {
            _trace->setTag("Terminate", "true");
            _trace.reset();
        }
        onTermination();
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("[BasicBehaviour] Exception in Behaviour-TERMINATE of: " << getName() << std::endl << e.what());
    }
}

void BasicBehaviour::sendLogMessage(int level, const std::string& message) const
{
    _engine->getCommunicator().sendLogMessage(level, message);
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

void BasicBehaviour::startTrace()
{
    if (!_engine->getTraceFactory() || _tracingDisabled) {
        return;
    }

    BasicPlan* parentPlan;
    do {
        RunningPlan* parent = _context.load()->getParent();
        assert(parent);
        parentPlan = parent->getBasicPlan();
        assert(parentPlan);
    } while (parentPlan->isTracingDisabled());

    _trace = _engine->getTraceFactory()->create(_name, parentPlan->getTrace().context());
}

} /* namespace alica */
