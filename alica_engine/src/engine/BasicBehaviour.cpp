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
        , _signalContext(nullptr)
        , _execContext(nullptr)
        , _signalState(1)
        , _execState(1)
        , _behResult(BehResult::UNKNOWN)
        , _activeRunJobId(-1)
        , _triggeredJobRunning(false)
        , _flags(static_cast<uint8_t>(Flags::TRACING_ENABLED))
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
    // Increment _signalState before setting the signal context, see initJob() for explanation
    ++_signalState;
    _signalContext.store(rp);
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
        _engine->editPlanBase().addFastPathEvent(_execContext.load());
        if (_trace) {
            _trace->setTag("Result", "Success");
        }
    }
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
    ++_execState;

    if (!isExecutingInContext()) {
        return;
    }
    setFlags(Flags::INIT_EXECUTED);

    // There is a possible race condition here in the sense that the _execState can be behind the _signalState
    // and yet this behaviour can execute in the _signalState's RunningPlan context. However this is harmless
    // since all methods are guarded by isExecutingInContext() which will return false in all such cases.
    // Atomically set the signal context to nullptr so that the RunningPlan instance can be deleted
    // when the behaviour is terminated
    _execContext = _signalContext.exchange(nullptr);

    // Get closest parent that has a trace
    if (areFlagsSet(Flags::TRACING_ENABLED)) {
        auto parent = _execContext.load()->getParent();
        for (; parent && !parent->getBasicPlan()->getTraceContext().has_value(); parent = parent->getParent());
        if (parent && _engine->getTraceFactory()) {
            _trace = _engine->getTraceFactory()->create(_name, parent->getBasicPlan()->getTraceContext());
        }
    }

    try {
        if (_trace) {
            _trace->setLog({"Init", "true"});
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
        if (_trace && !areFlagsSet(Flags::RUN_TRACED)) {
            _trace->setLog({"Run", "true"});
            setFlags(Flags::RUN_TRACED);
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
    // There will be no run job if the interval is 0 or init is not executed
    if (_activeRunJobId != -1) {
        _engine->editScheduler().cancelJob(_activeRunJobId);
        _activeRunJobId = -1;
    }
    // Just to be double safe in terms of the correct behaviour of isSuccess() & isFailure() ensure result is reset before incrementing _execState
    _behResult.store(BehResult::UNKNOWN);
    ++_execState;

    if (!areFlagsSet(Flags::INIT_EXECUTED)) {
        // Reset the execution context so that the RunningPlan instance can be deleted
        _execContext.store(nullptr);
        return;
    }
    clearFlags(Flags::INIT_EXECUTED);

    // Intentionally call onTermination() at the end. This prevents setting success/failure from this method
    try {
        if (_trace) {
            _trace->setLog({"Terminate", "true"});
            _trace.reset();
        }
        onTermination();
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("[BasicBehaviour] Exception in Behaviour-TERMINATE of: " << getName() << std::endl << e.what());
    }

    // Reset the execution context so that the RunningPlan instance can be deleted
    _execContext.store(nullptr);
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

std::optional<IAlicaTrace*> BasicBehaviour::getTrace() const
{
    return _trace ? std::optional<IAlicaTrace*>(_trace.get()) : std::nullopt;
}

std::optional<std::string> BasicBehaviour::getTraceContext() const
{
    return _trace ? std::optional<std::string>(_trace->context()) : std::nullopt;
}

} /* namespace alica */
