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
 * If using eventTrigger set behaviourTrigger and register runCV
 * @param name The name of the behaviour
 */
BasicBehaviour::BasicBehaviour(const std::string& name)
        : _name(name)
        , _engine(nullptr)
        , _behaviour(nullptr)
        , _contextInRun(nullptr)
        , _configuration(nullptr)
        , _msInterval(AlicaTime::milliseconds(100))
        , _msDelayedStart(AlicaTime::milliseconds(0))
        , _signalState(SignalState::STOP)
        , _stopCalled(false)
        , _behaviourResult(BehaviourResult::UNKNOWN)
        , _behaviourState(BehaviourState::UNINITIALIZED)
        , _behaviourTrigger(nullptr)
        , _runThread(nullptr)
        , _context(nullptr)
        , _activeRunJobId(-1)
        , _triggeredJobRunning(false)
{
}

void BasicBehaviour::terminate()
{
    {
        std::lock_guard<std::mutex> lck(_runLoopMutex);
        setSignalState(SignalState::TERMINATE);
    }
    _runCV.notify_all();
    if (_runThread) {
        _runThread->join();
        delete _runThread;
    }
}

bool BasicBehaviour::isRunningInContext(const RunningPlan* rp) const
{
    return _context == rp && (getSignalState() == SignalState::START || isBehaviourStarted());
}

void BasicBehaviour::setBehaviour(const Behaviour* beh)
{
    assert(_behaviour == nullptr);
    _behaviour = beh;
}

void BasicBehaviour::setConfiguration(const Configuration* conf)
{
    assert(_configuration == nullptr);
    _configuration = conf;
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
    if (!isStopCalled()) {
        _engine->editScheduler().schedule(std::bind(&BasicBehaviour::doTermination, this));
    }

    setSignalState(SignalState::STOP);
    setStopCalled(true);
    return true;
}

/**
 * Starts the execution of this BasicBehaviour.
 */
bool BasicBehaviour::start()
{
    setSignalState(SignalState::START);
    _engine->editScheduler().schedule(std::bind(&BasicBehaviour::doInit, this));
    return true;
}

void BasicBehaviour::setSuccess()
{
    if (getBehaviourResult() != BehaviourResult::SUCCESS && getBehaviourState() == BehaviourState::RUNNING) {
        setBehaviourResult(BehaviourResult::SUCCESS);
        _engine->editPlanBase().addFastPathEvent(_context);
    }
}

bool BasicBehaviour::isSuccess() const
{
    // Check for isStopCalled() before checking the behaviour result
    return !isStopCalled() && getBehaviourResult() == BehaviourResult::SUCCESS;
}

void BasicBehaviour::setFailure()
{
    if (getBehaviourResult() != BehaviourResult::FAILURE && getBehaviourState() == BehaviourState::RUNNING) {
        setBehaviourResult(BehaviourResult::FAILURE);
        _engine->editPlanBase().addFastPathEvent(_context);
    }
}

bool BasicBehaviour::isFailure() const
{
    // Check for isStopCalled() before checking the behaviour result
    return !isStopCalled() && getBehaviourResult() == BehaviourResult::FAILURE;
}

void BasicBehaviour::setTrigger(essentials::ITrigger* trigger)
{
    if (!trigger)
        return;

    std::lock_guard<std::mutex> lockGuard(_runLoopMutex);
    _behaviourTrigger = trigger;
    _behaviourTrigger->registerCV(&_runCV);
}

bool BasicBehaviour::isTriggeredRunFinished()
{
    return !_triggeredJobRunning;
}

void BasicBehaviour::doInit()
{
    setBehaviourResult(BehaviourResult::UNKNOWN);
    setStopCalled(false);
    setBehaviourState(BehaviourState::INITIALIZING);

    try {
        initialiseParameters();
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("[BasicBehaviour] Exception in Behaviour-INIT of: " << getName() << std::endl << e.what());
    }

    // Do not schedule repeatable run job when behaviour is event driven.
    if (!isEventDriven()) {
        auto& scheduler = _engine->editScheduler();
        std::function<void()> runCb = std::bind(&BasicBehaviour::doRun, this, nullptr);
        _activeRunJobId = scheduler.schedule(std::move(runCb), getInterval());
    }
}

void BasicBehaviour::doStop()
{
    // important to set stopCalled to false after setting behaviour result for correct lock-free
    // behaviour of isSuccess() & isFailure(). Should be called when _runLoopMutex is held by the thread
    setBehaviourResult(BehaviourResult::UNKNOWN);
    setBehaviourState(BehaviourState::TERMINATING);
    setStopCalled(false);
}

void BasicBehaviour::doRun(void* msg)
{
    setBehaviourState(BehaviourState::RUNNING);
    run(msg);
    _triggeredJobRunning = false;
}

void BasicBehaviour::doTrigger()
{
    if (!_behaviour->isEventDriven() || !isTriggeredRunFinished()) {
        return;
    }
    _triggeredJobRunning = true;
    _engine->editScheduler().schedule(std::bind(&BasicBehaviour::doRun, this, nullptr));
}

void BasicBehaviour::doTermination()
{
    _engine->editScheduler().stopJob(_activeRunJobId);
    onTermination();
    setBehaviourState(BehaviourState::UNINITIALIZED);
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

} /* namespace alica */
