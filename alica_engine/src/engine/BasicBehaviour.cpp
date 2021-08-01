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
#include "engine/teammanager/TeamManager.h"
#include "engine/scheduler/Scheduler.h"

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
    if (_behaviour->isEventDriven()) {
        //        _runThread = new std::thread(&BasicBehaviour::runThread, this, false);
    } else {
        //        _runThread = new std::thread(&BasicBehaviour::runThread, this, true);
    }
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
    {
        std::lock_guard<std::mutex> lck(_runLoopMutex);
        setSignalState(SignalState::STOP);
        setStopCalled(true);
    }
    if (_behaviour->isEventDriven()) {
        _runCV.notify_all();
    }
    return true;
}

/**
 * Starts the execution of this BasicBehaviour.
 */
bool BasicBehaviour::start()
{
    {
        std::lock_guard<std::mutex> lck(_runLoopMutex);
        setSignalState(SignalState::START);
    }

    if (!_behaviour->isEventDriven()) {
        _runCV.notify_all();
    }
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
    std::lock_guard<std::mutex> lockGuard(_runLoopMutex);
    return !_runJob.lock();
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
        std::vector<std::weak_ptr<scheduler::Job>> runJobPrerequisites;

        std::function<void()> runCb = std::bind(&BasicBehaviour::doRun, this, nullptr);
        std::shared_ptr<scheduler::Job> runJob = std::make_shared<scheduler::Job>(scheduler.getNextJobID(), runCb, runJobPrerequisites);
        _activeRunJobId = runJob->id;
        runJob->isRepeated = true;
        runJob->repeatInterval = getInterval();
        _runJob = runJob; //store runJob as weak_ptr
        scheduler.schedule(std::move(runJob));
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
}

void BasicBehaviour::doTrigger()
{
    if (!_behaviour->isEventDriven() && !_runJob.lock()) {
        return;
    }

    auto& scheduler = _engine->editScheduler();

    std::vector<std::weak_ptr<scheduler::Job>> runJobPrerequisites;
    runJobPrerequisites.push_back(_context->getInitJob());

    std::function<void()> runCb = std::bind(&BasicBehaviour::doRun, this, nullptr);
    std::shared_ptr<scheduler::Job> runJob = std::make_shared<scheduler::Job>(scheduler.getNextJobID(), runCb, runJobPrerequisites);
    runJob->isRepeated = false;
    _runJob = runJob; // store runJob as weak_ptr
    scheduler.schedule(std::move(runJob));
}

void BasicBehaviour::doTermination()
{
    auto& scheduler = _engine->editScheduler();
    scheduler.stopJob(_activeRunJobId);
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
