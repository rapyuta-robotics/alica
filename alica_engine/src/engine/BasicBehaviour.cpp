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
        _runThread = new std::thread(&BasicBehaviour::runThread, this, false);
    } else {
        _runThread = new std::thread(&BasicBehaviour::runThread, this, true);
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

bool BasicBehaviour::isTriggeredRunFinished() {
    if (!_behaviourTrigger)
        return false;

    return !_behaviourTrigger->isNotifyCalled(&_runCV);
}

bool BasicBehaviour::doWait()
{
    std::unique_lock<std::mutex> lck(_runLoopMutex);
    _runCV.wait(lck, [this] { return getSignalState() == SignalState::START || isTerminated(); });
    if (isTerminated()) {
        return false;
    }
    setBehaviourResult(BehaviourResult::UNKNOWN);
    setStopCalled(false);
    setBehaviourState(BehaviourState::INITIALIZING);
    return true;
}

void BasicBehaviour::doInit(bool timed)
{
    if (timed && _msDelayedStart > AlicaTime::milliseconds(0)) {
        _engine->getAlicaClock().sleep(_msDelayedStart);
    }
    try {
        initialiseParameters();
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("[BasicBehaviour] Exception in Behaviour-INIT of: " << getName() << std::endl << e.what());
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

void BasicBehaviour::doRun(bool timed)
{
    setBehaviourState(BehaviourState::RUNNING);
    if (timed) {
        while (true) {
            {
                std::unique_lock<std::mutex> lck(_runLoopMutex);
                if (isTerminated() || isStopCalled()) {
                    doStop();
                    return;
                }
            }
            AlicaTime start = _engine->getAlicaClock().now();
            try {
                run(nullptr);
            } catch (const std::exception& e) {
                std::string err = std::string("Exception caught:  ") + getName() + std::string(" - ") + std::string(e.what());
                sendLogMessage(4, err);
            }
            AlicaTime duration = _engine->getAlicaClock().now() - start;
            ALICA_WARNING_MSG_IF(duration > _msInterval + AlicaTime::microseconds(100),
                    "[BasicBehaviour] Behaviour " << _name << "exceeded runtime:  " << duration.inMilliseconds() << "ms!");
            if (duration < _msInterval) {
                _engine->getAlicaClock().sleep(_msInterval - duration);
            }
        }
    } else {
        while (true) {
            {
                std::unique_lock<std::mutex> lck(_runLoopMutex);
                _runCV.wait(lck, [this] { return (_behaviourTrigger && _behaviourTrigger->isNotifyCalled(&_runCV)) || isStopCalled() || isTerminated(); });
                if (isTerminated() || isStopCalled()) {
                    doStop();
                    return;
                }
            }
            try {
                run(static_cast<void*>(_behaviourTrigger));
            } catch (const std::exception& e) {
                std::string err = std::string("Exception caught:  ") + getName() + std::string(" - ") + std::string(e.what());
                sendLogMessage(4, err);
            }
            _behaviourTrigger->setNotifyCalled(&_runCV, false);
        }
    }
}

void BasicBehaviour::runThread(bool timed)
{
    while (doWait()) {
        doInit(timed);
        doRun(timed);
        onTermination();
        setBehaviourState(BehaviourState::UNINITIALIZED);
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

} /* namespace alica */
