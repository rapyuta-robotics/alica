#define BEH_DEBUG

#include "engine/BasicBehaviour.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanBase.h"
#include "engine/TeamObserver.h"
#include "engine/model/Behaviour.h"
#include "engine/model/BehaviourConfiguration.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/Task.h"
#include "engine/model/Variable.h"
#include "engine/teammanager/TeamManager.h"

#include <alica_common_config/debug_output.h>

#include <supplementary/ITrigger.h>
#include <supplementary/Timer.h>

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
        : name(name)
        , _engine(nullptr)
        , failure(false)
        , success(false)
        , callInit(true)
        , started(true)
        , _configuration(nullptr)
        , msInterval(100)
        , msDelayedStart(0)
        , running(false)
        , _inRun(false)
        , behaviourTrigger(nullptr)
{
    this->timer = new supplementary::Timer(0, 0);
    this->timer->registerCV(&this->runCV);
    this->runThread = new std::thread(&BasicBehaviour::runInternal, this);
}

BasicBehaviour::~BasicBehaviour()
{
    this->started = false;
    this->runCV.notify_all();
    this->timer->start();
    this->runThread->join();
    delete this->runThread;
    delete this->timer;
    if (behaviourTrigger != nullptr) {
        delete this->behaviourTrigger;
    }
}

bool BasicBehaviour::isProperlyStopped() const
{
    return !running && !timer->isRunning() && !_inRun;
}

void BasicBehaviour::setName(const std::string& name)
{
    this->name = name;
}

void BasicBehaviour::setConfiguration(const BehaviourConfiguration* beh)
{
    _configuration = beh;
}

const Variable* BasicBehaviour::getVariableByName(const std::string& name) const
{
    return _configuration->getVariableByName(name);
}

int BasicBehaviour::getDelayedStart() const
{
    return this->timer->getDelayedStart();
}

void BasicBehaviour::setDelayedStart(long msDelayedStart)
{
    this->timer->setDelayedStart(msDelayedStart);
}

int BasicBehaviour::getInterval() const
{
    return this->timer->getInterval();
}

void BasicBehaviour::setInterval(long msInterval)
{
    this->timer->setInterval(msInterval);
}

/**
 * Convenience method to obtain the robot's own id.
 * @return the own robot id
 */
AgentIDConstPtr BasicBehaviour::getOwnId() const
{
    return _engine->getTeamManager()->getLocalAgentID();
}

/**
 * Stops the execution of this BasicBehaviour.
 */
bool BasicBehaviour::stop()
{
    this->success = false;
    this->failure = false;
    if (behaviourTrigger == nullptr) {
        return this->timer->stop();
    } else {
        this->running = false;
    }
    return true;
}

/**
 * Starts the execution of this BasicBehaviour.
 */
bool BasicBehaviour::start()
{
    this->callInit = true;
    if (behaviourTrigger == nullptr) {
        return this->timer->start();
    } else {
        this->running = true;
    }
    return true;
}

void BasicBehaviour::setSuccess(bool success)
{
    if (!this->success && success) {
        _engine->getPlanBase()->addFastPathEvent(_context);
    }
    this->success = success;
}

bool BasicBehaviour::isSuccess() const
{
    return success && !this->callInit;
}

void BasicBehaviour::setFailure(bool failure)
{
    if (!this->failure && failure) {
        _engine->getPlanBase()->addFastPathEvent(_context);
    }
    this->failure = failure;
}

bool BasicBehaviour::isFailure() const
{
    return failure && !this->callInit;
}

void BasicBehaviour::setTrigger(supplementary::ITrigger* trigger)
{
    this->behaviourTrigger = trigger;
    this->behaviourTrigger->registerCV(&this->runCV);
}

void BasicBehaviour::initInternal()
{
    this->success = false;
    this->failure = false;
    this->callInit = false;
    try {
        initialiseParameters();
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("BB: Exception in Behaviour-INIT of: " << getName() << std::endl << e.what());
    }
}

bool BasicBehaviour::getParameter(const std::string& key, std::string& valueOut) const
{
    for (const auto& pair : _configuration->getParameters()) {
        if (pair.first == key) {
            valueOut = pair.second;
            return true;
        }
    }
    valueOut = "";
    return false;
}

void BasicBehaviour::runInternal()
{
    std::unique_lock<std::mutex> lck(runCV_mtx);
    while (this->started) {
        _inRun = false;
        this->runCV.wait(lck, [&] {
            if (behaviourTrigger == nullptr) {
                return !this->started || this->timer->isNotifyCalled(&runCV);
            } else {
                return !this->started || (this->behaviourTrigger->isNotifyCalled(&runCV) && this->running);
            }
        }); // protection against spurious wake-ups
        _inRun = true;
        if (!this->started) {
            return;
        }

        if (this->callInit) {
            this->initInternal();
        }
#ifdef BEH_DEBUG
        std::chrono::system_clock::time_point start = std::chrono::high_resolution_clock::now();
#endif
        // TODO: pass something like an eventarg (to be implemented) class-member, which could be set for an event
        // triggered (to be implemented) behaviour.
        try {
            if (behaviourTrigger == nullptr) {
                this->run((void*) timer);
            } else {
                this->run((void*) behaviourTrigger);
            }
        } catch (const std::exception& e) {
            std::string err = std::string("Exception caught:  ") + getName() + std::string(" - ") + std::string(e.what());
            sendLogMessage(4, err);
        }
#ifdef BEH_DEBUG
        const BehaviourConfiguration* conf = static_cast<const BehaviourConfiguration*>(PlanInterface(_context).getActivePlan());
        if (!conf->isEventDriven()) {
            double dura = (std::chrono::high_resolution_clock::now() - start).count() / 1000000.0 - 1.0 / conf->getFrequency() * 1000.0;
            if (dura > 0.1) {
                std::stringstream ss;
                ss << "BB: Behaviour " << conf->getBehaviour()->getName() << " exceeded runtime by \t" << dura << "ms!";
                sendLogMessage(2, ss.str());
            }
        }
#endif
        if (behaviourTrigger == nullptr) {
            this->timer->setNotifyCalled(false, &runCV);
        } else {
            this->behaviourTrigger->setNotifyCalled(false, &runCV);
        }
    }
}

void BasicBehaviour::sendLogMessage(int level, const std::string& message) const
{
    _engine->getCommunicator()->sendLogMessage(level, message);
}

} /* namespace alica */
