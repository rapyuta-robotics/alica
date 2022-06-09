#include "engine/PlanBase.h"
#include <engine/syncmodule/SyncModule.h>

#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/IAlicaCommunication.h"
#include "engine/IRoleAssignment.h"
#include "engine/Logger.h"
#include "engine/RuleBook.h"
#include "engine/RunningPlan.h"
#include "engine/TeamObserver.h"
#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include "engine/model/Task.h"
#include "engine/teammanager/TeamManager.h"

#include <alica_common_config/debug_output.h>
#include <functional>
#include <math.h>

namespace alica
{
/**
 * Constructs the PlanBase given a top-level plan to execute
 * @param masterplan A Plan
 */
PlanBase::PlanBase(ConfigChangeListener& configChangeListener, const AlicaClock& clock, Logger& log, const IAlicaCommunication& communicator,
        IRoleAssignment& roleAssignment, SyncModule& synchModule, AuthorityManager& authorityManager, TeamObserver& teamObserver, TeamManager& teamManager,
        const PlanRepository& planRepository, bool& stepEngine, bool& stepCalled, IAlicaWorldModel* worldModel, const RuntimePlanFactory& runTimePlanFactory)
        : _configChangeListener(configChangeListener)
        , _clock(clock)
        , _logger(log)
        , _communicator(communicator)
        , _roleAssignment(roleAssignment)
        , _synchModule(synchModule)
        , _authorityManager(authorityManager)
        , _teamObserver(teamObserver)
        , _teamManager(teamManager)
        , _planRepository(planRepository)
        , _stepEngine(stepEngine)
        , _stepCalled(stepCalled)
        , _worldModel(worldModel)
        , _runTimePlanFactory(runTimePlanFactory)
        , _rootNode(nullptr)
        , _deepestNode(nullptr)
        , _mainThread(nullptr)
        , _statusMessage(nullptr)
        , _stepModeCV()
        , _ruleBook(configChangeListener, log, synchModule, teamObserver, teamManager, planRepository, this)
        , _treeDepth(0)
        , _running(false)
        , _isWaiting(false)

{
    auto reloadFunctionPtr = std::bind(&PlanBase::reload, this, std::placeholders::_1);
    _configChangeListener.subscribe(reloadFunctionPtr);
    reload(_configChangeListener.getConfig());
}

void PlanBase::reload(const YAML::Node& config)
{
    double freq = config["Alica"]["EngineFrequency"].as<double>();
    double minbcfreq = config["Alica"]["MinBroadcastFrequency"].as<double>();
    double maxbcfreq = config["Alica"]["MaxBroadcastFrequency"].as<double>();

    if (freq > 1000) {
        AlicaEngine::abort("PB: ALICA should not be used with more than 1000Hz");
    }

    if (maxbcfreq > freq) {
        AlicaEngine::abort("PB: Alica.conf: Maximum broadcast frequency must not exceed the engine frequency");
    }

    if (minbcfreq > maxbcfreq) {
        AlicaEngine::abort("PB: Alica.conf: Minimal broadcast frequency must be lower or equal to maximal broadcast frequency!");
    }

    _loopTime = AlicaTime::seconds(1.0 / freq);
    _minSendInterval = AlicaTime::seconds(1.0 / maxbcfreq);
    _maxSendInterval = AlicaTime::seconds(1.0 / minbcfreq);

    AlicaTime halfLoopTime = _loopTime / 2;

    _sendStatusMessages = config["Alica"]["StatusMessages"]["Enabled"].as<bool>();
    if (_sendStatusMessages) {
        double stfreq = config["Alica"]["StatusMessages"]["Frequency"].as<double>();
        if (stfreq > freq) {
            AlicaEngine::abort("PB: Alica.conf: Status messages frequency must not exceed the engine frequency");
        }

        _sendStatusInterval = AlicaTime::seconds(1.0 / stfreq);
        if (!_statusMessage) {
            _statusMessage = new AlicaEngineInfo();
        }
    }

    ALICA_INFO_MSG("PB: Engine loop time is " << _loopTime.inMilliseconds() << "ms, broadcast interval is " << _minSendInterval.inMilliseconds() << "ms - "
                                              << _maxSendInterval.inMilliseconds() << "ms");

    if (halfLoopTime < _minSendInterval) {
        _minSendInterval -= halfLoopTime;
        _maxSendInterval -= halfLoopTime;
    }
}

/**
 * Starts execution of the plan tree, call once all necessary modules are initialised.
 */
void PlanBase::start(const Plan* masterPlan, const IAlicaWorldModel* wm)
{
    _ruleBook.init(wm);
    if (!_running) {
        _running = true;
        if (_statusMessage) {
            _statusMessage->senderID = _teamManager.getLocalAgentID();
            _statusMessage->masterPlan = masterPlan->getName();
        }
        _mainThread = new std::thread(&PlanBase::run, this, masterPlan);
    }
}

/**
 * The Engine's main loop
 */
void PlanBase::run(const Plan* masterPlan)
{
    ALICA_DEBUG_MSG("PB: Run-Method of PlanBase started. ");
    Logger& log = _logger;

    while (_running) {
        AlicaTime beginTime = _clock.now();
        log.itertionStarts();

        if (_stepEngine) {
#ifdef ALICA_DEBUG_ENABLED
            ALICA_DEBUG_MSG("PB: ===CUR TREE===");
            if (_rootNode == nullptr) {
                ALICA_DEBUG_MSG("PB: NULL");
            } else {
                _rootNode->printRecursive();
            }
            ALICA_DEBUG_MSG("PB: ===END CUR TREE===");
#endif
            {
                std::unique_lock<std::mutex> lckStep(_stepMutex);
                _isWaiting = true;
                _stepModeCV.wait(lckStep, [&] { return _stepCalled; });
                _stepCalled = false;
                _isWaiting = false;
                if (!_running) {
                    return;
                }
            }
            beginTime = _clock.now();
        }

        // Send tick to other modules
        //_ae->getCommunicator().tick(); // not implemented as ros works asynchronous
        _teamObserver.tick(_rootNode);
        _roleAssignment.tick();
        _synchModule.tick();
        _authorityManager.tick(_rootNode);
        _teamManager.tick();

        if (_rootNode == nullptr) {
            _rootNode = _ruleBook.initialisationRule(masterPlan);
        }
        _rootNode->preTick();
        if (_rootNode->tick(&_ruleBook) == PlanChange::FailChange) {
            ALICA_INFO_MSG("PB: MasterPlan Failed");
        }
        // clear deepest node pointer before deleting plans:
        if (_deepestNode && _deepestNode->isRetired()) {
            _deepestNode = nullptr;
        }
        // remove deletable plans:
        // this should be done just before clearing fpEvents, to make sure no spurious pointers remain
#ifdef ALICA_DEBUG_ENABLED
        int retiredCount = 0;
        int inActiveCount = 0;
        int deleteCount = 0;
        int totalCount = static_cast<int>(_runningPlans.size());
#endif
        for (int i = static_cast<int>(_runningPlans.size()) - 1; i >= 0; --i) {
#ifdef ALICA_DEBUG_ENABLED
            if (_runningPlans[i]->isRetired()) {
                ++retiredCount;
            } else if (!_runningPlans[i]->isActive()) {
                ++inActiveCount;
            }
#endif
            if (_runningPlans[i]->isDeleteable()) {
                assert(_runningPlans[i].use_count() == 1);
                _runningPlans.erase(_runningPlans.begin() + i);
#ifdef ALICA_DEBUG_ENABLED
                ++deleteCount;
#endif
            }
        }
#ifdef ALICA_DEBUG_ENABLED
        ALICA_DEBUG_MSG("PlanBase: " << (totalCount - inActiveCount - retiredCount) << " active " << retiredCount << " retired " << inActiveCount
                                     << " inactive deleted: " << deleteCount);
#endif
        // lock for fpEvents
        {
            std::lock_guard<std::mutex> lock(_lomutex);
            _fpEvents = std::queue<RunningPlan*>();
        }

        AlicaTime now = _clock.now();

        if (now < _lastSendTime) {
            ALICA_WARNING_MSG("PB: lastSendTime is in the future of the current system time, did the system time change?");
            _lastSendTime = now;
        }

        if ((_ruleBook.hasChangeOccurred() && _lastSendTime + _minSendInterval < now) || _lastSendTime + _maxSendInterval < now) {
            IdGrp msg;
            _deepestNode = _rootNode;
            _treeDepth = 0;
            _rootNode->toMessage(msg, _deepestNode, _treeDepth, 0);
            _teamObserver.doBroadCast(msg);
            _lastSendTime = now;
            _ruleBook.resetChangeOccurred();
        }

        if (_sendStatusMessages && _lastSentStatusTime + _sendStatusInterval < _clock.now()) {
            if (_deepestNode != nullptr) {
                _statusMessage->robotIDsWithMe.clear();
                _statusMessage->currentPlan = _deepestNode->getActivePlan()->getName();
                if (_deepestNode->getActiveEntryPoint() != nullptr) {
                    _statusMessage->currentTask = _deepestNode->getActiveEntryPoint()->getTask()->getName();
                } else {
                    _statusMessage->currentTask = "IDLE";
                }
                if (_deepestNode->getActiveState() != nullptr) {
                    _statusMessage->currentState = _deepestNode->getActiveState()->getName();
                    _deepestNode->getAssignment().getAgentsInState(_deepestNode->getActiveState(), _statusMessage->robotIDsWithMe);
                } else {
                    _statusMessage->currentState = "NONE";
                }
                auto tmpRole = _roleAssignment.getOwnRole();
                if (tmpRole) {
                    _statusMessage->currentRole = _roleAssignment.getOwnRole()->getName();
                } else {
                    _statusMessage->currentRole = "No Role";
                }
                _communicator.sendAlicaEngineInfo(*_statusMessage);
                _lastSentStatusTime = _clock.now();
            }
        }

        log.iterationEnds(_rootNode);

        //_ae->iterationComplete();

        now = _clock.now();

        AlicaTime availTime = _loopTime - (now - beginTime);
        bool checkFp = false;
        if (availTime > AlicaTime::milliseconds(1)) {
            std::unique_lock<std::mutex> lock(_lomutex);
            checkFp = std::cv_status::no_timeout == _fpEventWait.wait_for(lock, std::chrono::nanoseconds(availTime.inNanoseconds()));
        }

        if (checkFp) {
            std::queue<RunningPlan*> nextFpEvents;
            {
                // move fath path events to a local variable. Prevents calling visit() on a RunningPlan which can add a fast path event double locking _loMutex
                std::lock_guard<std::mutex> lock(_lomutex);
                nextFpEvents.swap(_fpEvents);
            }

            while (_running && availTime > AlicaTime::milliseconds(1) && !nextFpEvents.empty()) {
                RunningPlan* rp = nextFpEvents.front();
                nextFpEvents.pop();

                if (rp->isActive()) {
                    bool first = true;
                    while (rp != nullptr) {
                        PlanChange change = _ruleBook.visit(*rp);
                        if (!first && change == PlanChange::NoChange) {
                            break;
                        }
                        rp = rp->getParent();
                        first = false;
                    }
                }
                now = _clock.now();
                availTime = _loopTime - (now - beginTime);
            }

            {
                // if all fast path events could not be processed, prepend them back to _fpEvents
                if (!nextFpEvents.empty()) {
                    std::lock_guard<std::mutex> lock(_lomutex);
                    _fpEvents.swap(nextFpEvents);
                    while (!nextFpEvents.empty()) {
                        _fpEvents.push(nextFpEvents.front());
                        nextFpEvents.pop();
                    }
                }
            }
        }

        now = _clock.now();
        availTime = _loopTime - (now - beginTime);

        ALICA_DEBUG_MSG("PB: availTime " << availTime);

        if (availTime > AlicaTime::microseconds(100) && !_stepEngine) {
            _clock.sleep(availTime);
        }
    }
}

/**
 * Stops the plan base thread.
 */
void PlanBase::stop()
{
    _running = false;
    _stepCalled = true;

    if (_stepEngine) {
        _stepCalled = true;
        _stepModeCV.notify_one();
    }

    if (_mainThread != nullptr) {
        _mainThread->join();
        delete _mainThread;
    }

    if (_rootNode) {
        _rootNode->deactivate();
    }

    _mainThread = nullptr;
}

PlanBase::~PlanBase()
{
    delete _statusMessage;
    // Destroy running plans from most recent to least recent
    while (!_runningPlans.empty()) {
        _runningPlans.pop_back();
    }
}

void PlanBase::addFastPathEvent(RunningPlan* p)
{
    {
        std::lock_guard<std::mutex> lock(_lomutex);
        _fpEvents.push(p);
    }
    _fpEventWait.notify_all();
}

RunningPlan* PlanBase::makeRunningPlan(const Plan* plan, const Configuration* configuration)
{
    _runningPlans.emplace_back(new RunningPlan(
            _configChangeListener, _clock, _worldModel, _runTimePlanFactory, _teamObserver, _teamManager, _planRepository, plan, configuration));
    return _runningPlans.back().get();
}
RunningPlan* PlanBase::makeRunningPlan(const Behaviour* b, const Configuration* configuration)
{
    _runningPlans.emplace_back(
            new RunningPlan(_configChangeListener, _clock, _worldModel, _runTimePlanFactory, _teamObserver, _teamManager, _planRepository, b, configuration));
    return _runningPlans.back().get();
}
RunningPlan* PlanBase::makeRunningPlan(const PlanType* pt, const Configuration* configuration)
{
    _runningPlans.emplace_back(
            new RunningPlan(_configChangeListener, _clock, _worldModel, _runTimePlanFactory, _teamObserver, _teamManager, _planRepository, pt, configuration));
    return _runningPlans.back().get();
}

const AlicaTime PlanBase::getLoopInterval() const
{
    return _loopInterval;
}

void PlanBase::setLoopInterval(AlicaTime loopInterval)
{
    _loopInterval = loopInterval;
}

std::condition_variable* PlanBase::getStepModeCV()
{
    if (!_stepEngine) {
        return nullptr;
    }
    return &_stepModeCV;
}

/**
 * Returns the deepest ALICA node
 */
const RunningPlan* PlanBase::getDeepestNode() const
{
    return _deepestNode;
}

} // namespace alica
