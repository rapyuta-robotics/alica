#include "engine/PlanBase.h"
#include <engine/syncmodule/SyncModule.h>

#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/ConfigChangeListener.h"
#include "engine/IAlicaCommunication.h"
#include "engine/IRoleAssignment.h"
#include "engine/RuleBook.h"
#include "engine/RunningPlan.h"
#include "engine/TeamObserver.h"
#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/logging/Logging.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include "engine/model/Task.h"
#include "engine/teammanager/TeamManager.h"

#include <functional>
#include <math.h>

#include <string>

namespace alica
{
/**
 * Constructs the PlanBase given a top-level plan to execute
 * @param masterplan A Plan
 */
PlanBase::PlanBase(ConfigChangeListener& configChangeListener, const AlicaClock& clock, const IAlicaCommunication& communicator,
        IRoleAssignment& roleAssignment, SyncModule& syncModule, AuthorityManager& authorityManager, TeamObserver& teamObserver, TeamManager& teamManager,
        const PlanRepository& planRepository, std::atomic<bool>& stepEngine, std::atomic<bool>& stepCalled, std::shared_ptr<Blackboard> globalBlackboard,
        VariableSyncModule& resultStore, const std::unordered_map<size_t, std::unique_ptr<ISolverBase>>& solvers, const IAlicaTimerFactory& timerFactory,
        const IAlicaTraceFactory* traceFactory)
        : _configChangeListener(configChangeListener)
        , _clock(clock)
        , _communicator(communicator)
        , _roleAssignment(roleAssignment)
        , _syncModule(syncModule)
        , _authorityManager(authorityManager)
        , _teamObserver(teamObserver)
        , _teamManager(teamManager)
        , _planRepository(planRepository)
        , _stepEngine(stepEngine)
        , _stepCalled(stepCalled)
        , _globalBlackboard(globalBlackboard)
        , _runTimePlanFactory(globalBlackboard, traceFactory, teamManager, timerFactory)
        , _runTimeBehaviourFactory(globalBlackboard, teamManager, *this, communicator, traceFactory, timerFactory)
        , _resultStore(resultStore)
        , _solvers(solvers)
        , _rootNode(nullptr)
        , _deepestNode(nullptr)
        , _mainThread(nullptr)
        , _statusMessage(nullptr)
        , _stepModeCV()
        , _ruleBook(configChangeListener, syncModule, teamObserver, teamManager, planRepository, this)
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
        AlicaEngine::abort(LOGNAME, "Alica.conf: engine frequency should not be more than 1000Hz");
    }

    if (maxbcfreq > freq) {
        AlicaEngine::abort(LOGNAME, "Alica.conf: Maximum broadcast frequency must not exceed the engine frequency");
    }

    if (minbcfreq > maxbcfreq) {
        AlicaEngine::abort(LOGNAME, "Alica.conf: Minimal broadcast frequency must be lower or equal to maximal broadcast frequency!");
    }

    _loopTime = AlicaTime::seconds(1.0 / freq);
    _minSendInterval = AlicaTime::seconds(1.0 / maxbcfreq);
    _maxSendInterval = AlicaTime::seconds(1.0 / minbcfreq);

    AlicaTime halfLoopTime = _loopTime / 2;

    _sendStatusMessages = config["Alica"]["StatusMessages"]["Enabled"].as<bool>();
    if (_sendStatusMessages) {
        double stfreq = config["Alica"]["StatusMessages"]["Frequency"].as<double>();
        if (stfreq > freq) {
            AlicaEngine::abort(LOGNAME, "Alica.conf: Status messages frequency must not exceed the engine frequency");
        }

        _sendStatusInterval = AlicaTime::seconds(1.0 / stfreq);
        if (!_statusMessage) {
            _statusMessage = std::make_unique<AlicaEngineInfo>();
        }
    }

    Logging::logInfo(LOGNAME) << "Engine loop time is " << _loopTime.inMilliseconds() << "ms, broadcast interval is " << _minSendInterval.inMilliseconds()
                              << "ms - " << _maxSendInterval.inMilliseconds() << "ms";

    if (halfLoopTime < _minSendInterval) {
        _minSendInterval -= halfLoopTime;
        _maxSendInterval -= halfLoopTime;
    }
}

/**
 * Starts execution of the plan tree, call once all necessary modules are initialised.
 */
void PlanBase::start(const Plan* masterPlan)
{
    _ruleBook.init(_globalBlackboard);
    if (!_running) {
        _running = true;
        if (_statusMessage) {
            _statusMessage->senderID = _teamManager.getLocalAgentID();
            _statusMessage->masterPlan = masterPlan->getName();
        }
        _mainThread = std::make_unique<std::thread>(&PlanBase::run, this, masterPlan);
    }
}

/**
 * The Engine's main loop
 */
void PlanBase::run(const Plan* masterPlan)
{
    Logging::logDebug(LOGNAME) << "Run-Method of PlanBase started.";

    while (_running) {
        AlicaTime beginTime = _clock.now();

        if (_stepEngine) {
#ifdef ALICA_DEBUG_ENABLED
            Logging::logDebug(LOGNAME) << "===CUR TREE===";
            if (_rootNode == nullptr) {
                Logging::logDebug(LOGNAME) << "NULL";
            } else {
                _rootNode->printRecursive();
            }
            Logging::logDebug(LOGNAME) << "===END CUR TREE===";
#endif
            {
                std::unique_lock<std::mutex> lckStep(_stepMutex);
                _isWaiting = true;
                _stepModeCV.wait(lckStep, [&] { return _stepCalled.load(); });
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
        _syncModule.tick();
        _authorityManager.tick(_rootNode);
        _teamManager.tick();

        if (_rootNode == nullptr) {
            _rootNode = _ruleBook.initialisationRule(masterPlan);
        }
        _rootNode->preTick();
        if (_rootNode->tick(&_ruleBook) == PlanChange::FailChange) {
            Logging::logInfo(LOGNAME) << "MasterPlan Failed";
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
        Logging::logDebug(LOGNAME) << (totalCount - inActiveCount - retiredCount) << " active " << retiredCount << " retired " << inActiveCount
                                   << " inactive deleted: " << deleteCount;
#endif
        // lock for fpEvents
        {
            std::lock_guard<std::mutex> lock(_lomutex);
            _fpEvents = std::queue<RunningPlan*>();
        }

        AlicaTime now = _clock.now();

        if (now < _lastSendTime) {
            Logging::logWarn(LOGNAME) << "lastSendTime is in the future of the current system time, did the system time change?";
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

        //_ae->iterationComplete(); TODO modify when AlicaEngine::iterationComplete will be written

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
                // move fast path events to a local variable. Prevents calling visit() on a RunningPlan which can add a fast path event double locking
                // _loMutex
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

        if (_running && availTime > AlicaTime::microseconds(100) && !_stepEngine) {
            _clock.sleep(availTime);
        }
    }
}

/**
 * Stops the plan base thread.
 */
void PlanBase::stop()
{
    _stepCalled = true;
    _running = false;
    _stepModeCV.notify_one();

    if (_stepEngine) {
        _stepCalled = true;
        _stepModeCV.notify_one();
    }

    if (_mainThread != nullptr) {
        _mainThread->join();
        _mainThread.reset();
    }

    if (_rootNode) {
        _rootNode->deactivate();
    }
}

PlanBase::~PlanBase()
{
    if (_running) {
        stop();
    }
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

RunningPlan* PlanBase::makeRunningPlan(const AbstractPlan* abstractPlan, const ConfAbstractPlanWrapper* wrapper)
{
    _runningPlans.emplace_back(std::make_shared<RunningPlan>(_configChangeListener, _clock, _globalBlackboard, _teamObserver, _teamManager, _planRepository,
            _resultStore, _solvers, _runTimePlanFactory, _runTimeBehaviourFactory, abstractPlan, wrapper));
    return _runningPlans.back().get();
}

void PlanBase::init(std::unique_ptr<IBehaviourCreator>&& behaviourCreator, std::unique_ptr<IPlanCreator>&& planCreator)
{
    // due communication could be already started, this factories should handle concurrent access properly
    _runTimeBehaviourFactory.init(std::move(behaviourCreator));
    _runTimePlanFactory.init(std::move(planCreator));
}

const AlicaTime PlanBase::getLoopInterval() const
{
    return _loopInterval;
}

void PlanBase::setLoopInterval(AlicaTime loopInterval)
{
    _loopInterval = loopInterval;
}

/**
 * Returns the deepest ALICA node
 */
const RunningPlan* PlanBase::getDeepestNode() const
{
    return _deepestNode;
}

void PlanBase::stepNotify()
{
    _stepCalled = true;
    if (!_stepEngine) {
        return;
    }
    _stepModeCV.notify_all();
}

} // namespace alica
