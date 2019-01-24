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
#include <math.h>

namespace alica
{
/**
 * Constructs the PlanBase given a top-level plan to execute
 * @param masterplan A Plan
 */
PlanBase::PlanBase(AlicaEngine* ae)
        : _ae(ae)
        , _ra(ae->getRoleAssignment())
        , _alicaClock(ae->getAlicaClock())
        , _rootNode(nullptr)
        , _deepestNode(nullptr)
        , _mainThread(nullptr)
        , _statusMessage(nullptr)
        , _stepModeCV()
        , _ruleBook(ae, this)
        , _treeDepth(0)
        , _running(false)
        , _isWaiting(false)

{
    essentials::SystemConfig& sc = essentials::SystemConfig::getInstance();

    double freq = sc["Alica"]->get<double>("Alica.EngineFrequency", NULL);
    double minbcfreq = sc["Alica"]->get<double>("Alica.MinBroadcastFrequency", NULL);
    double maxbcfreq = sc["Alica"]->get<double>("Alica.MaxBroadcastFrequency", NULL);

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

    _sendStatusMessages = sc["Alica"]->get<bool>("Alica.StatusMessages.Enabled", NULL);
    if (_sendStatusMessages) {
        double stfreq = sc["Alica"]->get<double>("Alica.StatusMessages.Frequency", NULL);
        if (stfreq > freq) {
            AlicaEngine::abort("PB: Alica.conf: Status messages frequency must not exceed the engine frequency");
        }

        _sendStatusInterval = AlicaTime::seconds(1.0 / stfreq);
        _statusMessage = new AlicaEngineInfo();
        _statusMessage->senderID = _ae->getTeamManager().getLocalAgentID();
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
void PlanBase::start(const Plan* masterPlan)
{
    if (!_running) {
        _running = true;
        _statusMessage->masterPlan = masterPlan->getName();
        _mainThread = new std::thread(&PlanBase::run, this, masterPlan);
    }
}
/**
 * The Engine's main loop
 */
void PlanBase::run(const Plan* masterPlan)
{
    ALICA_DEBUG_MSG("PB: Run-Method of PlanBase started. ");
    while (_running) {
        AlicaTime beginTime = _alicaClock.now();
        _ae->getLog().itertionStarts();

        if (_ae->getStepEngine()) {
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
                AlicaEngine* ae = _ae;
                _stepModeCV.wait(lckStep, [ae] { return ae->getStepCalled(); });
                _ae->setStepCalled(false);
                _isWaiting = false;
                if (!_running) {
                    return;
                }
            }
            beginTime = _alicaClock.now();
        }

        // Send tick to other modules
        //_ae->getCommunicator().tick(); // not implemented as ros works asynchronous
        _ae->getTeamObserver().tick(_rootNode);
        _ra.tick();
        _ae->getSyncModul().tick();
        _ae->getAuth().tick(_rootNode);

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

        AlicaTime now = _alicaClock.now();

        if (now < _lastSendTime) {
            ALICA_WARNING_MSG("PB: lastSendTime is in the future of the current system time, did the system time change?");
            _lastSendTime = now;
        }

        if ((_ruleBook.hasChangeOccurred() && _lastSendTime + _minSendInterval < now) || _lastSendTime + _maxSendInterval < now) {
            IdGrp msg;
            _deepestNode = _rootNode;
            _treeDepth = 0;
            _rootNode->toMessage(msg, _deepestNode, _treeDepth, 0);
            _ae->getTeamObserver().doBroadCast(msg);
            _lastSendTime = now;
            _ruleBook.resetChangeOccurred();
        }

        if (_sendStatusMessages && _lastSentStatusTime + _sendStatusInterval < _alicaClock.now()) {
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
                auto tmpRole = _ra.getOwnRole();
                if (tmpRole) {
                    _statusMessage->currentRole = _ra.getOwnRole()->getName();
                } else {
                    _statusMessage->currentRole = "No Role";
                }
                _ae->getCommunicator().sendAlicaEngineInfo(*_statusMessage);
                _lastSentStatusTime = _alicaClock.now();
            }
        }

        _ae->getLog().iterationEnds(_rootNode);

        _ae->iterationComplete();

        now = _alicaClock.now();

        AlicaTime availTime = _loopTime - (now - beginTime);
        bool checkFp = false;
        if (availTime > AlicaTime::milliseconds(1)) {
            std::unique_lock<std::mutex> lock(_lomutex);
            checkFp = std::cv_status::no_timeout == _fpEventWait.wait_for(lock, std::chrono::nanoseconds(availTime.inNanoseconds()));
        }

        if (checkFp && _fpEvents.size() > 0) {
            // lock for fpEvents
            std::lock_guard<std::mutex> lock(_lomutex);
            while (_running && availTime > AlicaTime::milliseconds(1) && _fpEvents.size() > 0) {
                RunningPlan* rp = _fpEvents.front();
                _fpEvents.pop();

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
                now = _alicaClock.now();
                availTime = _loopTime - (now - beginTime);
            }
        }

        now = _alicaClock.now();
        availTime = _loopTime - (now - beginTime);

        ALICA_DEBUG_MSG("PB: availTime " << availTime);

        if (availTime > AlicaTime::microseconds(100) && !_ae->getStepEngine()) {
            _alicaClock.sleep(availTime);
        }
    }
}

/**
 * Stops the plan base thread.
 */
void PlanBase::stop()
{
    _running = false;
    _ae->setStepCalled(true);

    if (_ae->getStepEngine()) {
        _ae->setStepCalled(true);
        _stepModeCV.notify_one();
    }

    if (_mainThread != nullptr) {
        _mainThread->join();
        delete _mainThread;
    }
    _mainThread = nullptr;
}

PlanBase::~PlanBase()
{
    delete _statusMessage;
}

void PlanBase::addFastPathEvent(RunningPlan* p)
{
    {
        std::lock_guard<std::mutex> lock(_lomutex);
        _fpEvents.push(p);
    }
    _fpEventWait.notify_all();
}

RunningPlan* PlanBase::makeRunningPlan(const Plan* plan)
{
    _runningPlans.emplace_back(new RunningPlan(_ae, plan));
    return _runningPlans.back().get();
}
RunningPlan* PlanBase::makeRunningPlan(const BehaviourConfiguration* bc)
{
    _runningPlans.emplace_back(new RunningPlan(_ae, bc));
    return _runningPlans.back().get();
}
RunningPlan* PlanBase::makeRunningPlan(const PlanType* pt)
{
    _runningPlans.emplace_back(new RunningPlan(_ae, pt));
    return _runningPlans.back().get();
}

const AlicaTime PlanBase::getloopInterval() const
{
    return _loopInterval;
}

void PlanBase::setLoopInterval(AlicaTime loopInterval)
{
    _loopInterval = loopInterval;
}

std::condition_variable* PlanBase::getStepModeCV()
{
    if (!_ae->getStepEngine()) {
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
