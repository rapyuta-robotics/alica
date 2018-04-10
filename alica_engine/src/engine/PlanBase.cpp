#include <engine/syncmodule/SyncModule.h>
#include "engine/PlanBase.h"

#include "engine/RunningPlan.h"
#include "engine/model/Plan.h"
#include "engine/RuleBook.h"
#include "engine/AlicaEngine.h"
#include "engine/TeamObserver.h"
#include "engine/teammanager/TeamManager.h"
#include "engine/IRoleAssignment.h"
#include "engine/Logger.h"
#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/model/Task.h"
#include "engine/model/State.h"
#include "engine/model/EntryPoint.h"
#include "engine/IAlicaClock.h"
#include "engine/Assignment.h"
#include "engine/collections/StateCollection.h"
#include "engine/IAlicaCommunication.h"

#include <math.h>

//#define PB_DEBUG

namespace alica {
/**
 * Constructs the PlanBase given a top-level plan to execute
 * @param masterplan A Plan
 */
PlanBase::PlanBase(AlicaEngine* ae, Plan* masterPlan)
        : _ae(ae)
        , _masterPlan(masterPlan)
        , _teamObserver(ae->getTeamObserver())
        , _ra(ae->getRoleAssignment())
        , _syncModel(ae->getSyncModul())
        , _authModul(ae->getAuth())
        , _statusPublisher(nullptr)
        , _alicaClock(ae->getIAlicaClock())
        , _rootNode(nullptr)
        , _deepestNode(nullptr)
        , _mainThread(nullptr)
        , _log(ae->getLog())
        , _statusMessage(nullptr)
        , _lastSendTime(0)
        , _loopInterval(0)
        , _lastSentStatusTime(0)
        , _stepModeCV()
        , _ruleBook(ae)
        , _treeDepth(0)
        , _running(false)
        , _isWaiting(false)

{
    supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

    double freq = (*sc)["Alica"]->get<double>("Alica.EngineFrequency", NULL);
    double minbcfreq = (*sc)["Alica"]->get<double>("Alica.MinBroadcastFrequency", NULL);
    double maxbcfreq = (*sc)["Alica"]->get<double>("Alica.MaxBroadcastFrequency", NULL);

    _loopTime = (AlicaTime) fmax(1000000, lround(1.0 / freq * 1000000000));
    if (_loopTime <= 1000000) {
        cerr << "PB: ALICA should not be used with more than 1000Hz -> 1000Hz assumed" << endl;
    }

    if (minbcfreq > maxbcfreq) {
        AlicaEngine::abort(
                "PB: Alica.conf: Minimal broadcast frequency must be lower or equal to maximal broadcast frequency!");
    }

    _minSendInterval = (AlicaTime) fmax(1000000, lround(1.0 / maxbcfreq * 1000000000));
    _maxSendInterval = (AlicaTime) fmax(1000000, lround(1.0 / minbcfreq * 1000000000));

    AlicaTime halfLoopTime = _loopTime / 2;

    _sendStatusMessages = (*sc)["Alica"]->get<bool>("Alica.StatusMessages.Enabled", NULL);
    if (_sendStatusMessages) {
        double stfreq = (*sc)["Alica"]->get<double>("Alica.StatusMessages.Frequency", NULL);
        _sendStatusInterval = (AlicaTime) max(1000000.0, round(1.0 / stfreq * 1000000000));
        _statusMessage = new AlicaEngineInfo();
        _statusMessage->senderID = ae->getTeamManager()->getLocalAgentID();
        _statusMessage->masterPlan = masterPlan->getName();
    }

    //#ifdef PB_DEBUG
    cout << "PB: Engine loop time is " << to_string(_loopTime / 1000000) << "ms, broadcast interval is "
         << to_string(_minSendInterval / 1000000) << "ms - " << to_string(_maxSendInterval / 1000000) << "ms" << endl;
    //#endif
    if (halfLoopTime < _minSendInterval) {
        _minSendInterval -= halfLoopTime;
        _maxSendInterval -= halfLoopTime;
    }
}

/**
 * Starts execution of the plan tree, call once all necessary modules are initialised.
 */
void PlanBase::start() {
    if (!_running) {
        _running = true;
        _mainThread = new thread(&PlanBase::run, this);
    }
}
/**
 * The Engine's main loop
 */
void PlanBase::run() {
#ifdef PB_DEBUG
    cout << "PB: Run-Method of PlanBase started. " << endl;
#endif
    while (_running) {
        AlicaTime beginTime = _alicaClock->now();
        _log->itertionStarts();

        if (_ae->getStepEngine()) {
#ifdef PB_DEBUG
            cout << "PB: ===CUR TREE===" << endl;

            if (_rootNode == nullptr) {
                cout << "PB: NULL" << endl;
            } else {
                _rootNode->printRecursive();
            }
            cout << "PB: ===END CUR TREE===" << endl;
#endif
            {
                unique_lock<mutex> lckStep(_stepMutex);
                _isWaiting = true;
                AlicaEngine* ae = _ae;
                _stepModeCV.wait(lckStep, [ae] { return ae->getStepCalled(); });
                _ae->setStepCalled(false);
                if (!_running) {
                    return;
                }
            }
            _isWaiting = false;
            beginTime = _alicaClock->now();
        }

        // Send tick to other modules
        //_ae->getCommunicator()->tick(); // not implemented as ros does work asynchronous
        _teamObserver->tick(_rootNode);
        _ra->tick();
        _syncModel->tick();
        _authModul->tick(_rootNode);

        if (_rootNode == nullptr) {
            _rootNode = _ruleBook.initialisationRule(_masterPlan);
        }
        if (_rootNode->tick(&_ruleBook) == PlanChange::FailChange) {
            cout << "PB: MasterPlan Failed" << endl;
        }
        // lock for fpEvents
        {
            lock_guard<mutex> lock(_lomutex);
            _fpEvents = queue<shared_ptr<RunningPlan>>();
        }

        AlicaTime now = _alicaClock->now();

        if (now < _lastSendTime) {
            // Taker fix
            std::cout << "PB: lastSendTime is in the future of the current system time, did the system time change?"
                      << endl;
            _lastSendTime = now;
        }

        if ((_ruleBook.isChangeOccured() && _lastSendTime + _minSendInterval < now) ||
                _lastSendTime + _maxSendInterval < now) {
            list<long> msg;
            _deepestNode = _rootNode;
            _treeDepth = 0;
            _rootNode->toMessage(msg, _deepestNode, _treeDepth, 0);
            _teamObserver->doBroadCast(msg);
            _lastSendTime = now;
            _ruleBook.setChangeOccured(false);
        }

        if (_sendStatusMessages && _lastSentStatusTime + _sendStatusInterval < _alicaClock->now()) {
            if (_deepestNode != nullptr) {
                _statusMessage->robotIDsWithMe.clear();
                _statusMessage->currentPlan = _deepestNode->getPlan()->getName();
                if (_deepestNode->getOwnEntryPoint() != nullptr) {
                    _statusMessage->currentTask = _deepestNode->getOwnEntryPoint()->getTask()->getName();
                } else {
                    _statusMessage->currentTask = "IDLE";
                }
                if (_deepestNode->getActiveState() != nullptr) {
                    _statusMessage->currentState = _deepestNode->getActiveState()->getName();
                    _deepestNode->getAssignment()
                                      ->getRobotStateMapping()
                                      ->getRobotsInState(_deepestNode->getActiveState(),_statusMessage->robotIDsWithMe);

                } else {
                    _statusMessage->currentState = "NONE";
                }
                auto tmpRole = _ra->getOwnRole();
                if (tmpRole) {
                    _statusMessage->currentRole = _ra->getOwnRole()->getName();
                } else {
                    _statusMessage->currentRole = "No Role";
                }
                _ae->getCommunicator()->sendAlicaEngineInfo(*_statusMessage);
                _lastSentStatusTime = _alicaClock->now();
            }
        }

        _log->iterationEnds(_rootNode);

        _ae->iterationComplete();

        AlicaTime availTime;

        now = _alicaClock->now();

        // TODO: FIXME: the division by 1000 is brittle and should not be needed
        // replace AlicaTime with a proper time class.
        availTime = (AlicaTime)((_loopTime - (now - beginTime)) / 1000L);
        bool checkFp = false;
        if (availTime > 1000) {
            std::unique_lock<std::mutex> lock(_lomutex);
            checkFp = std::cv_status::no_timeout == _fpEventWait.wait_for(lock, std::chrono::microseconds(availTime));
        }

        if (checkFp && _fpEvents.size() > 0) {
            // lock for fpEvents
            {
                lock_guard<mutex> lock(_lomutex);
                while (_running && availTime > 1000 && _fpEvents.size() > 0) {
                    shared_ptr<RunningPlan> rp = _fpEvents.front();
                    _fpEvents.pop();

                    if (rp->isActive()) {
                        bool first = true;
                        while (rp != nullptr) {
                            cout << "TICK FPEVENT " << endl;
                            PlanChange change = _ruleBook.visit(rp);
                            cout << "AFTER TICK FPEVENT " << endl;
                            if (!first && change == PlanChange::NoChange) {
                                break;
                            }
                            rp = rp->getParent().lock();
                            first = false;
                        }
                    }
                    now = _alicaClock->now();
                    availTime = (AlicaTime)((_loopTime - (now - beginTime)) / 1000L);
                }
            }
        }

        now = _alicaClock->now();
        availTime = (AlicaTime)((_loopTime - (now - beginTime)) / 1000L);

#ifdef PB_DEBUG
        cout << "PB: availTime " << availTime << endl;
#endif
        if (availTime > 100 && !_ae->getStepEngine()) {
            _alicaClock->sleep(availTime);
        }
    }
}

/**
 * Stops the plan base thread.
 */
void PlanBase::stop() {
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

PlanBase::~PlanBase() {
    delete _statusMessage;
}

void PlanBase::addFastPathEvent(shared_ptr<RunningPlan> p) {
    {
        lock_guard<mutex> lock(_lomutex);
        _fpEvents.push(p);
    }
    _fpEventWait.notify_all();
}

const AlicaTime PlanBase::getloopInterval() const {
    return _loopInterval;
}

void PlanBase::setLoopInterval(ulong loopInterval) {
    _loopInterval = loopInterval;
}

condition_variable* PlanBase::getStepModeCV() {
    if (!_ae->getStepEngine()) {
        return nullptr;
    }
    return &_stepModeCV;
}
/**
 * Returns the root node of the ALICA plan tree in execution.
 */
const shared_ptr<RunningPlan> PlanBase::getRootNode() const {
    return _rootNode;
}

void PlanBase::setRootNode(shared_ptr<RunningPlan> rootNode) {
    _rootNode = rootNode;
}

/**
 * Returns the deepest ALICA node
 */
shared_ptr<const RunningPlan> PlanBase::getDeepestNode() const {
    return _deepestNode;
}

/**
 * Returns the deepest ALICA node
 */
shared_ptr<RunningPlan> PlanBase::getRootNode() {
    return _rootNode;
}

}  // namespace alica
