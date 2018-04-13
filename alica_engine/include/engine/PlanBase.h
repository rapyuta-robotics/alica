#pragma once

#include <engine/containers/AlicaEngineInfo.h>
#include <queue>
#include <stdio.h>
#include <thread>
#include <condition_variable>
#include <algorithm>
#include <math.h>
#include <mutex>
#include <memory>
#include <typeinfo>
#include "engine/IAlicaClock.h"
#include "engine/RunningPlan.h"
#include "engine/RuleBook.h"

namespace alica {
class Plan;

class AlicaEngine;
class TeamObserver;
class IRoleAssignment;
class Logger;
class AuthorityManager;
class SyncModule;
class IAlicaCommunication;
class Task;
class State;
class EntryPoint;
class IAlicaClock;
class Assignment;
class StateCollection;
class AlicaEngine;

/**
 * A PlanBase holds the internal representation of the plan graph and issues all operations on it.
 * It is the most central object within the ALICA Engine.
 */
class PlanBase {
public:
    PlanBase(AlicaEngine* ae, const Plan* masterplan);
    ~PlanBase();
    std::condition_variable* getStepModeCV();
    const std::shared_ptr<RunningPlan> getRootNode() const;
    void setRootNode(std::shared_ptr<RunningPlan> rootNode);
    const AlicaTime getloopInterval() const;
    void setLoopInterval(ulong loopInterval);
    void stop();
    void start();
    void addFastPathEvent(shared_ptr<RunningPlan> p);
    std::shared_ptr<const RunningPlan> getDeepestNode() const;
    std::shared_ptr<RunningPlan> getRootNode();

    const Plan* getMasterPlan() const { return _masterPlan; }
    bool isWaiting() const { return _isWaiting; }

private:
    void run();

    /**
     * List of RunningPlans scheduled for out-of-loop evaluation.
     */

    AlicaEngine* _ae;
    const Plan* _masterPlan;

    TeamObserver* _teamObserver;
    IRoleAssignment* _ra;
    SyncModule* _syncModel;
    AuthorityManager* _authModul;
    IAlicaCommunication* _statusPublisher;
    IAlicaClock* _alicaClock;

    std::shared_ptr<RunningPlan> _rootNode;
    std::shared_ptr<const RunningPlan> _deepestNode;

    std::thread* _mainThread;
    Logger* _log;
    AlicaEngineInfo* _statusMessage;

    AlicaTime _loopTime;
    AlicaTime _lastSendTime;
    AlicaTime _minSendInterval;
    AlicaTime _maxSendInterval;
    AlicaTime _loopInterval;
    AlicaTime _lastSentStatusTime;
    AlicaTime _sendStatusInterval;

    std::mutex _lomutex;
    std::mutex _stepMutex;

    std::queue<shared_ptr<RunningPlan>> _fpEvents;
    std::condition_variable _fpEventWait;
    std::condition_variable _stepModeCV;
    RuleBook _ruleBook;

    int _treeDepth;
    bool _running;
    bool _sendStatusMessages;
    bool _isWaiting;
};

}  // namespace alica
