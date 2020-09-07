#pragma once

#include "engine/AlicaClock.h"
#include "engine/RuleBook.h"
#include "engine/RunningPlan.h"
#include <algorithm>
#include <condition_variable>
#include <engine/containers/AlicaEngineInfo.h>
#include <math.h>
#include <memory>
#include <mutex>
#include <queue>
#include <stdio.h>
#include <thread>
#include <typeinfo>

namespace alica
{
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
class Assignment;
class StateCollection;
class AlicaEngine;
class PlanType;
class Plan;
/**
 * A PlanBase holds the internal representation of the plan graph and issues all operations on it.
 * It is the most central object within the ALICA Engine.
 */
class PlanBase
{
public:
    PlanBase(AlicaEngine* ae);
    ~PlanBase();
    RunningPlan* getRootNode() const { return _runningPlans.empty() ? nullptr : _runningPlans[0].get(); }
    PlanSelector* getPlanSelector() const { return _ruleBook.getPlanSelector(); }
    const RunningPlan* getDeepestNode() const;

    std::condition_variable* getStepModeCV();

    const AlicaTime getLoopInterval() const;
    void setLoopInterval(AlicaTime loopInterval);
    void stop();
    void start(const Plan* masterPlan);
    void addFastPathEvent(RunningPlan* p);
    bool isWaiting() const { return _isWaiting; }

    // factory functions
    RunningPlan* makeRunningPlan(const Plan* plan, const Configuration* configuration);
    RunningPlan* makeRunningPlan(const Behaviour* b, const Configuration* configuration);
    RunningPlan* makeRunningPlan(const PlanType* pt, const Configuration* configuration);

private:
    void run(const Plan* masterPlan);

    // Owning container of running plans (replace with uniqueptrs once possibe)
    std::vector<std::shared_ptr<RunningPlan>> _runningPlans;

    /**
     * List of RunningPlans scheduled for out-of-loop evaluation.
     */

    AlicaEngine* _ae;
    RunningPlan* _rootNode;

    const RunningPlan* _deepestNode;

    std::thread* _mainThread;
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

    std::queue<RunningPlan*> _fpEvents;
    std::condition_variable _fpEventWait;
    std::condition_variable _stepModeCV;
    RuleBook _ruleBook;

    int _treeDepth;
    bool _running;
    bool _sendStatusMessages;
    bool _isWaiting;
};

} // namespace alica
