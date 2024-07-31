#pragma once

#include "engine/AlicaClock.h"
#include "engine/RuleBook.h"
#include "engine/RunningPlan.h"
#include "engine/RuntimeBehaviourFactory.h"
#include "engine/RuntimePlanFactory.h"
#include "engine/containers/AlicaEngineInfo.h"
#include <algorithm>
#include <atomic>
#include <condition_variable>
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
class TeamObserver;
class IRoleAssignment;
class AuthorityManager;
class SyncModule;
class IAlicaCommunication;
class Task;
class State;
class EntryPoint;
class Assignment;
class StateCollection;
class PlanType;
class Plan;
class Blackboard;
class RuntimePlanFactory;
class RuntimeBehaviourFactory;
class VariableSyncModule;
class ISolverBase;
/**
 * A PlanBase holds the internal representation of the plan graph and issues all operations on it.
 * It is the most central object within the ALICA Engine.
 */
class PlanBase
{
public:
    PlanBase(ConfigChangeListener& configChangeListener, const AlicaClock& clock, const IAlicaCommunication& communicator, IRoleAssignment& roleAssignment,
            SyncModule& syncModule, AuthorityManager& authorityManager, TeamObserver& teamObserver, TeamManager& teamManager,
            const PlanRepository& planRepository, std::atomic<bool>& stepEngine, std::atomic<bool>& stepCalled, std::shared_ptr<Blackboard> globalBlackboard,
            VariableSyncModule& resultStore, const std::unordered_map<size_t, std::unique_ptr<ISolverBase>>& solvers, const IAlicaTimerFactory& timerFactory,
            const IAlicaTraceFactory* traceFactory);
    ~PlanBase();
    RunningPlan* getRootNode() const { return _runningPlans.empty() ? nullptr : _runningPlans[0].get(); }
    PlanSelector* getPlanSelector() const { return _ruleBook.getPlanSelector(); }
    const RunningPlan* getDeepestNode() const;

    const AlicaTime getLoopInterval() const;
    void setLoopInterval(AlicaTime loopInterval);
    void stop();
    void start(const Plan* masterPlan);
    void addFastPathEvent(RunningPlan* p);
    bool isWaiting() const { return _isWaiting; }

    // factory function
    RunningPlan* makeRunningPlan(const AbstractPlan* abstractPlan, const ConfAbstractPlanWrapper* wrapper);

    void reload(const YAML::Node& config);
    void init(std::unique_ptr<IBehaviourCreator>&& behaviourCreator, std::unique_ptr<IPlanCreator>&& planCreator);

    const RuntimePlanFactory& getRuntimePlanFactory() const { return _runTimePlanFactory; }
    const RuntimeBehaviourFactory& getRuntimeBehaviourFactory() const { return _runTimeBehaviourFactory; }

    void stepNotify();

private:
    static constexpr const char* LOGNAME = "PlanBase";

    void run(const Plan* masterPlan);

    // Owning container of running plans (replace with uniqueptrs once possible)
    std::vector<std::shared_ptr<RunningPlan>> _runningPlans;

    /**
     * List of RunningPlans scheduled for out-of-loop evaluation.
     */

    ConfigChangeListener& _configChangeListener;
    const AlicaClock& _clock;
    const IAlicaCommunication& _communicator;
    IRoleAssignment& _roleAssignment;
    SyncModule& _syncModule;
    AuthorityManager& _authorityManager;
    TeamObserver& _teamObserver;
    TeamManager& _teamManager;
    const PlanRepository& _planRepository;
    std::atomic<bool>& _stepEngine;
    std::atomic<bool>& _stepCalled;
    std::shared_ptr<Blackboard> _globalBlackboard;
    RuntimePlanFactory _runTimePlanFactory;
    RuntimeBehaviourFactory _runTimeBehaviourFactory;
    VariableSyncModule& _resultStore;
    const std::unordered_map<size_t, std::unique_ptr<ISolverBase>>& _solvers;
    RunningPlan* _rootNode;

    const RunningPlan* _deepestNode;

    std::unique_ptr<std::thread> _mainThread;
    std::unique_ptr<AlicaEngineInfo> _statusMessage;

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
    std::atomic<bool> _running;
    bool _sendStatusMessages;
    std::atomic<bool> _isWaiting;
};

} // namespace alica
