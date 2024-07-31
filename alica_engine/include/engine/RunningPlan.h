#pragma once

#include "engine/AlicaClock.h"
#include "engine/Assignment.h"
#include "engine/PlanChange.h"
#include "engine/PlanStatus.h"
#include "engine/Types.h"
#include "engine/allocationauthority/CycleManager.h"
#include "engine/constraintmodul/ConditionStore.h"
#include "engine/teammanager/TeamManager.h"

#include <algorithm>
#include <atomic>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <yaml-cpp/yaml.h>

namespace alica
{
class AbstractPlan;
class BasicBehaviour;
class BasicPlan;
class State;
class EntryPoint;
class PlanType;
class PlanBase;
class TeamObserver;
class Plan;
class RuleBook;
class ConditionStore;
class CycleManager;
class Behaviour;
class IPlanTreeVisitor;
class SimplePlanTree;
class Blackboard;
class KeyMapping;
class Blackboard;
class RuntimePlanFactory;
class RuntimeBehaviourFactory;
class VariableSyncModule;
class ISolverBase;

struct PlanStateTriple
{
    PlanStateTriple()
            : state(nullptr)
            , entryPoint(nullptr)
            , abstractPlan(nullptr)
    {
    }
    PlanStateTriple(const AbstractPlan* p, const EntryPoint* e, const State* s)
            : state(s)
            , entryPoint(e)
            , abstractPlan(p)
    {
    }
    const State* state;
    const EntryPoint* entryPoint;
    const AbstractPlan* abstractPlan;
};

/**
 * A RunningPlan represents a plan or a behaviour in execution, holding all information relevant at runtime.
 */
// TODO remove enable_share_from_this
class RunningPlan : public std::enable_shared_from_this<RunningPlan>
{
public:
    using ScopedReadLock = std::unique_lock<std::mutex>;
    using ScopedWriteLock = std::unique_lock<std::mutex>;
    enum class EvalStatus
    {
        True,
        False,
        Unknown
    };
    struct PlanStatusInfo
    {
        PlanStatusInfo()
                : status(PlanStatus::Running)
                , active(PlanActivity::InActive)
                , stateStartTime()
                , planStartTime()
                , failCount(0)
                , failHandlingNeeded(false)
                , allocationNeeded(false)
                , runTimeConditionStatus(EvalStatus::Unknown)
        {
        }
        PlanStatus status;
        PlanActivity active;
        AlicaTime stateStartTime;
        AlicaTime planStartTime;

        int failCount;
        bool failHandlingNeeded;
        bool allocationNeeded;
        mutable std::atomic<EvalStatus> runTimeConditionStatus;
    };
    explicit RunningPlan(ConfigChangeListener& configChangeListener, const AlicaClock& clock, std::shared_ptr<const Blackboard> globalBlackboard,
            const RuntimePlanFactory& runTimePlanFactory, TeamObserver& teamObserver, TeamManager& teamManager, const PlanRepository& planRepository,
            VariableSyncModule& resultStore, const std::unordered_map<size_t, std::unique_ptr<ISolverBase>>& solvers);
    RunningPlan(ConfigChangeListener& configChangeListener, const AlicaClock& clock, std::shared_ptr<const Blackboard> globalBlackboard,
            TeamObserver& teamObserver, TeamManager& teamManager, const PlanRepository& planRepository, VariableSyncModule& resultStore,
            const std::unordered_map<size_t, std::unique_ptr<ISolverBase>>& solvers, const RuntimePlanFactory& runTimePlanFactory,
            const RuntimeBehaviourFactory& runTimeBehaviourFactory, const AbstractPlan* abstractPlan, const ConfAbstractPlanWrapper* wrapper);
    static void init(const YAML::Node& config);
    static void setAssignmentProtectionTime(AlicaTime t);

    virtual ~RunningPlan();

    bool isBehaviour() const { return _behaviour; };
    bool isAllocationNeeded() const { return _status.allocationNeeded; }
    bool isFailureHandlingNeeded() const { return _status.failHandlingNeeded; }
    PlanStatus getStatus() const;
    AlicaTime getPlanStartTime() const { return _status.planStartTime; }
    AlicaTime getStateStartTime() const { return _status.stateStartTime; }
    bool isActive() const { return _status.active == PlanActivity::Active; }
    bool isRetired() const { return _status.active == PlanActivity::Retired; }
    bool isDeleteable() const;

    // Read/Write lock access, currently map to a single mutex
    // for future use already defined apart
    ScopedReadLock getReadLock() const { return ScopedReadLock(_accessMutex); }
    ScopedWriteLock getWriteLock() { return ScopedWriteLock(_accessMutex); }

    const std::vector<RunningPlan*>& getChildren() const { return _children; }
    RunningPlan* getParent() const { return _parent; }

    const PlanType* getPlanType() const { return _planType; }

    PlanStateTriple getActiveTriple() const { return _activeTriple; }
    const PlanStatusInfo& getStatusInfo() const { return _status; }
    const State* getActiveState() const { return _activeTriple.state; }
    const EntryPoint* getActiveEntryPoint() const { return _activeTriple.entryPoint; }
    const AbstractPlan* getActivePlan() const { return _activeTriple.abstractPlan; }
    const Plan* getActivePlanAsPlan() const { return isBehaviour() ? nullptr : static_cast<const Plan*>(_activeTriple.abstractPlan); }
    const Assignment& getAssignment() const { return _assignment; }
    Assignment& editAssignment() { return _assignment; }
    // TODO Cleanup: Get rid of these 2 methods
    BasicBehaviour* getBasicBehaviour() const { return _basicBehaviour.get(); }
    BasicPlan* getBasicPlan() const { return _basicPlan.get(); }

    std::shared_ptr<Blackboard> getBlackboard() const;
    const KeyMapping* getKeyMapping(int64_t wrapperId) const;
    const KeyMapping* getKeyMapping() const;

    void printRecursive() const;

    void setAllocationNeeded(bool allocationNeeded);
    void useEntryPoint(const EntryPoint* value);
    void useState(const State* activeState);

    void usePlan(const AbstractPlan* plan);
    void setParent(RunningPlan* parent) { _parent = parent; }
    void setFailureHandlingNeeded(bool failHandlingNeeded);
    void setAssignment(const Assignment& assignment) { _assignment = assignment; }
    void adaptAssignment(const RunningPlan& replacement);
    void clearFailures();

    void preTick();
    PlanChange tick(RuleBook* rules);

    const ConditionStore& getConstraintStore() const { return _constraintStore; }
    ConditionStore& editConstraintStore() { return _constraintStore; }

    const CycleManager& getCycleManagement() const { return _cycleManagement; }
    CycleManager& editCycleManagement() { return _cycleManagement; }

    // Temporary helper:
    std::shared_ptr<RunningPlan> getSharedPointer() { return shared_from_this(); }

    bool evalPreCondition() const;
    bool isRuntimeConditionValid() const
    {
        switch (_status.runTimeConditionStatus) {
        case EvalStatus::True:
            return true;
        case EvalStatus::False:
            return false;
        case EvalStatus::Unknown:
        default:
            return evalRuntimeCondition();
        }
    }

    void addChildren(const std::vector<RunningPlan*>& runningPlans);
    void removeChild(RunningPlan* rp);

    void moveState(const State* nextState);

    void clearFailedChildren();
    void addFailure();
    int getFailureCount() const;
    void deactivateChildren();
    void clearChildren();

    void setFailedChild(const AbstractPlan* child);
    void accept(IPlanTreeVisitor* vis);

    void deactivate();
    void activate();

    bool isAnyChildStatus(PlanStatus ps) const;
    bool areAllChildrenStatus(PlanStatus ps) const;
    bool isAnyChildTaskSuccessful() const;
    bool amISuccessful() const;
    bool amISuccessfulInAnyChild() const;

    void setActiveEntryPoint(EntryPoint* activeEntryPoint);
    void limitToRobots(const AgentGrp& robots);

    void revokeAllConstraints();
    void attachPlanConstraints();
    bool recursiveUpdateAssignment(const std::vector<const SimplePlanTree*>& spts, AgentGrp& availableAgents, const AgentGrp& noUpdates, AlicaTime now);
    void toMessage(IdGrp& message, const RunningPlan*& o_deepestNode, int& o_depth, int curDepth) const;
    AgentId getOwnID() const;
    int64_t getParentWrapperId(const RunningPlan* rp) const;
    const AlicaClock& getAlicaClock() const { return _clock; };
    VariableSyncModule& editResultStore() const { return _resultStore; }

    const ConfAbstractPlanWrapper* getConfAbstractPlanWrapper() const { return _wrapper; };

    //[[deprecated("temporary method tobe removed in last PR")]]
    template <class SolverType>
    SolverType& getSolver() const;
    ISolverBase& getSolverBase(const std::type_info& solverType) const;

    //[[deprecated("temporary method tobe removed in last PR")]]
    template <class SolverType>
    bool existSolver() const;

    TeamManager& getTeamManager() const;

private:
    static constexpr const char* LOGNAME = "RunningPlan";

    friend std::ostream& operator<<(std::ostream& out, const RunningPlan& r);
    bool evalRuntimeCondition() const;
    // Status Information
    PlanStateTriple _activeTriple;
    PlanStatusInfo _status;

    std::vector<RunningPlan*> _children;
    RunningPlan* _parent;
    std::unique_ptr<BasicBehaviour> _basicBehaviour;
    std::unique_ptr<BasicPlan> _basicPlan;
    const ConfAbstractPlanWrapper* _wrapper;
    // Components
    Assignment _assignment;
    CycleManager _cycleManagement;
    ConditionStore _constraintStore;

    // Type info
    const PlanType* _planType;
    bool _behaviour; // TODO: get rid of this, the behaviour pointer should not be null for behaviors (currently it can be)

    // engine Pointer
    const AlicaClock& _clock;
    std::shared_ptr<const Blackboard> _globalBlackboard;
    const RuntimePlanFactory& _runTimePlanFactory;
    TeamObserver& _teamObserver;
    TeamManager& _teamManager;
    VariableSyncModule& _resultStore;
    const std::unordered_map<size_t, std::unique_ptr<ISolverBase>>& _solvers;

    // iffy stuff
    std::map<const AbstractPlan*, int> _failedSubPlans;

    mutable std::mutex _accessMutex;
};

template <class SolverType>
SolverType& RunningPlan::getSolver() const
{
    auto cit = _solvers.find(typeid(SolverType).hash_code());
    assert(cit != _solvers.end());
    return static_cast<SolverType&>(*(cit->second));
}

template <class SolverType>
bool RunningPlan::existSolver() const
{
    auto cit = _solvers.find(typeid(SolverType).hash_code());
    return (cit != _solvers.end());
}

std::ostream& operator<<(std::ostream& out, const RunningPlan& r);

} /* namespace alica */
