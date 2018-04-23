#pragma once

#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/PlanChange.h"
#include "engine/PlanStatus.h"
#include "engine/Types.h"
#include "engine/constraintmodul/ConditionStore.h"
#include "engine/teammanager/TeamManager.h"
#include "supplementary/AgentID.h"

#include <SystemConfig.h>

#include <algorithm>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>

namespace alica
{
class BehaviourPool;
class BasicBehaviour;
class AbstractPlan;
class Assignment;
class State;
class EntryPoint;
class PlanType;
class TeamObserver;
class Plan;
class RuleBook;
class ConditionStore;
class CycleManager;
class BehaviourConfiguration;
class IPlanTreeVisitor;
class SimplePlanTree;

/**
 * A RunningPlan represents a plan or a behaviour in execution, holding all information relevant at runtime.
 */
class RunningPlan : public enable_shared_from_this<RunningPlan>
{
  public:
    static void init();
    RunningPlan(AlicaEngine* ae);
    RunningPlan(AlicaEngine* ae, const Plan* plan);
    RunningPlan(AlicaEngine* ae, const PlanType* pt);
    RunningPlan(AlicaEngine* ae, const BehaviourConfiguration* bc);
    virtual ~RunningPlan();
    /**
     * Indicates whether this running plan represents a behaviour.
     * true if this instance is representing a behaviour; otherwise, false.
     */
    bool isBehaviour() const { return _behaviour; };

    bool isAllocationNeeded() const { return _allocationNeeded; }
    list<shared_ptr<RunningPlan>>* getChildren();
    void setChildren(list<shared_ptr<RunningPlan>> children);
    /**
     * The abstract plan associated with this running plan, a model element.
     */
    const AbstractPlan* getPlan() const { return _plan; }
    void setPlan(const AbstractPlan* plan);
    shared_ptr<BasicBehaviour> getBasicBehaviour();
    void setBasicBehaviour(shared_ptr<BasicBehaviour> basicBehaviour);
    shared_ptr<Assignment> getAssignment() const;
    void setAssignment(shared_ptr<Assignment> assignment);
    void printRecursive();
    AlicaTime getPlanStartTime() const { return _planStartTime; }
    AlicaTime getStateStartTime() const { return _stateStartTime; }
    bool isActive() const { return _active; }
    void setActive(bool active);

    const AgentGrp& getRobotsAvail() const { return _robotsAvail; }
    AgentGrp& editRobotsAvail() { return _robotsAvail; }

    void setAllocationNeeded(bool allocationNeeded);
    void setFailHandlingNeeded(bool failHandlingNeeded);
    void setOwnEntryPoint(const EntryPoint* value);
    PlanChange tick(RuleBook* rules);

    const ConditionStore& getConstraintStore() const { return _constraintStore; }
    ConditionStore& editConstraintStore() { return _constraintStore; }

    const EntryPoint* getOwnEntryPoint() const;
    void setParent(weak_ptr<RunningPlan> s);
    weak_ptr<RunningPlan> getParent() const;
    bool getFailHandlingNeeded() const;
    PlanStatus getStatus() const;
    /**
     * Gets the PlanType of the currently executed plan. nullptr if the AbstractPlan associated does not belong to a
     * PlanType.
     */
    const PlanType* getPlanType() const { return _planType; }
    bool evalPreCondition();
    bool evalRuntimeCondition();
    const State* getActiveState() const { return _activeState; }
    void setActiveState(const State* activeState);
    void addChildren(shared_ptr<list<shared_ptr<RunningPlan>>>& runningPlans);
    void addChildren(list<shared_ptr<RunningPlan>>& children);
    void moveState(const State* nextState);
    void clearFailures();
    void clearFailedChildren();
    void addFailure();
    int getFailure();
    void deactivateChildren();
    void clearChildren();
    void adaptAssignment(shared_ptr<RunningPlan> r);
    void setFailedChild(const AbstractPlan* child);
    void accept(IPlanTreeVisitor* vis);
    void deactivate();
    bool anyChildrenStatus(PlanStatus ps);
    bool allChildrenStatus(PlanStatus ps);
    bool anyChildrenTaskSuccess();
    void activate();
    const EntryPoint* getActiveEntryPoint() const { return _activeEntryPoint; }
    void setActiveEntryPoint(EntryPoint* activeEntryPoint);
    void limitToRobots(const AgentGrp& robots);
    std::shared_ptr<CycleManager> getCycleManagement();
    void revokeAllConstraints();
    void attachPlanConstraints();
    bool recursiveUpdateAssignment(list<shared_ptr<SimplePlanTree>> spts, AgentGrp& availableAgents, list<const supplementary::AgentID*> noUpdates,
                                   AlicaTime now);
    void toMessage(list<long>& message, shared_ptr<const RunningPlan>& deepestNode, int& depth, int curDepth) const;
    std::string toString() const;
    const supplementary::AgentID* getOwnID() const { return _ae->getTeamManager()->getLocalAgentID(); }
    AlicaEngine* getAlicaEngine() const { return _ae; }

    void sendLogMessage(int level, string& message);

  protected:
    const State* _activeState;
    const EntryPoint* _activeEntryPoint;

    AlicaEngine* _ae;

    const AbstractPlan* _plan;
    const PlanType* _planType;
    AgentGrp _robotsAvail;
    std::map<const AbstractPlan*, int> _failedSubPlans;
    weak_ptr<RunningPlan> _parent;
    list<shared_ptr<RunningPlan>> _children;
    std::shared_ptr<Assignment> _assignment;
    std::shared_ptr<CycleManager> _cycleManagement;

    shared_ptr<BasicBehaviour> _basicBehaviour;

    PlanStatus _status;
    /**
     * The (ROS-)timestamp referring to when the local robot entered the ActiveState.
     */
    AlicaTime _stateStartTime;
    /**
     * The timestamp referring to when this plan was started by the local robot
     */
    AlicaTime _planStartTime;

    int _failCount;
    bool _failHandlingNeeded;
    bool _active;
    /**
     * Whether or not this running plan is active or has been removed from the plan tree
     */
    const bool _behaviour;
    bool _allocationNeeded;

    ConditionStore _constraintStore;

    static AlicaTime assignmentProtectionTime;
};

} /* namespace alica */
