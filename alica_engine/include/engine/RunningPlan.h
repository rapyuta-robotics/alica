#pragma once

#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/PlanChange.h"
#include "engine/PlanStatus.h"
#include "engine/Types.h"
#include "engine/allocationauthority/CycleManager.h"
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
class State;
class EntryPoint;
class PlanType;
class PlanBase;
class TeamObserver;
class Plan;
class RuleBook;
class BehaviourConfiguration;
class IPlanTreeVisitor;
class SimplePlanTree;

struct PlanStateTriple
{
    PlanStateTriple()
            : state(nullptr)
            , entryPoint(nullptr)
            , plan(nullptr)
    {
    }
    PlanStateTriple(const AbstractPlan* p, const EntryPoint* e, const State* s)
            : state(s)
            , entryPoint(e)
            , plan(p)
    {
    }
    const State* state;
    const EntryPoint* entryPoint;
    const AbstractPlan* plan;
};

/**
 * A RunningPlan represents a plan or a behaviour in execution, holding all information relevant at runtime.
 */

class RunningPlan : public std::enable_shared_from_this<RunningPlan>
{

    // TODO: read write locks to this class
public:
    struct PlanStatusInfo
    {
        PlanStatusInfo()
                : status(PlanStatus::Running)
                , failCount(0)
                , stateStartTime()
                , planStartTime()
                , active(false)
                , allocationNeeded(false)
                , failHandlingNeeded(false)
        {
        }
        PlanStatus status;
        AlicaTime stateStartTime;
        AlicaTime planStartTime;

        int failCount;
        bool failHandlingNeeded;
        bool active;
        bool allocationNeeded;
    };
    static void init();

    virtual ~RunningPlan();

    bool isBehaviour() const { return _behaviour; };
    bool isAllocationNeeded() const { return _status.allocationNeeded; }
    bool isFailureHandlingNeeded() const { return _status.failHandlingNeeded; }
    PlanStatus getStatus() const { return _status.status; }
    AlicaTime getPlanStartTime() const { return _status.planStartTime; }
    AlicaTime getStateStartTime() const { return _status.stateStartTime; }
    bool isActive() const { return _status.active; }
    bool isRetired() const { return _status.status == PlanStatus::Retired; }

    const std::vector<RunningPlan*>& getChildren() const { return _children; }
    RunningPlan* getParent() const { return _parent; }

    const PlanType* getPlanType() const { return _planType; }

    PlanStateTriple getActiveTriple() const { return _activeTriple; }
    const PlanStatusInfo& getStatusInfo() const { return _status; }
    const State* getActiveState() const { return _activeTriple.state; }
    const EntryPoint* getActiveEntryPoint() const { return _activeTriple.entryPoint; }
    const AbstractPlan* getActivePlan() const { return _activeTriple.plan; }
    const Assignment& getAssignment() const { return _assignment; }
    Assignment& editAssignment() { return _assignment; }
    BasicBehaviour* getBasicBehaviour() const { return _basicBehaviour; }

    void printRecursive() const;

    const AgentGrp& getRobotsAvail() const { return _robotsAvail; }
    AgentGrp& editRobotsAvail() { return _robotsAvail; }

    void setAllocationNeeded(bool allocationNeeded);
    void useEntryPoint(const EntryPoint* value);
    void useState(const State* activeState);

    void usePlan(const AbstractPlan* plan);
    void setParent(RunningPlan* parent) { _parent = parent; }
    void setFailureHandlingNeeded(bool failHandlingNeeded) { _status.failHandlingNeeded = true; }
    void setAssignment(const Assignment& assignment) { _assignment = assignment; }
    void setBasicBehaviour(BasicBehaviour* basicBehaviour) { _basicBehaviour = basicBehaviour; }
    void adaptAssignment(const RunningPlan& replacement);
    void clearFailures();
    // void setActive(bool active);

    PlanChange tick(RuleBook* rules);

    const ConditionStore& getConstraintStore() const { return _constraintStore; }
    ConditionStore& editConstraintStore() { return _constraintStore; }

    const CycleManager& getCycleManagement() const { return _cycleManagement; }
    CycleManager& editCycleManagement() { return _cycleManagement; }

    // Temporary helper:
    std::shared_ptr<RunningPlan> getSharedPointer() { return shared_from_this(); }

    // const EntryPoint* getOwnEntryPoint() const;
    // void setParent(std::weak_ptr<RunningPlan> s);

    // bool getFailHandlingNeeded() const;

    bool evalPreCondition() const;
    bool evalRuntimeCondition() const;

    void addChildren(const std::vector<RunningPlan*>& runningPlans);
    // void addChildren(std::list<std::shared_ptr<RunningPlan>>& children);
    void moveState(const State* nextState);

    void clearFailedChildren();
    void addFailure();
    int getFailure();
    void deactivateChildren();
    void clearChildren();

    void setFailedChild(const AbstractPlan* child);
    void accept(IPlanTreeVisitor* vis);

    void deactivate();
    void activate();

    bool isAnyChildStatus(PlanStatus ps) const;
    bool areAllChildrenStatus(PlanStatus ps) const;
    bool isAnyChildTaskSuccessful() const;

    void setActiveEntryPoint(EntryPoint* activeEntryPoint);
    void limitToRobots(const AgentGrp& robots);

    void revokeAllConstraints();
    void attachPlanConstraints();
    bool recursiveUpdateAssignment(
            std::list<std::shared_ptr<SimplePlanTree>> spts, AgentGrp& availableAgents, std::list<AgentIDConstPtr> noUpdates, AlicaTime now);
    void toMessage(IdGrp& message, const RunningPlan*& o_deepestNode, int& depth, int curDepth) const;
    std::string toString() const;
    AgentIDConstPtr getOwnID() const { return _ae->getTeamManager()->getLocalAgentID(); }
    AlicaEngine* getAlicaEngine() const { return _ae; }

    void sendLogMessage(int level, const std::string& message) const;

private:
    friend std::ostream& operator<<(std::ostream& out, const RunningPlan& r);
    friend PlanBase; // temporary while refactoring
    RunningPlan(AlicaEngine* ae);
    RunningPlan(AlicaEngine* ae, const Plan* plan);
    RunningPlan(AlicaEngine* ae, const PlanType* pt);
    RunningPlan(AlicaEngine* ae, const BehaviourConfiguration* bc);

    // Status Information
    PlanStateTriple _activeTriple;
    PlanStatusInfo _status;

    std::vector<RunningPlan*> _children;
    RunningPlan* _parent;

    BasicBehaviour* _basicBehaviour;
    // Components
    Assignment _assignment;
    CycleManager _cycleManagement;
    ConditionStore _constraintStore;

    // Type info

    const bool _behaviour; // TODO: get rid of this, the behaviour pointer should not be null for behaviors (currently it can be)

    // engine Pointer
    AlicaEngine* const _ae;
    const PlanType* const _planType;

    // iffy stuff
    AgentGrp _robotsAvail;
    std::map<const AbstractPlan*, int> _failedSubPlans;

    static AlicaTime assignmentProtectionTime;
};

std::ostream& operator<<(std::ostream& out, const RunningPlan& r);

} /* namespace alica */
