#pragma once

#include "engine/Types.h"
#include <engine/planselector/PartialAssignmentPool.h>
namespace alica
{
class RunningPlan;
class AbstractPlan;
class TeamObserver;
class RunningPlan;
class AbstractPlan;
class PlanType;
class Plan;
class PlanBase;

/**
 * Implements the task allocation algorithm
 */
class PlanSelector
{
public:
    PlanSelector(const TeamObserver& teamObserver, const TeamManager& teamManager, PlanBase* pb);
    virtual ~PlanSelector();

    virtual RunningPlan* getBestSimilarAssignment(const RunningPlan& rp);
    virtual RunningPlan* getBestSimilarAssignment(const RunningPlan& rp, const AgentGrp& robots, double& o_currentUtility);
    virtual bool getPlansForState(
            RunningPlan* planningParent, const ConfAbstractPlanWrapperGrp& wrappers, const AgentGrp& robotIDs, std::vector<RunningPlan*>& o_plans);

    RunningPlan* createRunningPlan(RunningPlan* planningParent, const PlanGrp& plans, const AgentGrp& robotIDs, const RunningPlan* oldRp,
            const PlanType* relevantPlanType, double& o_oldUtility, const ConfAbstractPlanWrapper* wrapper);
    void setGlobalBlackboard(std::shared_ptr<const Blackboard> globalBlackboard);

private:
    static constexpr const char* LOGNAME = "PlanSelector";

    bool getPlansForStateInternal(
            RunningPlan* planningParent, const ConfAbstractPlanWrapperGrp& wrappers, const AgentGrp& robotIDs, std::vector<RunningPlan*>& o_plans);

    PartialAssignmentPool _pap;
    const TeamObserver& _teamObserver;
    const TeamManager& _teamManager;

    PlanBase* _pb;
    std::atomic<const Blackboard*> _globalBlackboard;
};

} /* namespace alica */
