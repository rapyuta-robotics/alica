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
class AlicaEngine;

/**
 * Implements the task allocation algorithm
 */
class PlanSelector
{
public:
    PlanSelector(AlicaEngine* ae);
    virtual ~PlanSelector();

    virtual RunningPlan* getBestSimilarAssignment(const RunningPlan& rp);
    virtual RunningPlan* getBestSimilarAssignment(const RunningPlan& rp, const AgentGrp& robots);
    virtual bool getPlansForState(
            RunningPlan* planningParent, const AbstractPlanGrp& plans, const AgentGrp& robotIDs, std::vector<RunningPlan*>& o_plans) const;

    RunningPlan* createRunningPlan(
            RunningPlan* planningParent, const PlanGrp& plans, const AgentGrp& robotIDs, const RunningPlan* oldRp, const PlanType* relevantPlanType) const;

private:
    bool getPlansForStateInternal(
            RunningPlan* planningParent, const AbstractPlanGrp& plans, const AgentGrp& robotIDs, std::vector<RunningPlan*>& o_plans) const;

    PartialAssignmentPool _pap;
    TeamObserver* _to;
    AlicaEngine* _ae;
    PlanBase* _pb;
};

} /* namespace alica */
