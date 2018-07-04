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
    PlanSelector(AlicaEngine* ae, PlanBase* pb);
    virtual ~PlanSelector();

    virtual RunningPlan* getBestSimilarAssignment(const RunningPlan& rp);
    virtual RunningPlan* getBestSimilarAssignment(const RunningPlan& rp, const AgentGrp& robots, double& o_currentUtility);
    virtual bool getPlansForState(RunningPlan* planningParent, const AbstractPlanGrp& plans, const AgentGrp& robotIDs, std::vector<RunningPlan*>& o_plans);

    RunningPlan* createRunningPlan(RunningPlan* planningParent, const PlanGrp& plans, const AgentGrp& robotIDs, const RunningPlan* oldRp,
            const PlanType* relevantPlanType, double& o_oldUtility);

private:
    bool getPlansForStateInternal(RunningPlan* planningParent, const AbstractPlanGrp& plans, const AgentGrp& robotIDs, std::vector<RunningPlan*>& o_plans);

    PartialAssignmentPool _pap;
    TeamObserver* _to;
    AlicaEngine* _ae;
    PlanBase* _pb;
};

} /* namespace alica */
