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

    virtual RunningPlan* getBestSimilarAssignment(const RunningPlan& rp, const IAlicaWorldModel& wm);
    virtual RunningPlan* getBestSimilarAssignment(const RunningPlan& rp, const AgentGrp& robots, double& o_currentUtility, const IAlicaWorldModel& wm);
    virtual bool getPlansForState(RunningPlan* planningParent, const ConfAbstractPlanWrapperGrp& wrappers, const AgentGrp& robotIDs, std::vector<RunningPlan*>& o_plans, const IAlicaWorldModel& wm);

    RunningPlan* createRunningPlan(RunningPlan* planningParent, const PlanGrp& plans, const Configuration* configuration, const AgentGrp& robotIDs, const RunningPlan* oldRp,
            const PlanType* relevantPlanType, double& o_oldUtility, const IAlicaWorldModel& wm);

private:
    bool getPlansForStateInternal(RunningPlan* planningParent, const ConfAbstractPlanWrapperGrp& wrappers, const AgentGrp& robotIDs, std::vector<RunningPlan*>& o_plans, const IAlicaWorldModel& wm);

    PartialAssignmentPool _pap;
    AlicaEngine* _ae;
    PlanBase* _pb;
};

} /* namespace alica */
