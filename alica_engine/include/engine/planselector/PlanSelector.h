#pragma once

//#define PSDEBUG

#include <list>
#include <memory>
#include <sstream>
#include <unordered_set>
#include <vector>

#include "engine/Types.h"

namespace supplementary
{
class AgentID;
}

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
class PartialAssignmentPool;

/**
 * Implements the task allocation algorithm
 */
class PlanSelector
{
public:
    PlanSelector(AlicaEngine* ae, PartialAssignmentPool* pap);
    virtual ~PlanSelector();

    virtual std::shared_ptr<RunningPlan> getBestSimilarAssignment(std::shared_ptr<RunningPlan> rp);
    virtual std::shared_ptr<RunningPlan> getBestSimilarAssignment(std::shared_ptr<RunningPlan> rp, const AgentGrp& robots);
    virtual std::shared_ptr<std::list<std::shared_ptr<RunningPlan>>> getPlansForState(
            std::shared_ptr<RunningPlan> planningParent, const AbstractPlanGrp& plans, const AgentGrp& robotIDs);

    RunningPlan* createRunningPlan(
            const RunningPlan&* planningParent, const PlanGrp& plans, const AgentGrp& robotIDs, const RunningPlan* oldRp, const PlanType* relevantPlanType);

private:
    PartialAssignmentPool* _pap;
    TeamObserver* _to;
    AlicaEngine* _ae;
    PlanBase* _pb;
    std::vector<RunningPlan*> getPlansForStateInternal(RunningPlan* planningParent, const AbstractPlanGrp& plans, const AgentGrp& robotIDs);
};

} /* namespace alica */
