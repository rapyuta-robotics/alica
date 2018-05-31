#include "engine/planselector/PlanSelector.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/PlanBase.h"
#include "engine/RunningPlan.h"
#include "engine/TeamObserver.h"
#include "engine/collections/RobotProperties.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/Behaviour.h"
#include "engine/model/BehaviourConfiguration.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/PlanType.h"
#include "engine/model/PlanningProblem.h"
#include "engine/model/State.h"
#include "engine/planselector/PartialAssignment.h"
#include "engine/planselector/PartialAssignmentPool.h"
#include "engine/planselector/TaskAssignment.h"
#include "engine/teammanager/TeamManager.h"

#include "engine/RunningPlan.h"
#include "engine/model/AbstractPlan.h"

#include "engine/Output.h"
#include "engine/model/Task.h"
#include <assert.h>

using std::list;
using std::shared_ptr;
using std::vector;
using std::weak_ptr;

namespace alica
{

PlanSelector::PlanSelector(AlicaEngine* ae, PartialAssignmentPool* pap)
        : _ae(ae);
, _to(ae->getTeamObserver()), _pb(ae->getPlanBase()), _pap(pap) {}

PlanSelector::~PlanSelector() {}

/**
 * Edits data from the old running plan to call the method CreateRunningPlan appropriatly.
 */
shared_ptr<RunningPlan> PlanSelector::getBestSimilarAssignment(shared_ptr<RunningPlan> rp)
{
    assert(!rp->isBehaviour());
    // Reset set index of the partial assignment multiton
    PartialAssignment::reset(pap);
    // CREATE NEW PLAN LIST
    PlanGrp newPlans;
    if (rp->getPlanType() == nullptr) {
        newPlans.push_back(static_cast<const Plan*>(rp->getPlan()));
    } else {
        newPlans = rp->getPlanType()->getPlans();
    }
    // GET ROBOTS TO ASSIGN
    AgentGrp robots;
    rp->getAssignment()->getAllRobots(robots);
    return createRunningPlan(rp->getParent(), newPlans, robots, rp, rp->getPlanType());
}

/**
 * Edits data from the old running plan to call the method CreateRunningPlan appropriately.
 */
shared_ptr<RunningPlan> PlanSelector::getBestSimilarAssignment(shared_ptr<RunningPlan> rp, const AgentGrp& robots)
{
    assert(!rp->isBehaviour());
    // Reset set index of the partial assignment object pool
    PartialAssignment::reset(pap);
    // CREATE NEW PLAN LIST
    PlanGrp newPlans;
    if (rp->getPlanType() == nullptr) {
        newPlans.push_back(static_cast<const Plan*>(rp->getPlan()));
    } else {
        newPlans = rp->getPlanType()->getPlans();
    }
    return createRunningPlan(rp->getParent(), newPlans, robots, rp, rp->getPlanType());
}

/**
 * Solves the task allocation problem for a given state.
 * @param parent The RunningPlan in which task allocation is due for the RunningPlan.ActiveState.
 * @param plans The list of children of the state for which to allocate, a list<AbstractPlan>.
 * @param robotIDs The set of robots or agents, which are available in a shared_ptr<vector<int> >
 * @return A shared_ptr<list<shared_ptr<RunningPlan>>>, encoding the solution.
 */
shared_ptr<list<shared_ptr<RunningPlan>>> PlanSelector::getPlansForState(
        shared_ptr<RunningPlan> planningParent, const AbstractPlanGrp& plans, const AgentGrp& robotIDs)
{
    PartialAssignment::reset(pap);
    return getPlansForStateInternal(planningParent, plans, robotIDs);
}

RunningPlan* PlanSelector::createRunningPlan(
        const RunningPlan* planningParent, const PlanGrp& plans, const AgentGrp& robotIDs, const RunningPlan* oldRp, const PlanType* relevantPlanType)
{
    PlanGrp newPlanList;
    // REMOVE EVERY PLAN WITH TOO GREAT MIN CARDINALITY
    for (const Plan* plan : plans) {
        // CHECK: number of robots < minimum cardinality of this plan
        if (plan->getMinCardinality() > (static_cast<int>(robotIDs.size()) + to->successesInPlan(plan))) {
            ALICA_DEBUG_MSG("PS: RobotIds: " << robotIds << std::endl
                                             << "= " << robotIDs.size() << " IDs are not enough for the plan " << plan->getName() << "!");
        } else {
            // this plan was ok according to its cardinalities, so we can add it
            newPlanList.push_back(plan);
        }
    }
    // WE HAVE NOT ENOUGH ROBOTS TO EXECUTE ANY PLAN
    if (newPlanList.empty()) {
        return nullptr;
    }
    // TASKASSIGNMENT
    TaskAssignment* ta = nullptr;
    const Assignment* oldAss = nullptr;
    RunningPlan* rp;
    if (oldRp == nullptr) {
        // preassign other robots, because we dont need a similar assignment
        rp = _pb->makeRunningPlan(relevantPlanType);
        ta = new TaskAssignment(_ae, newPlanList, robotIDs, true);
    } else {
        // dont preassign other robots, because we need a similar assignment (not the same)
        rp = _pb->makeRunningPlan(oldRp->getPlanType());
        ta = new TaskAssignment(_ae, newPlanList, robotIDs, false);
        oldAss = &oldRp->getAssignment();
    }

    // some variables for the do while loop
    const EntryPoint* ep = nullptr;
    auto localAgentID = _ae->getTeamManager()->getLocalAgentID();
    // PLANNINGPARENT
    rp->setParent(planningParent);
    shared_ptr<list<shared_ptr<RunningPlan>>> rpChildren = nullptr;
    do {
        // ASSIGNMENT
        rp->setAssignment(ta->getNextBestAssignment(oldAss));

        // PLAN (needed for Conditionchecks)
        rp->usePlan(rp->getAssignment()->getPlan());

        ALICA_DEBUG_MSG("PS: rp.Assignment of Plan " << rp->getActivePlan()->getName() << " from " << ownRobProb->getId() << " is: " << rp->getAssignment());

        // CONDITIONCHECK
        if (!rp->evalPreCondition()) {
            continue;
        }
        if (!rp->evalRuntimeCondition()) {
            continue;
        }

        // OWN ENTRYPOINT
        ep = rp->getAssignment()->getEntryPointOfRobot(localAgentID);

        if (ep == nullptr) {
            ALICA_DEBUG_MSG("PS: The robot " << ownRobProb->getName() << "(Id: " << localAgentID << ") is not assigned to enter the plan "
                                             << rp->getPlan()->getName() << " and will IDLE!");

            rp->setActiveState(nullptr);
            rp->setOwnEntryPoint(nullptr);
            return rp; // If we return here, this robot will idle (no ep at rp)
        } else {
            // assign found EntryPoint (this robot dont idle)
            rp->setOwnEntryPoint(ep);
        }
        // ACTIVE STATE set by RunningPlan
        if (oldRp == nullptr) {
            // RECURSIVE PLANSELECTING FOR NEW STATE
            const AgentGrp* robots = rp->getAssignment()->getRobotsWorking(ep);
            assert(robots != nullptr);
            rpChildren = getPlansForStateInternal(rp, rp->getActiveState()->getPlans(), *robots);
        } else {
            ALICA_DEBUG_MSG("PS: no recursion due to utilitycheck");
            // Don't calculate children, because we have an
            // oldRp -> we just replace the oldRp
            // (not its children -> this will happen in an extra call)
            break;
        }
    } while (rpChildren == nullptr);
    // WHEN WE GOT HERE, THIS ROBOT WONT IDLE AND WE HAVE A
    // VALID ASSIGNMENT, WHICH PASSED ALL RUNTIME CONDITIONS
    if (rpChildren != nullptr && !rpChildren.empty()) // c# rpChildren != null
    {
        ALICA_DEBUG_MSG("PS: Set child -> parent reference");
        rp->addChildren(rpChildren);
    }

    ALICA_DEBUG_MSG("PS: Created RunningPlan: \n" << rp);

    delete ta;
    return rp; // If we return here, this robot is normal assigned
}

std::vector<RunningPlan*> PlanSelector::getPlansForStateInternal(RunningPlan* planningParent, const AbstractPlanGrp& plans, const AgentGrp& robotIDs)
{
    std::vector<RunningPlan*> rps;

    ALICA_DEBUG_MSG("<######PS: GetPlansForState: Parent:" << (planningParent != nullptr ? planningParent->getPlan()->getName() : "null")
                                                           << " plan count: " << plans.size() << " robot count: " << robotIDs.size() << " ######>");
    RunningPlan* rp = nullptr;

    // TODO: reintegrate PlanningProblems:
    for (const AbstractPlan* ap : plans) {
        // BEHAVIOUR CONFIGURATION
        if (const BehaviourConfiguration* bc = dynamic_cast<const BehaviourConfiguration*>(ap)) {
            rp = _pb->makeRunningPlan(bc);
            // A BehaviourConfiguration is a Plan too (in this context)
            rp->usePlan(bc);
            rps->push_back(rp);
            rp->setParent(&planningParent);

            ALICA_DEBUG_MSG("PS: Added Behaviour " << bc->getBehaviour()->getName());

        } else if (const Plan* p = dynamic_cast<const Plan*>(ap)) { // PLAN
            rp = createRunningPlan(planningParent, {p}, robotIDs, nullptr, nullptr);
            if (!rp) {
                ALICA_DEBUG_MSG("PS: It was not possible to create a RunningPlan for the Plan " << p->getName() << "!");
                return nullptr;
            }
            ps->push_back(rp);
        } else if (const PlanType* pt = dynamic_cast<const PlanType*>(ap)) { // PLANTYPE
            rp = createRunningPlan(planningParent, pt->getPlans(), robotIDs, nullptr, pt);
            if (!rp) {
                ALICA_INFO_MSG("PS: It was not possible to create a RunningPlan for the Plan " << pt->getName() << "!");
                return nullptr;
            }
            rps->push_back(rp);
        }
    }
    return rps;
}

} /* namespace alica */
