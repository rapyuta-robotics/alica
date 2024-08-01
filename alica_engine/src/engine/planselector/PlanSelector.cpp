#include "engine/planselector/PlanSelector.h"
#include "engine/Assignment.h"
#include "engine/Output.h"
#include "engine/PlanBase.h"
#include "engine/RunningPlan.h"
#include "engine/TeamObserver.h"
#include "engine/collections/RobotProperties.h"
#include "engine/collections/SuccessCollection.h"
#include "engine/logging/Logging.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/Behaviour.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/PlanType.h"
#include "engine/model/State.h"
#include "engine/model/Task.h"
#include "engine/planselector/PartialAssignment.h"
#include "engine/planselector/PartialAssignmentPool.h"
#include "engine/planselector/TaskAssignmentProblem.h"
#include "engine/teammanager/TeamManager.h"

#include <assert.h>

using std::vector;

namespace alica
{
namespace
{
constexpr int POOL_SIZE = 10100;
}

PlanSelector::PlanSelector(const TeamObserver& teamObserver, const TeamManager& teamManager, PlanBase* pb)
        : _pap(POOL_SIZE)
        , _teamManager(teamManager)
        , _teamObserver(teamObserver)
        , _pb(pb)
{
    assert(_pb);
}

PlanSelector::~PlanSelector() {}

/**
 * Edits data from the old running plan to call the method CreateRunningPlan appropriately.
 */
RunningPlan* PlanSelector::getBestSimilarAssignment(const RunningPlan& rp)
{
    // GET ROBOTS TO ASSIGN
    AgentGrp robots;
    rp.getAssignment().getAllAgents(robots);
    double oldUtil;
    return getBestSimilarAssignment(rp, robots, oldUtil);
}

/**
 * Edits data from the old running plan to call the method CreateRunningPlan appropriately.
 */
RunningPlan* PlanSelector::getBestSimilarAssignment(const RunningPlan& rp, const AgentGrp& robots, double& o_currentUtility)
{
    assert(!rp.isBehaviour());
    // Reset set index of the partial assignment object pool
    _pap.reset();
    try {
        if (rp.getPlanType() == nullptr) {
            return createRunningPlan(
                    rp.getParent(), {static_cast<const Plan*>(rp.getActivePlan())}, robots, &rp, nullptr, o_currentUtility, rp.getConfAbstractPlanWrapper());
        } else {
            return createRunningPlan(
                    rp.getParent(), rp.getPlanType()->getPlans(), robots, &rp, rp.getPlanType(), o_currentUtility, rp.getConfAbstractPlanWrapper());
        }
    } catch (const PoolExhaustedException& pee) {
        Logging::logError(LOGNAME) << pee.what();
        _pap.increaseSize();
        return nullptr;
    }
}

/**
 * Solves the task allocation problem for a given state.
 */
bool PlanSelector::getPlansForState(
        RunningPlan* planningParent, const ConfAbstractPlanWrapperGrp& wrappers, const AgentGrp& robotIDs, std::vector<RunningPlan*>& o_plans)
{
    _pap.reset();
    try {
        return getPlansForStateInternal(planningParent, wrappers, robotIDs, o_plans);
    } catch (const PoolExhaustedException& pee) {
        Logging::logError(LOGNAME) << pee.what();
        _pap.increaseSize();
        return false;
    }
}

RunningPlan* PlanSelector::createRunningPlan(RunningPlan* planningParent, const PlanGrp& plans, const AgentGrp& robotIDs, const RunningPlan* oldRp,
        const PlanType* relevantPlanType, double& o_oldUtility, const ConfAbstractPlanWrapper* wrapper)
{
    PlanGrp newPlanList;
    // REMOVE EVERY PLAN WITH TOO GREAT MIN CARDINALITY
    for (const Plan* plan : plans) {
        // CHECK: number of robots < minimum cardinality of this plan
        if (plan->getMinCardinality() > (static_cast<int>(robotIDs.size()) + _teamObserver.successesInPlan(plan))) {
            Logging::logDebug(LOGNAME) << "AgentIds: " << robotIDs << " = " << robotIDs.size() << " IDs are not enough for the plan " << plan->getName() << "!";
        } else {
            // this plan was ok according to its cardinalities, so we can add it
            newPlanList.push_back(plan);
        }
    }
    // WE HAVE NOT ENOUGH ROBOTS TO EXECUTE ANY PLAN
    if (newPlanList.empty()) {
        return nullptr;
    }

    TaskAssignmentProblem ta(_teamObserver, _teamManager, newPlanList, robotIDs, _pap, _globalBlackboard);
    const Assignment* oldAss = nullptr;
    RunningPlan* rp;
    if (oldRp == nullptr) {
        // preassign other robots, because we dont need a similar assignment
        rp = _pb->makeRunningPlan(relevantPlanType, wrapper);
        ta.preassignOtherAgents();
    } else {
        if (!oldRp->getAssignment().isValid() || !oldRp->isRuntimeConditionValid()) {
            o_oldUtility = -1.0;
        } else {
            assert(!oldRp->isBehaviour());
            PartialAssignment* ptemp = _pap.getNext();
            const Plan* oldPlan = static_cast<const Plan*>(oldRp->getActivePlan());
            ptemp->prepare(oldPlan, &ta);
            oldRp->getAssignment().fillPartial(*ptemp);
            o_oldUtility = oldPlan->getUtilityFunction()->eval(ptemp, &oldRp->getAssignment(), _globalBlackboard).getMax();
        }
        // dont preassign other robots, because we need a similar assignment (not the same)
        rp = _pb->makeRunningPlan(oldRp->getPlanType(), wrapper);
        oldAss = &oldRp->getAssignment();
    }

    // some variables for the do while loop
    const AgentId localAgentID = _teamManager.getLocalAgentID();
    // PLANNINGPARENT
    rp->setParent(planningParent);
    std::vector<RunningPlan*> rpChildren;
    bool found = false;
    AgentGrp agents;
    do {
        rpChildren.clear();
        // ASSIGNMENT
        rp->setAssignment(ta.getNextBestAssignment(oldAss));

        if (rp->getAssignment().getPlan() == nullptr) {
            return nullptr;
        }
        // PLAN (needed for Conditionchecks)
        rp->usePlan(rp->getAssignment().getPlan());

        // CONDITIONCHECK
        if (!rp->evalPreCondition()) {
            continue;
        }
        if (!rp->isRuntimeConditionValid()) {
            continue;
        }

        // OWN ENTRYPOINT
        const EntryPoint* ep = rp->getAssignment().getEntryPointOfAgent(localAgentID);

        if (ep == nullptr) {
            Logging::logDebug(LOGNAME) << "The agent "
                                       << "(Id: " << localAgentID << ") is not assigned to enter the plan " << rp->getActivePlan()->getName()
                                       << " and will IDLE!";

            rp->useState(nullptr);
            rp->useEntryPoint(nullptr);
            return rp; // If we return here, this robot will idle (no ep at rp)
        } else {
            // assign found EntryPoint (this robot dont idle)
            rp->useEntryPoint(ep);
        }
        // ACTIVE STATE set by RunningPlan
        if (oldRp == nullptr) {
            // RECURSIVE PLANSELECTING FOR NEW STATE
            agents.clear();
            rp->getAssignment().getAgentsWorking(ep, agents);

            found = getPlansForStateInternal(rp, rp->getActiveState()->getConfAbstractPlanWrappers(), agents, rpChildren);
        } else {
            // Don't calculate children, because we have an
            // oldRp -> we just replace the oldRp
            // (not its children -> this will happen in an extra call)
            break;
        }
    } while (!found);
    // WHEN WE GOT HERE, THIS ROBOT WONT IDLE AND WE HAVE A
    // VALID ASSIGNMENT, WHICH PASSED ALL RUNTIME CONDITIONS
    if (found && !rpChildren.empty()) {
        rp->addChildren(rpChildren);
    }

    return rp; // If we return here, this robot is normal assigned
}

bool PlanSelector::getPlansForStateInternal(
        RunningPlan* planningParent, const ConfAbstractPlanWrapperGrp& wrappers, const AgentGrp& robotIDs, std::vector<RunningPlan*>& o_plans)
{
    for (const ConfAbstractPlanWrapper* wrapper : wrappers) {
        const AbstractPlan* ap = wrapper->getAbstractPlan();
        if (const Behaviour* beh = dynamic_cast<const Behaviour*>(ap)) {
            RunningPlan* rp = _pb->makeRunningPlan(beh, wrapper);
            // A Behaviour is a Plan too (in this context)
            rp->usePlan(beh);
            o_plans.push_back(rp);
            rp->setParent(planningParent);
        } else if (const Plan* p = dynamic_cast<const Plan*>(ap)) {
            double zeroValue;
            RunningPlan* rp = createRunningPlan(planningParent, {p}, robotIDs, nullptr, nullptr, zeroValue, wrapper);
            if (!rp) {
                Logging::logDebug(LOGNAME) << "It was not possible to create a RunningPlan for the Plan " << p->getName() << "!";
                return false;
            }
            o_plans.push_back(rp);
        } else if (const PlanType* pt = dynamic_cast<const PlanType*>(ap)) {
            double zeroVal;
            RunningPlan* rp = createRunningPlan(planningParent, pt->getPlans(), robotIDs, nullptr, pt, zeroVal, wrapper);
            if (!rp) {
                Logging::logInfo(LOGNAME) << "It was not possible to create a RunningPlan for the Plan " << pt->getName() << "!";
                return false;
            }
            o_plans.push_back(rp);
        }
    }
    return true;
}

void PlanSelector::setGlobalBlackboard(std::shared_ptr<const Blackboard> globalBlackboard)
{
    _globalBlackboard = globalBlackboard.get();
}

} /* namespace alica */
