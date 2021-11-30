#include "engine/RuleBook.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/Logger.h"
#include "engine/PlanBase.h"
#include "engine/RunningPlan.h"
#include "engine/UtilityFunction.h"
#include "engine/IAlicaWorldModel.h"
#include "engine/allocationauthority/CycleManager.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/PreCondition.h"
#include "engine/model/State.h"
#include "engine/model/Transition.h"
#include "engine/planselector/PlanSelector.h"
#include "engine/teammanager/TeamManager.h"
#include "engine/syncmodule/SyncModule.h"
#include "engine/constraintmodul/ConditionStore.h"
#include "engine/model/ConfAbstractPlanWrapper.h"



//#define ALICA_DEBUG_LEVEL_ALL
#include <alica_common_config/debug_output.h>
#include <functional>

namespace alica
{

using std::cout;
using std::endl;
/**
 * Basic constructor
 */
RuleBook::RuleBook(AlicaEngine* ae, PlanBase* pb)
        : _tm(ae->getTeamManager())
        , _ps(new PlanSelector(ae, pb))
        , _log(ae->editLog())
        , _pb(pb)
        , _sm(ae->editSyncModul())
        , _changeOccurred(true)
{
    auto reloadFunctionPtr = std::bind(&RuleBook::reload, this, std::placeholders::_1);
    ae->subscribe(reloadFunctionPtr);
    reload(ae->getConfig());
    assert(_ps && _pb);
}

RuleBook::~RuleBook() {}

void RuleBook::init(const IAlicaWorldModel* wm)
{
    _wm = wm;
    _ps->setWorldModel(wm);
}

void RuleBook::reload(const YAML::Node& config)
{
    _maxConsecutiveChanges = config["Alica"]["MaxRuleApplications"].as<int>();
}

/**
 * Implementation of the Init Rule
 * @param masterPlan A Plan
 * @return the shared_ptr of a Runningplan constructed from the given plan
 */
RunningPlan* RuleBook::initialisationRule(const Plan* masterPlan)
{
    ALICA_DEBUG_MSG("RB: Init-Rule called.");
    if (masterPlan->getEntryPoints().size() != 1) {
        AlicaEngine::abort("RB: Masterplan does not have exactly one task!");
    }

    RunningPlan* main = _pb->makeRunningPlan(contextHash(0), masterPlan, nullptr);

    main->setAllocationNeeded(true);

    const EntryPoint* defep = masterPlan->getEntryPoints()[0];
    ActiveAgentIdView agents = _tm.getActiveAgentIds();
    main->editAssignment().setAllToInitialState(agents.begin(), agents.end(), defep);
    main->activate();
    main->useEntryPoint(defep);
    _log.eventOccurred("Init");
    return main;
}

/**
 * Called in every iteration by a RunningPlan to apply rules to it.
 * Will consecutively apply rules until no further changes can be made or
 * maxConsecutiveChanges are made. This method also dictates the sequence in which rules are applied.
 * @param r A shared_ptr of a RunningPlan
 * @return A PlanChange
 */
PlanChange RuleBook::visit(RunningPlan& r)
{
    int changes = 0;
    bool doDynAlloc = true;
    PlanChange changeRecord = PlanChange::NoChange;
    PlanChange msChange = PlanChange::NoChange;
    // obtain modification lock for visited plan
    RunningPlan::ScopedWriteLock lck = r.getWriteLock();

    do {
        msChange = updateChange(msChange, changeRecord);
        changeRecord = PlanChange::NoChange;
        changeRecord = updateChange(changeRecord, synchTransitionRule(r));
        PlanChange transChange = transitionRule(r);
        while (transChange != PlanChange::NoChange && ++changes < _maxConsecutiveChanges) {
            changeRecord = updateChange(changeRecord, transChange);
            transChange = transitionRule(r);
        }
        changeRecord = updateChange(changeRecord, transitionRule(r));
        changeRecord = updateChange(changeRecord, topFailRule(r));
        changeRecord = updateChange(changeRecord, allocationRule(r));
        changeRecord = updateChange(changeRecord, authorityOverrideRule(r));

        if (doDynAlloc) {
            changeRecord = updateChange(changeRecord, dynamicAllocationRule(r));
            doDynAlloc = false;
        }
        changeRecord = updateChange(changeRecord, planAbortRule(r));
        if (changeRecord == PlanChange::FailChange) {
            return PlanChange::FailChange; // allow higher level to react
        }
        changeRecord = updateChange(changeRecord, planRedoRule(r));
        changeRecord = updateChange(changeRecord, planReplaceRule(r));
        // planReplace may retire the current plan.
        if (r.isRetired()) {
            return changeRecord;
        }

        PlanChange propChange = planPropagationRule(r);
        changeRecord = updateChange(changeRecord, propChange);

        if (propChange != PlanChange::NoChange) {
            break; // abort applying rules to this plan as propagation has occurred
        }

    } while (changeRecord != PlanChange::NoChange && ++changes < _maxConsecutiveChanges);
    return msChange;
}

/**
 * Changes the allocation of r to a better one, if one can be found and the plan is currently allowed to change
 * allocation.
 * @param r A shared_ptr of a RunningPlan
 * @return PlanChange
 */
PlanChange RuleBook::dynamicAllocationRule(RunningPlan& r)
{
    assert(!r.isRetired());
    ALICA_DEBUG_MSG("RB: dynAlloc-Rule called.");
    ALICA_DEBUG_MSG("RB: dynAlloc RP \n" << r);

    if (r.isAllocationNeeded() || r.isBehaviour()) {
        return PlanChange::NoChange;
    }
    if (r.getParent() == nullptr) {
        return PlanChange::NoChange; // masterplan excluded
    }
    if (!r.getCycleManagement().mayDoUtilityCheck()) {
        return PlanChange::NoChange;
    }
    const RunningPlan* parent = r.getParent();

    AgentGrp robots;
    parent->getAssignment().getAgentsInState(parent->getActiveState(), robots);
    double curUtil = 0.0;
    RunningPlan* newr = _ps->getBestSimilarAssignment(r, robots, curUtil);

    if (newr == nullptr) {
        return PlanChange::NoChange;
    }
    const Plan* p = static_cast<const Plan*>(r.getActivePlan());

    double possibleUtil = newr->getAssignment().getLastUtilityValue();
    ALICA_DEBUG_MSG("RB: Old U " << curUtil << " | "
                                 << " New U:" << possibleUtil);
    ALICA_DEBUG_MSG_IF(curUtil < -0.99, "#############Assignment is valid?: " << r.getAssignment().isValid() << endl << r);
    ALICA_DEBUG_MSG("RB: New Assignment" << newr->getAssignment() << endl << "RB: Old Assignment" << r.getAssignment());

    if (possibleUtil - curUtil > p->getUtilityThreshold()) {
        // cout << "RB: AllocationDifference::Reason::utility " << endl;
        r.editCycleManagement().setNewAllocDiff(r.getAssignment(), newr->getAssignment(), AllocationDifference::Reason::utility);
        const State* before = r.getActiveState();
        r.adaptAssignment(*newr);
        if (r.getActiveState() != nullptr && r.getActiveState() != before) {
            r.setAllocationNeeded(true);
        }

        ALICA_INFO_MSG("RB: B4 dynChange: Util is " << curUtil << " | "
                                                    << " suggested is " << possibleUtil << " | "
                                                    << " threshold " << p->getUtilityThreshold() << std::endl
                                                    << "RB: DynAlloc in " << p->getName());

        _log.eventOccurred("DynAlloc(", p->getName(), ")");
        return PlanChange::InternalChange;
    }
    return PlanChange::NoChange;
}
/**
 * Adopts an authorative assignment in case the CycleManager of r is in overridden mode
 * @param r A shared_ptr of a RunningPlan
 * @return PlanChange
 */
PlanChange RuleBook::authorityOverrideRule(RunningPlan& r)
{
    assert(!r.isRetired());
    ALICA_DEBUG_MSG("RB: AuthorityOverride-Rule called.");

    if (r.isBehaviour()) {
        return PlanChange::NoChange;
    }
    ALICA_DEBUG_MSG("RB: AuthorityOverride RP \n" << r);

    if (r.getCycleManagement().isOverridden()) {
        if (r.editCycleManagement().applyAssignment()) {
            _log.eventOccurred("AuthorityOverride(", r.getActivePlan()->getName(), ")");
            ALICA_DEBUG_MSG("RB: Authorative set assignment of " << r.getActivePlan()->getName() << " is:" << r.getAssignment());
            return PlanChange::InternalChange;
        }
    }
    return PlanChange::NoChange;
}
/**
 * The abort rule, sets a failure if a failure state is reached, the allocation invalid or the runtimecondition does not
 * hold.
 * @param r A shared_ptr of a RunningPlan
 * @return PlanChange
 */
PlanChange RuleBook::planAbortRule(RunningPlan& r)
{
    assert(!r.isRetired());
    if (r.isFailureHandlingNeeded())
        return PlanChange::NoChange;
    if (r.isBehaviour())
        return PlanChange::NoChange;
    if (r.getStatus() == PlanStatus::Success)
        return PlanChange::NoChange;
    if (!r.getCycleManagement().mayDoUtilityCheck())
        return PlanChange::NoChange;

    if ((r.getActiveState() != nullptr && r.getActiveState()->isFailureState()) || !r.getAssignment().isValid() || !r.isRuntimeConditionValid()) {

        ALICA_DEBUG_MSG("RB: PlanAbort-Rule called.");
        ALICA_DEBUG_MSG("RB: PlanAbort RP \n" << r);
        ALICA_DEBUG_MSG("RB: PlanAbort " << r.getActivePlan()->getName());
        r.addFailure();
        _log.eventOccurred("PAbort(", r.getActivePlan()->getName(), ")");
        return PlanChange::FailChange;
    }
    return PlanChange::NoChange;
}

/**
 * Tries to repair a plan by moving all robots in the current state to the corresponding initial state.
 * @param r A shared_ptr of a RunningPlan
 * @return PlanChange
 */
PlanChange RuleBook::planRedoRule(RunningPlan& r)
{
    assert(!r.isRetired());
    ALICA_DEBUG_MSG("RB: PlanRedoRule-Rule called.");
    ALICA_DEBUG_MSG("RB: PlanRedoRule RP \n" << r);

    if (!r.getParent() || !r.isFailureHandlingNeeded() || r.isBehaviour())
        return PlanChange::NoChange;
    if (r.getFailureCount() != 1)
        return PlanChange::NoChange;
    if (r.getActiveEntryPoint() == nullptr)
        return PlanChange::NoChange;
    if (r.getActiveState() == r.getActiveEntryPoint()->getState()) {
        r.addFailure();
        ALICA_DEBUG_MSG("RB: PlanRedoRule not executed for " << r.getActivePlan()->getName()
                                                             << "- Unable to repair, as the current state is already the initial state.");

        return PlanChange::FailChange;
    }
    r.setFailureHandlingNeeded(false);
    r.deactivateChildren();
    r.clearChildren();
    r.editAssignment().moveAllFromTo(r.getActiveEntryPoint(), r.getActiveState(), r.getActiveEntryPoint()->getState());

    r.useState(r.getActiveEntryPoint()->getState());
    r.setAllocationNeeded(true);

    ALICA_DEBUG_MSG("RB: PlanRedoRule executed for " << r.getActivePlan()->getName());

    _log.eventOccurred("PRedo(", r.getActivePlan()->getName(), ")");
    return PlanChange::InternalChange;
}

/**
 * Tries to repair a failure by removing this plan and triggering a new task allocation.
 * @param r A shared_ptr of a RunningPlan
 * @return PlanChange
 */
PlanChange RuleBook::planReplaceRule(RunningPlan& r)
{
    assert(!r.isRetired());
    ALICA_DEBUG_MSG("RB: PlanReplace-Rule called.");
    ALICA_DEBUG_MSG("RB: PlanReplace RP \n" << r);

    if (!r.getParent() || !r.isFailureHandlingNeeded() || r.isBehaviour())
        return PlanChange::NoChange;
    if (r.getFailureCount() != 2)
        return PlanChange::NoChange;
    RunningPlan* parent = r.getParent();
    {
        RunningPlan::ScopedWriteLock lck = parent->getWriteLock();
        parent->deactivateChildren();
        parent->setFailedChild(r.getActivePlan());
        parent->setAllocationNeeded(true);
        parent->clearChildren();
    }
    r.setFailureHandlingNeeded(false);

    ALICA_DEBUG_MSG("RB: PlanReplace" << r.getActivePlan()->getName());

    _log.eventOccurred("PReplace(", r.getActivePlan()->getName(), ")");
    return PlanChange::FailChange;
}
/**
 * Propagates a failure to the parent in case it couldn't be repaired on this level.
 * @param r A shared_ptr of a RunningPlan
 * @return A PlanChange
 */
PlanChange RuleBook::planPropagationRule(RunningPlan& r)
{
    assert(!r.isRetired());
    ALICA_DEBUG_MSG("RB: PlanPropagation-Rule called.");
    ALICA_DEBUG_MSG("RB: PlanPropagation RP \n" << r);

    if (!r.getParent() || !r.isFailureHandlingNeeded() || r.isBehaviour())
        return PlanChange::NoChange;
    if (r.getFailureCount() < 3)
        return PlanChange::NoChange;
    r.getParent()->addFailure();
    r.setFailureHandlingNeeded(false);

    ALICA_DEBUG_MSG("RB: PlanPropagation " << r.getActivePlan()->getName());

    _log.eventOccurred("PProp(", r.getActivePlan()->getName(), ")");
    return PlanChange::FailChange;
}

/**
 * Allocates agents in the current state within r to sub-plans.
 * @param r A shared_ptr of a RunningPlan
 * @return PlanChange
 */
PlanChange RuleBook::allocationRule(RunningPlan& rp)
{
    assert(!rp.isRetired());
    ALICA_DEBUG_MSG("RB: Allocation-Rule called.");
    ALICA_DEBUG_MSG("RB: Allocation RP \n" << rp);

    if (!rp.isAllocationNeeded()) {
        return PlanChange::NoChange;
    }
    rp.setAllocationNeeded(false);

    AgentGrp agents;
    rp.getAssignment().getAgentsInState(rp.getActiveState(), agents);

    ALICA_DEBUG_MSG(rp.getActiveState()->getConfAbstractPlanWrappers().size() << " Plans in State " << rp.getActiveState()->getName());

    std::vector<RunningPlan*> children;
    bool ok = _ps->getPlansForState(&rp, rp.getActiveState()->getConfAbstractPlanWrappers(), agents, children);
    if (!ok || children.size() < rp.getActiveState()->getConfAbstractPlanWrappers().size()) {
        rp.addFailure();
        ALICA_DEBUG_MSG("RB: PlanAllocFailed " << rp.getActivePlan()->getName());
        return PlanChange::FailChange;
    }
    rp.addChildren(children);

    ALICA_DEBUG_MSG("RB: after add children");
    ALICA_DEBUG_MSG("RB: PlanAlloc " << rp.getActivePlan()->getName());

    if (!children.empty()) {
        _log.eventOccurred("PAlloc(", rp.getActivePlan()->getName(), " in State ", rp.getActiveState()->getName(), ")");
        return PlanChange::InternalChange;
    }
    return PlanChange::NoChange;
}

/**
 * Handles a failure at the top-level plan by resetting everything.
 * @param r A shared_ptr of a RunningPlan
 * @return PlanChange
 */
PlanChange RuleBook::topFailRule(RunningPlan& r)
{
    assert(!r.isRetired());
    ALICA_DEBUG_MSG("RB: TopFail-Rule called.");
    ALICA_DEBUG_MSG("RB: TopFail RP \n" << r);

    if (r.getParent())
        return PlanChange::NoChange;

    if (r.isFailureHandlingNeeded()) {
        r.setFailureHandlingNeeded(false);
        r.clearFailures();
        const EntryPoint* ep = static_cast<const Plan*>(r.getActivePlan())->getEntryPoints()[0];

        r.useEntryPoint(ep);

        r.setAllocationNeeded(true);
        r.editAssignment().clear();
        ActiveAgentIdView agents = _tm.getActiveAgentIds();
        r.editAssignment().setAllToInitialState(agents.begin(), agents.end(), ep);
        r.useState(ep->getState());
        r.clearFailedChildren();

        ALICA_DEBUG_MSG("RB: PlanTopFail " << r.getActivePlan()->getName());

        _log.eventOccurred("TopFail");
        return PlanChange::InternalChange;
    }
    return PlanChange::NoChange;
}

/**
 * The transition rule, moves an agent along a transition to a next state if the corresponding condition holds,
 * flags the RunningPlan for allocation in the next state.
 * Note, in case multiple transitions are eligible, one is chosen implementation dependent.
 * @param r A shared_ptr of a RunningPlan
 * @return PlanChange
 */
PlanChange RuleBook::transitionRule(RunningPlan& r)
{
    assert(!r.isRetired());
    ALICA_DEBUG_MSG("RB: Transition-Rule called.");
    ALICA_DEBUG_MSG("RB: Transition RP \n" << r);

    if (r.getActiveState() == nullptr)
        return PlanChange::NoChange;
    const State* nextState = nullptr;

    for (const Transition* t : r.getActiveState()->getOutTransitions()) {
        if (t->getSynchronisation() != nullptr)
            continue;
        if (t->evalCondition(r, _wm)) {
            nextState = t->getOutState();
            r.editConstraintStore().addCondition(t->getPreCondition());
            break;
        }
    }
    if (nextState == nullptr) {
        return PlanChange::NoChange;
    }

    ALICA_DEBUG_MSG("RB: Transition " << r.getActivePlan()->getName());

    r.moveState(nextState);

    r.setAllocationNeeded(true);
    _log.eventOccurred("Transition(", r.getActivePlan()->getName(), " to State ", r.getActiveState()->getName(), ")");
    if (r.getActiveState()->isSuccessState())
        return PlanChange::SuccesChange;
    else if (r.getActiveState()->isFailureState())
        return PlanChange::FailChange;
    return PlanChange::InternalChange;
}

/**
 * Moves the agent along a synchronized transition, if the corresponding transition holds and the team
 * deems the transition as synchronized.
 * @param rp A shared_ptr of a RunningPlan
 * @return PlanChange
 */
PlanChange RuleBook::synchTransitionRule(RunningPlan& rp)
{
    assert(!rp.isRetired());
    ALICA_DEBUG_MSG("RB: Sync-Rule called.");
    ALICA_DEBUG_MSG("RB: Sync RP \n" << rp);

    if (rp.getActiveState() == nullptr) {
        return PlanChange::NoChange;
    }

    const State* nextState = nullptr;

    for (const Transition* t : rp.getActiveState()->getOutTransitions()) {
        if (t->getSynchronisation() == nullptr) {
            continue;
        }
        if (_sm.isTransitionSuccessfullySynchronised(t)) {
            if (t->evalCondition(rp, _wm)) {
                // we follow the transition, because it holds and is synchronised
                nextState = t->getOutState();
                rp.editConstraintStore().addCondition(t->getPreCondition());
                break;
            } else {
                // adds a new synchronisation process or updates existing
                _sm.setSynchronisation(t, false);
            }
        } else {
            // adds a new synchronisation process or updates existing
            _sm.setSynchronisation(t, t->evalCondition(rp, _wm));
        }
    }
    if (nextState == nullptr) {
        return PlanChange::NoChange;
    }

    rp.moveState(nextState);
    rp.setAllocationNeeded(true);

    ALICA_DEBUG_MSG("RB: Follow synchronised transition in plan " << rp.getActivePlan()->getName());
    _log.eventOccurred("SynchTrans(", rp.getActivePlan()->getName(), ")");

    if (rp.getActiveState()->isSuccessState())
        return PlanChange::SuccesChange;
    else if (rp.getActiveState()->isFailureState())
        return PlanChange::FailChange;
    return PlanChange::InternalChange;
}

/**
 * Combines to PlanChange flags to one, giving priority to Failures.
 * @param cur A PlanChange
 * @param update A PlanChange
 * @return PlanChange
 */
PlanChange RuleBook::updateChange(PlanChange cur, PlanChange update)
{
    if (update != PlanChange::NoChange) {
        _changeOccurred = true;
    }
    if (cur == PlanChange::NoChange) {
        return update;
    }
    if (cur == PlanChange::FailChange) {
        return cur;
    }
    if (cur == PlanChange::InternalChange) {
        if (update != PlanChange::NoChange) {
            return update;
        }
    }
    if (update == PlanChange::FailChange) {
        return update;
    }
    return cur;
}

} // namespace alica
