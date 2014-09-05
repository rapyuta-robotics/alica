/*
 * RuleBook.cpp
 *
 *  Created on: Jun 17, 2014
 *      Author: Paul Panin
 */

#include "engine/rules/RuleBook.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"
#include "engine/RunningPlan.h"
#include "engine/ITeamObserver.h"
#include "engine/model/EntryPoint.h"
#include "engine/logging/Logger.h"
#include "engine/RunningPlan.h"
#include "engine/ISyncModul.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/State.h"
#include "engine/model/Transition.h"
#include "engine/constraintmodul/ConstraintStore.h"
#include "engine/IPlanSelector.h"
#include "engine/collections/StateCollection.h"
#include "engine/allocationauthority/CycleManager.h"
#include "engine/UtilityFunction.h"

#include <SystemConfig.h>

namespace alica
{
	RuleBook::RuleBook()
	{
		AlicaEngine* ae = AlicaEngine::getInstance();
		this->to = ae->getTeamObserver();
		this->ps = ae->getPlanSelector();
		this->sm = ae->getSyncModul();
		this->log = ae->getLog();
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		this->maxConsecutiveChanges = (*sc)["Alica"]->get<int>("Alica.MaxRuleApplications", NULL);
		this->changeOccured = true;
	}

	RuleBook::~RuleBook()
	{
		// TODO Auto-generated destructor stub
	}

	RunningPlan* RuleBook::initialisationRule(Plan* masterPlan)
	{
		if (masterPlan->getEntryPoints().size() != 1)
		{
			AlicaEngine::getInstance()->abort("RB: Masterplan does not have exactly one task!");
		}

		RunningPlan* main = new RunningPlan(masterPlan);
		main->setAssignment(new Assignment(masterPlan));

		main->setAllocationNeeded(true);
		unique_ptr<list<int> > robots = to->getAvailableRobotIds();
		main->setRobotsAvail(move(robots));

		EntryPoint* defep;
		list<EntryPoint*> l;
		transform(masterPlan->getEntryPoints().begin(), masterPlan->getEntryPoints().end(), back_inserter(l),
					[](map<long, EntryPoint*>::value_type& val)
					{	return val.second;});
		for (EntryPoint* e : l)
		{
			defep = e;
			break;
		}
		unique_ptr<list<int> > r = to->getAvailableRobotIds();
		main->getAssignment()->setAllToInitialState(move(r), defep);
		main->setActive(true);
		main->setOwnEntryPoint(defep);
		this->log->eventOccured("Init");
		return main;

	}

	PlanChange RuleBook::visit(RunningPlan* r)
	{
		int changes = 0;
		bool doDynAlloc = true;

		PlanChange changeRecord = PlanChange::NoChange;
		PlanChange msChange = PlanChange::NoChange;

		do
		{

			msChange = updateChange(msChange, changeRecord);
			changeRecord = PlanChange::NoChange;
			changeRecord = updateChange(changeRecord, synchTransitionRule(r));
			PlanChange transChange = transitionRule(r);

			while (transChange != PlanChange::NoChange && ++changes < this->maxConsecutiveChanges)
			{
				changeRecord = updateChange(changeRecord, transChange);
				transChange = transitionRule(r);
			}

			changeRecord = updateChange(changeRecord, transitionRule(r));
			changeRecord = updateChange(changeRecord, topFailRule(r));
			changeRecord = updateChange(changeRecord, allocationRule(r));
			changeRecord = updateChange(changeRecord, authorityOverrideRule(r));

			if (doDynAlloc)
			{
				changeRecord = updateChange(changeRecord, dynamicAllocationRule(r));
				doDynAlloc = false;
			}
			changeRecord = updateChange(changeRecord, planAbortRule(r));
			changeRecord = updateChange(changeRecord, planRedoRule(r));
			changeRecord = updateChange(changeRecord, planReplaceRule(r));

			PlanChange propChange = planPropagationRule(r);
			changeRecord = updateChange(changeRecord, propChange);

			if (propChange != PlanChange::NoChange)
				break; //abort applying rules to this plan as propagation has

		} while (changeRecord != PlanChange::NoChange && ++changes < this->maxConsecutiveChanges);

		return msChange;
	}

	/**
	 * Changes the allocation of r to a better one, if one can be found and the plan is currently allowed to change allocation.
	 * @param r
	 * @return PlanChange
	 */
	PlanChange RuleBook::dynamicAllocationRule(RunningPlan* r)
	{
		if (r->isAllocationNeeded() || r->isBehaviour())
			return PlanChange::NoChange;
		if (r->getParent() == nullptr)
			return PlanChange::NoChange; //masterplan excluded
		if (!r->getCycleManager()->mayDoUtilityCheck())
			return PlanChange::NoChange;

		vector<int> robots;
		copy(r->getParent()->getAssignment()->getRobotStateMapping()->getRobotsInState(r->getActiveState()).begin(),
					r->getParent()->getAssignment()->getRobotStateMapping()->getRobotsInState(r->getActiveState()).end(),
					back_inserter(robots));

		RunningPlan* newr = ps->getBestSimilarAssignment(r, make_shared<vector<int> >(robots));
		if (newr == nullptr)
			return PlanChange::NoChange;
		double curUtil;
		if (!r->evalRuntimeCondition())
			curUtil = -1.0;
		else
			curUtil = r->getPlan()->getUtilityFunction()->eval(r, r);
		double possibleUtil = newr->getAssignment()->getMax();
#if RULE_debug
		cout << "RB: Old U " << curUtil << " | " << " New U:" << possibleUtil << endl;
		cout << "RB: New Assignment" << newr->getAssignment() << endl;
		cout << "RB: Old Assignment" << r->getAssignment() << endl;
#endif

		if (possibleUtil - curUtil > r->getPlan()->getUtilityThreshold())
		{
//			TODO: method
//			r->getCycleManager()->setNewAllocDiff(r, newr->getAssignment(), AllocationDifference.Reason.utility);
			State* before = r->getActiveState();
			r->adaptAssignment(newr);
			if (r->getActiveState() != nullptr && r->getActiveState() != before)
				r->setAllocationNeeded(true);
#if RULE_debug
			cout << "RB: B4 dynChange: Util is " << curUtil << " | " << " suggested is " << possibleUtil << " | " << " threshold " << r->getPlan()->getUtilityThreshold() << endl;
			cout << "RB: DynAlloc" <<r->getPlan()->getName() << endl;
#endif

			log->eventOccured("DynAlloc(" + r->getPlan()->getName() + ")");
			return PlanChange::InternalChange;
		}
		return PlanChange::NoChange;
	}
	/**
	 * Adopts an authorative assignment in case the CycleManager of r is in overridden mode
	 * @param r
	 * @return PlanChange
	 */
	PlanChange RuleBook::authorityOverrideRule(RunningPlan* r)
	{
		if (r->isBehaviour())
			return PlanChange::NoChange;
		if (r->getCycleManager()->isOverridden())
		{
			if (r->getCycleManager()->setAssignment(r))
			{
				log->eventOccured("AuthorityOverride(" + r->getPlan()->getName() + ")");
				return PlanChange::InternalChange;
			}
		}
		return PlanChange::NoChange;
	}
	/**
	 * The abort rule, sets a failure if a failure state is reached, the allocation invalid or the runtimecondition does not hold.
	 * @param r
	 * @return PlanChange
	 */
	PlanChange RuleBook::planAbortRule(RunningPlan* r)
	{
		if (r->getFailHandlingNeeded())
			return PlanChange::NoChange;
		if (r->isBehaviour())
			return PlanChange::NoChange;
		if (r->getStatus() == PlanStatus::Success)
			return PlanChange::NoChange;
		if (!r->getCycleManager()->mayDoUtilityCheck())
			return PlanChange::NoChange;

		if ((r->getActiveState() != nullptr && r->getActiveState()->isFailureState()) || !r->getAssignment()->isValid()
				|| !r->evalRuntimeCondition())
		{
#if RULE_debug
			cout << "RB: PlanAbort" << r.Plan.Name << endl;
#endif
			log->eventOccured("PAbort(" + r->getPlan()->getName() + ")");
			return PlanChange::FailChange;
		}
		return PlanChange::NoChange;
	}

	/**
	 * Tries to repair a plan by moving all robots in the current state to the corresponding initial state.
	 * @param r
	 * @return PlanChange
	 */
	PlanChange RuleBook::planRedoRule(RunningPlan* r)
	{
		if (r->getParent() == nullptr || !r->getFailHandlingNeeded() || r->isBehaviour())
			return PlanChange::NoChange;
		if (r->getFailure() != 1)
			return PlanChange::NoChange;
		if (r->getOwnEntryPoint() == nullptr)
			return PlanChange::NoChange;
		if (r->getActiveState() == r->getOwnEntryPoint()->getState())
		{
			r->addFailure();
			return PlanChange::FailChange;
		}
		r->setFailHandlingNeeded(false);
		r->deactivateChildren();
		r->clearChildren();
		vector<int> robots;
		copy(r->getAssignment()->getRobotStateMapping()->getRobotsInState(r->getActiveState()).begin(),
				r->getAssignment()->getRobotStateMapping()->getRobotsInState(r->getActiveState()).end(),
				back_inserter(robots));
		r->getAssignment()->getRobotStateMapping()->setStates(robots, r->getOwnEntryPoint()->getState());

		r->setActiveState(r->getOwnEntryPoint()->getState());
		r->setAllocationNeeded(true);
#if RULE_debug
		cout << "RB: PlanRedo" << r.Plan.Name << endl;
#endif
		log->eventOccured("PRede(" + r->getPlan()->getName() + ")");
		return PlanChange::InternalChange;
	}

	/**
	 * Tries to repair a failure by removing this plan and triggering a new task allocation.
	 * @param r
	 * @return PlanChange
	 */
	PlanChange RuleBook::planReplaceRule(RunningPlan* r)
	{
		if (r->getParent() == nullptr || !r->getFailHandlingNeeded() || r->isBehaviour())
			return PlanChange::NoChange;
		if (r->getFailure() != 2)
			return PlanChange::NoChange;
		r->getParent()->deactivateChildren();
		r->getParent()->setFailedChild(r->getPlan());
		r->getParent()->setAllocationNeeded(true);
		r->getParent()->clearChildren();
		r->setFailHandlingNeeded(false);

#if RULE_debug
		cout << "RB: PlanReplace" << r.Plan.Name << endl;
#endif
		log->eventOccured("PReplace(" + r->getPlan()->getName() + ")");
		return PlanChange::FailChange;
	}
	/**
	 * Propagates a failure to the parent in case it couldn't be repaired on this level.
	 * @param r
	 * @return
	 */
	PlanChange RuleBook::planPropagationRule(RunningPlan* r)
	{
		if (r->getParent() == nullptr || !r->getFailHandlingNeeded() || r->isBehaviour())
			return PlanChange::NoChange;
		if (r->getFailure() != 3)
			return PlanChange::NoChange;
		r->getParent()->addFailure();

#if RULE_debug
		cout << "RB: PlanPropagation" << r.Plan.Name << endl;
#endif
		log->eventOccured("PProp(" + r->getPlan()->getName() + ")");
		return PlanChange::FailChange;
	}

	/**
	 * Allocates agents in the current state within r to sub-plans.
	 * @param r
	 * @return PlanChange
	 */
	PlanChange RuleBook::allocationRule(RunningPlan* r)
	{
		if (!r->isAllocationNeeded())
		{
			return PlanChange::NoChange;
		}
		r->setAllocationNeeded(false);
		vector<int> robots;
		copy(r->getAssignment()->getRobotStateMapping()->getRobotsInState(r->getActiveState()).begin(),
						r->getAssignment()->getRobotStateMapping()->getRobotsInState(r->getActiveState()).end(),
						back_inserter(robots));
		shared_ptr<list<RunningPlan*> > children = this->ps->getPlansForState(
				r, r->getActiveState()->getPlans(),
				make_shared<vector<int> >(robots));

		if (children->size() == 0 || children->size() < r->getActiveState()->getPlans().size())
		{
			r->addFailure();
#if RULE_debug
			cout << "RB: PlanAllocFailed" << r.Plan.Name << endl;
#endif
			return PlanChange::FailChange;
		}

		r->addChildren(children);
#if RULE_debug
		cout << "RB: PlanAlloc" << r.Plan.Name << endl;
#endif

		if (children->size() > 0)
		{
			log->eventOccured("PAlloc(" + r->getPlan()->getName() + " in State " + r->getActiveState()->getName() + ")");
			return PlanChange::InternalChange;
		}
		return PlanChange::NoChange;
	}

	/**
	 * Handles a failure at the top-level plan by resetting everything.
	 * @param r
	 * @return PlanChnage
	 */
	PlanChange RuleBook::topFailRule(RunningPlan* r)
	{
		if (r->getParent() == nullptr)
			return PlanChange::NoChange;

		if (r->getFailHandlingNeeded())
		{
			r->setFailHandlingNeeded(false);
			r->clearFailures();
			vector<EntryPoint*> epCol;
			transform(((Plan*)r->getPlan())->getEntryPoints().begin(), ((Plan*)r->getPlan())->getEntryPoints().end(),
						back_inserter(epCol), [](map<long, EntryPoint*>::value_type& val)
						{	return val.second;});
			for (EntryPoint* e : epCol)
			{
				r->setOwnEntryPoint(e);
				break;
			}
			r->setAllocationNeeded(true);
			unique_ptr<list<int> > robots = to->getAvailableRobotIds();
			r->setRobotsAvail(move(robots));
			r->getAssignment()->clear();
			r->getAssignment()->setAllToInitialState(move(robots), r->getOwnEntryPoint());
			r->setActiveState(r->getOwnEntryPoint()->getState());
			r->clearFailedChildren();
#if RULE_debug
			cout << "RB: PlanTopFail" << r.Plan.Name << endl;
#endif
			log->eventOccured("TopFail");
			return PlanChange::InternalChange;
		}
		return PlanChange::NoChange;
	}

	/**
	 * The transition rule, moves an agent along a transition to a next state if the corresponding condition holds,
	 * flags the RunningPlan for allocation in the next state.
	 * Note, in case multiple transitions are eligble, one is chosen implementation dependent.
	 * @param r
	 * @return PlanChange
	 */
	PlanChange RuleBook::transitionRule(RunningPlan* r)
	{
		if (r->getActiveState() == nullptr)
			return PlanChange::NoChange;
		State* nextState;

		for (Transition* t : r->getActiveState()->getOutTransitions())
		{
			if (t->getSyncTransition() == nullptr)
				continue;
			if (t->evalCondition(r))
			{
				nextState = t->getOutState();
				r->getConstraintStore()->addCondition(((Condition*)t->getPreCondition()));
				break;
			}
		}
		if (nextState == nullptr)
			return PlanChange::NoChange;
#if RULE_debug
		cout << "RB: SynchTransition" << r.Plan.Name << endl;
#endif
		r->moveState(nextState);

		r->setAllocationNeeded(true);
		log->eventOccured("SynchTrans(" + r->getPlan()->getName() + ")");
		if (r->getActiveState()->isSuccessState())
			return PlanChange::SuccesChange;
		else if (r->getActiveState()->isFailureState())
			return PlanChange::FailChange;
		return PlanChange::InternalChange;
	}

	/**
	 * Moves the agent along a synchronised transition, if the corresponding transition holds and the
	 * deems the transition as synchronised.
	 * @param r
	 * @return PlanChange
	 */
	PlanChange RuleBook::synchTransitionRule(RunningPlan* r)
	{
		if (r->getActiveState() == nullptr)
		{
			return PlanChange::NoChange;
		}
		State* nextState;

		for (Transition* t : r->getActiveState()->getOutTransitions())
		{
			if (t->getSyncTransition() == nullptr)
			{
				continue;
			}
			if (this->sm->followSyncTransition((t)))
			{
				if (t->evalCondition(r))
				{
					nextState = t->getOutState();
					r->getConstraintStore()->addCondition(((Condition*)t->getPreCondition()));
					break;
				}
				else
				{
					this->sm->setSynchronisation(t, false);
				}
			}
			else
			{
				this->sm->setSynchronisation(t, t->evalCondition(r));
			}
		}
		if (nextState == nullptr)
			return PlanChange::NoChange;
#if RULE_debug
		cout << "RB: SynchTransition" << r.Plan.Name << endl;
#endif

		r->moveState(nextState);

		r->setAllocationNeeded(true);
		log->eventOccured("SynchTrans(" + r->getPlan()->getName() + ")");
		if (r->getActiveState()->isSuccessState())
			return PlanChange::SuccesChange;
		else if (r->getActiveState()->isFailureState())
			return PlanChange::FailChange;
		return PlanChange::InternalChange;
	}

	/**
	 * Combines to PlanChange flags to one, giving priority to Failures.
	 * @param cur
	 * @param update
	 * @return PlanChange
	 */
	PlanChange RuleBook::updateChange(PlanChange cur, PlanChange update)
	{
		if (update != PlanChange::NoChange)
		{
			this->changeOccured = true;
		}
		if (cur == PlanChange::NoChange)
		{
			return update;
		}
		if (cur == PlanChange::FailChange)
		{
			return cur;
		}
		if (cur == PlanChange::InternalChange)
		{
			if (update != PlanChange::NoChange)
			{
				return update;
			}
		}
		if (update == PlanChange::FailChange)
		{
			return update;
		}
		return cur;
	}

	/** Getter and Setter **/
	bool RuleBook::isChangeOccured() const
	{
		return changeOccured;
	}

	void RuleBook::setChangeOccured(bool changeOccured)
	{
		this->changeOccured = changeOccured;
	}
}
