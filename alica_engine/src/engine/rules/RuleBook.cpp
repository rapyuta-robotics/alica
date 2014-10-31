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
	/**
	 * Basic constructor
	 */
	RuleBook::RuleBook(AlicaEngine* ae)
	{
		this->ae = ae;
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

	/**
	 * Implementation of the Init Rule
	 * @param masterPlan A Plan
	 * @return the shared_ptr of a Runningplan constructed from the given plan
	 */
	shared_ptr<RunningPlan> RuleBook::initialisationRule(Plan* masterPlan)
	{
#ifdef RULE_debug
		cout << "RB: Init-Rule called." << endl;
#endif
		if (masterPlan->getEntryPoints().size() != 1)
		{
			ae->abort("RB: Masterplan does not have exactly one task!");
		}

		shared_ptr<RunningPlan> main = make_shared<RunningPlan>(ae, masterPlan);
		main->setAssignment(make_shared<Assignment>(masterPlan));

		main->setAllocationNeeded(true);
		main->setRobotsAvail(move(to->getAvailableRobotIds()));

		EntryPoint* defep = nullptr;
		list<EntryPoint*> l;
		defep = masterPlan->getEntryPoints().begin()->second;
		main->getAssignment()->setAllToInitialState(move(to->getAvailableRobotIds()), defep);
		main->activate();
		main->setOwnEntryPoint(defep);
		this->log->eventOccured("Init");
		return main;

	}

	/**
	 * Called in every iteration by a RunningPlan to apply rules to it.
	 * Will consecutively apply rules until no further changes can be made or
	 * maxConsecutiveChangesare made. This method also dictates the sequence in which rules are applied.
	 * @param r A shared_ptr of a RunningPlan
	 * @return A PlanChange
	 */
	PlanChange RuleBook::visit(shared_ptr<RunningPlan> r)
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
	 * @param r A shared_ptr of a RunningPlan
	 * @return PlanChange
	 */
	PlanChange RuleBook::dynamicAllocationRule(shared_ptr<RunningPlan> r)
	{
#ifdef RULE_debug
		cout << "RB: dynAlloc-Rule called." << endl;
		cout << "RB: dynAlloc RP \n" << r->toString() << endl;
#endif
		if (r->isAllocationNeeded() || r->isBehaviour())
		{
			return PlanChange::NoChange;
		}
		if (r->getParent().expired())
		{
			return PlanChange::NoChange; //masterplan excluded
		}
		if (!r->getCycleManagement()->mayDoUtilityCheck())
		{
			return PlanChange::NoChange;
		}

		auto temp = r->getParent().lock();
		vector<int> robots = vector<int>(temp->getAssignment()->getRobotStateMapping()->getRobotsInState(temp->getActiveState()).size());
		copy(temp->getAssignment()->getRobotStateMapping()->getRobotsInState(temp->getActiveState()).begin(),
					temp->getAssignment()->getRobotStateMapping()->getRobotsInState(temp->getActiveState()).end(),
					robots.begin());
		shared_ptr<RunningPlan> newr = ps->getBestSimilarAssignment(r, make_shared<vector<int> >(robots));
		if (newr == nullptr)
			return PlanChange::NoChange;
		double curUtil;
		if (!r->evalRuntimeCondition()) {
			curUtil = -1.0;
		}
		else {
			curUtil = r->getPlan()->getUtilityFunction()->eval(r, r);
		}
		double possibleUtil = newr->getAssignment()->getMax();
#ifdef RULE_debug
		cout << "RB: Old U " << curUtil << " | " << " New U:" << possibleUtil << endl;
		if(curUtil < -0.99) {
			cout << "#############Assignment is valid?: " << r->getAssignment()->isValid() << endl;
			cout << r->toString() << endl;
		}
		cout << "RB: New Assignment" << newr->getAssignment()->toString() << endl;
		cout << "RB: Old Assignment" << r->getAssignment()->toString() << endl;
//remove comments
#endif

		if (possibleUtil - curUtil > r->getPlan()->getUtilityThreshold())
		{
			cout << "RB: AllocationDifference::Reason::utility " << endl;
			r->getCycleManagement()->setNewAllocDiff(r->getAssignment(), newr->getAssignment(), AllocationDifference::Reason::utility);
			State* before = r->getActiveState();
			r->adaptAssignment(newr);
			if (r->getActiveState() != nullptr && r->getActiveState() != before)
				r->setAllocationNeeded(true);
#ifdef RULE_debug
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
	 * @param r A shared_ptr of a RunningPlan
	 * @return PlanChange
	 */
	PlanChange RuleBook::authorityOverrideRule(shared_ptr<RunningPlan> r)
	{
//#ifdef RULE_debug
		cout << "RB: AuthorityOverride-Rule called." << endl;
		cout << "RB: AuthorityOverride RP \n" << r->toString() << endl;
//#endif
		if (r->isBehaviour())
			return PlanChange::NoChange;
		cout << "CM: overridden " << r->getCycleManagement()->isOverridden() << endl;
		if (r->getCycleManagement()->isOverridden())
		{
			if (r->getCycleManagement()->setAssignment(r))
			{
				log->eventOccured("AuthorityOverride(" + r->getPlan()->getName() + ")");
				return PlanChange::InternalChange;
			}
		}
		return PlanChange::NoChange;
	}
	/**
	 * The abort rule, sets a failure if a failure state is reached, the allocation invalid or the runtimecondition does not hold.
	 * @param r A shared_ptr of a RunningPlan
	 * @return PlanChange
	 */
	PlanChange RuleBook::planAbortRule(shared_ptr<RunningPlan> r)
	{
#ifdef RULE_debug
		cout << "RB: PlanAbort-Rule called." << endl;
		cout << "RB: PlanAbort RP \n" << r->toString() << endl;
#endif
		if (r->getFailHandlingNeeded())
			return PlanChange::NoChange;
		if (r->isBehaviour())
			return PlanChange::NoChange;
		if (r->getStatus() == PlanStatus::Success)
			return PlanChange::NoChange;
		if (!r->getCycleManagement()->mayDoUtilityCheck())
			return PlanChange::NoChange;

		if ((r->getActiveState() != nullptr && r->getActiveState()->isFailureState()) || !r->getAssignment()->isValid()
				|| !r->evalRuntimeCondition())
		{
#ifdef RULE_debug
			cout << "RB: PlanAbort" << r->getPlan()->getName() << endl;
#endif
			r->addFailure();
			log->eventOccured("PAbort(" + r->getPlan()->getName() + ")");
			return PlanChange::FailChange;
		}
		return PlanChange::NoChange;
	}

	/**
	 * Tries to repair a plan by moving all robots in the current state to the corresponding initial state.
	 * @param r A shared_ptr of a RunningPlan
	 * @return PlanChange
	 */
	PlanChange RuleBook::planRedoRule(shared_ptr<RunningPlan> r)
	{
#ifdef RULE_debug
		cout << "RB: PlanRedoRule-Rule called." << endl;
		cout << "RB: PlanRedoRule RP \n" << r->toString() << endl;
#endif
		if (r->getParent().expired() || !r->getFailHandlingNeeded() || r->isBehaviour())
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
		vector<int> robots(r->getAssignment()->getRobotStateMapping()->getRobotsInState(r->getActiveState()).size());
		copy(r->getAssignment()->getRobotStateMapping()->getRobotsInState(r->getActiveState()).begin(),
				r->getAssignment()->getRobotStateMapping()->getRobotsInState(r->getActiveState()).end(),
				robots.begin()); // backinserter

		r->getAssignment()->getRobotStateMapping()->setStates(robots, r->getOwnEntryPoint()->getState());

		r->setActiveState(r->getOwnEntryPoint()->getState());
		r->setAllocationNeeded(true);
#ifdef RULE_debug
		cout << "RB: PlanRedo" << r->getPlan()->getName() << endl;
#endif
		log->eventOccured("PRede(" + r->getPlan()->getName() + ")");
		return PlanChange::InternalChange;
	}

	/**
	 * Tries to repair a failure by removing this plan and triggering a new task allocation.
	 * @param r A shared_ptr of a RunningPlan
	 * @return PlanChange
	 */
	PlanChange RuleBook::planReplaceRule(shared_ptr<RunningPlan> r)
	{
#ifdef RULE_debug
		cout << "RB: PlanReplace-Rule called." << endl;
		cout << "RB: PlanReplace RP \n" << r->toString() << endl;
#endif
		if (r->getParent().expired()|| !r->getFailHandlingNeeded() || r->isBehaviour())
			return PlanChange::NoChange;
		if (r->getFailure() != 2)
			return PlanChange::NoChange;
		auto temp = r->getParent().lock();
		temp->deactivateChildren();
		temp->setFailedChild(r->getPlan());
		temp->setAllocationNeeded(true);
		temp->clearChildren();
		r->setFailHandlingNeeded(false);

#ifdef RULE_debug
		cout << "RB: PlanReplace" << r->getPlan()->getName() << endl;
#endif
		log->eventOccured("PReplace(" + r->getPlan()->getName() + ")");
		return PlanChange::FailChange;
	}
	/**
	 * Propagates a failure to the parent in case it couldn't be repaired on this level.
	 * @param r A shared_ptr of a RunningPlan
	 * @return A PlanChange
	 */
	PlanChange RuleBook::planPropagationRule(shared_ptr<RunningPlan> r)
	{
#ifdef RULE_debug
		cout << "RB: PlanPropagation-Rule called." << endl;
		cout << "RB: PlanPropagation RP \n" << r->toString() << endl;
#endif
		if (r->getParent().expired() || !r->getFailHandlingNeeded() || r->isBehaviour())
			return PlanChange::NoChange;
		if (r->getFailure() < 3)
			return PlanChange::NoChange;
		r->getParent().lock()->addFailure();
		r->setFailHandlingNeeded(false);

#ifdef RULE_debug
		cout << "RB: PlanPropagation" << r->getPlan()->getName() << endl;
#endif
		log->eventOccured("PProp(" + r->getPlan()->getName() + ")");
		return PlanChange::FailChange;
	}

	/**
	 * Allocates agents in the current state within r to sub-plans.
	 * @param r A shared_ptr of a RunningPlan
	 * @return PlanChange
	 */
	PlanChange RuleBook::allocationRule(shared_ptr<RunningPlan> r)
	{
#ifdef RULE_debug
		cout << "RB: Allocation-Rule called." << endl;
		cout << "RB: Allocation RP \n" << r->toString() << endl;
#endif
		if (!r->isAllocationNeeded())
		{
			return PlanChange::NoChange;
		}
		r->setAllocationNeeded(false);

		shared_ptr<vector<int> > robots = make_shared<vector<int> >(r->getAssignment()->getRobotStateMapping()->getRobotsInState(r->getActiveState()).size());
		copy(r->getAssignment()->getRobotStateMapping()->getRobotsInState(r->getActiveState()).begin(),
						r->getAssignment()->getRobotStateMapping()->getRobotsInState(r->getActiveState()).end(),
						robots->begin()); //Was before: back_inserter(*robots)
//		auto iter = r->getAssignment()->getRobotStateMapping()->getRobotsInState(r->getActiveState()).begin();
//		for (int i = 0; i < robots->size();i++)
//		{
//			robots->at(i) = *iter;
//			iter++;
//		}
#ifdef RULE_debug
		cout << "RB: There are " << r->getActiveState()->getPlans().size() << " Plans in State " << r->getActiveState()->getName() << endl;
#endif
		shared_ptr<list<shared_ptr<RunningPlan>> > children = this->ps->getPlansForState(
				r, &r->getActiveState()->getPlans(),
				robots);
		if (children == nullptr || children->size() < r->getActiveState()->getPlans().size())
		{
			r->addFailure();
#ifdef RULE_debug
			cout << "RB: PlanAllocFailed " << r->getPlan()->getName() << endl;
#endif
			return PlanChange::FailChange;
		}
		r->addChildren(children);
#ifdef RULE_debug
		cout << "RB: after add children" << endl;
		cout << "RB: PlanAlloc " <<  r->getPlan()->getName() << endl;
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
	 * @param r A shared_ptr of a RunningPlan
	 * @return PlanChnage
	 */
	PlanChange RuleBook::topFailRule(shared_ptr<RunningPlan> r)
	{
#ifdef RULE_debug
		cout << "RB: TopFail-Rule called." << endl;
		cout << "RB: TopFail RP \n" << r->toString() << endl;
#endif
		if (!r->getParent().expired())
			return PlanChange::NoChange;

		if (r->getFailHandlingNeeded())
		{
			cerr << "PB: TopFailed" << endl;
			r->setFailHandlingNeeded(false);
			r->clearFailures();

			r->setOwnEntryPoint(((Plan*)r->getPlan())->getEntryPoints().begin()->second);

			r->setAllocationNeeded(true);
			r->setRobotsAvail(move(to->getAvailableRobotIds()));
			r->getAssignment()->clear();
			r->getAssignment()->setAllToInitialState(move(to->getAvailableRobotIds()), r->getOwnEntryPoint());
			r->setActiveState(r->getOwnEntryPoint()->getState());
			r->clearFailedChildren();
#ifdef RULE_debug
			cout << "RB: PlanTopFail" << r->getPlan()->getName() << endl;
#endif
			log->eventOccured("TopFail");
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
	PlanChange RuleBook::transitionRule(shared_ptr<RunningPlan> r)
	{
#ifdef RULE_debug
		cout << "RB: Transition-Rule called." << endl;
		cout << "RB: Transition RP \n" << r->toString() << endl;
#endif
		if (r->getActiveState() == nullptr)
			return PlanChange::NoChange;
		State* nextState = nullptr;

		for (Transition* t : r->getActiveState()->getOutTransitions())
		{
			if (t->getSyncTransition() != nullptr)
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
#ifdef RULE_debug
		cout << "RB: SynchTransition" << r->getPlan()->getName() << endl;
#endif
		r->moveState(nextState);

		r->setAllocationNeeded(true);
		log->eventOccured("Transition(" + r->getPlan()->getName() + "to State" + r->getActiveState()->getName() + ")");
		if (r->getActiveState()->isSuccessState())
			return PlanChange::SuccesChange;
		else if (r->getActiveState()->isFailureState())
			return PlanChange::FailChange;
		return PlanChange::InternalChange;
	}

	/**
	 * Moves the agent along a synchronized transition, if the corresponding transition holds and the
	 * deems the transition as synchronized.
	 * @param r A shared_ptr of a RunningPlan
	 * @return PlanChange
	 */
	PlanChange RuleBook::synchTransitionRule(shared_ptr<RunningPlan> r)
	{
#ifdef RULE_debug
		cout << "RB: Sync-Rule called." << endl;
		cout << "RB: Sync RP \n" << r->toString() << endl;
#endif
		if (r->getActiveState() == nullptr)
		{
			return PlanChange::NoChange;
		}

		State* nextState = nullptr;

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
#ifdef RULE_debug
		cout << "RB: SynchTransition" << r->getPlan()->getName()<< endl;
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
	 * @param cur A PlanChange
	 * @param update A PlanChange
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
