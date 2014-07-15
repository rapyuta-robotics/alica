/*
 * PlanSelector.cpp
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#include <engine/planselector/PlanSelector.h>
#include "engine/AlicaEngine.h"
#include "engine/teamobserver/TeamObserver.h"
#include "engine/planselector/PartialAssignment.h"
#include "engine/model/Plan.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "engine/model/PlanType.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/BehaviourConfiguration.h"
#include "engine/model/PlanningProblem.h"
#include "engine/model/Behaviour.h"
#include "engine/IPlanner.h"
#include "engine/planselector/TaskAssignment.h"
#include "engine/model/EntryPoint.h"
#include "engine/collections/RobotProperties.h"
#include "engine/model/State.h"

#include "engine/RunningPlan.h"
#include "engine/model/AbstractPlan.h"

namespace alica
{

	PlanSelector::PlanSelector()
	{
		this->to = AlicaEngine::getInstance()->getTeamObserver();
		PartialAssignment::init();
	}

	PlanSelector::~PlanSelector()
	{
	}

	RunningPlan* PlanSelector::getBestSimilarAssignment(RunningPlan* rp)
	{
		PartialAssignment::reset();
		list<Plan*> newPlanList;
		if (rp->getPlanType() == nullptr)
		{
			newPlanList = list<Plan*>();
			newPlanList.push_back(dynamic_cast<Plan*>(rp->getPlan()));
		}
		else
		{
			newPlanList = rp->getPlanType()->getPlans();
		}
		auto robots = rp->getAssignment()->getAllRobots();
		return this->createRunningPlan(rp->getParent(), newPlanList, robots, rp, rp->getPlanType());
	}

	RunningPlan* PlanSelector::getBestSimilarAssignment(RunningPlan* rp, shared_ptr<vector<int> > robots)
	{
		PartialAssignment::reset();
		list<Plan*> newPlanList;
		if (rp->getPlanType() == nullptr)
		{
			newPlanList = list<Plan*>();
			newPlanList.push_back(dynamic_cast<Plan*>(rp->getPlan()));
		}
		else
		{
			newPlanList = rp->getPlanType()->getPlans();
		}
		return this->createRunningPlan(rp->getParent(), newPlanList, robots, rp, rp->getPlanType());
	}

	shared_ptr<list<RunningPlan*> > PlanSelector::getPlansForState(RunningPlan* planningParent,
																	list<AbstractPlan*> plans,
																	shared_ptr<vector<int> > robotIDs)
	{
		PartialAssignment::reset();
		shared_ptr<list<RunningPlan*> > ll = this->getPlansForStateInternal(planningParent, plans, robotIDs);
		return ll;

	}

	RunningPlan* PlanSelector::createRunningPlan(RunningPlan* planningParent, list<Plan*> plans,
													shared_ptr<vector<int> > robotIDs, RunningPlan* oldRp,
													PlanType* relevantPlanType)
	{
		list<Plan*> newPlanList = list<Plan*>();
		for (Plan* plan : plans)
		{
			if (plan->getMinCardinality() < robotIDs->size() + to->successesInPlan(plan))
			{
#ifdef PSDEBUG
				stringstream ss;
				ss << "PS: RobotIds: ";
				for (int robot : (*robotIDs))
				{
					ss << robot << ", ";
				}
				ss << "= " << robotIDs->size() << " IDs are not enough for the plan " << plan->getName() << "!" << endl;
				//this.baseModule.Mon.Error(1000, msg);

				cout << ss.str();
#endif
			}
			else
			{
				newPlanList.push_back(plan);
			}
		}
		if (newPlanList.size() == 0)
		{
			return nullptr;
		}
		TaskAssignment* ta;
		Assignment* oldAss = nullptr;
		RunningPlan* rp;
		if (oldRp == nullptr)
		{
			rp = new RunningPlan(relevantPlanType);
			ta = new TaskAssignment(newPlanList, robotIDs, true);
		}
		else
		{
			rp = new RunningPlan(oldRp->getPlanType());
			ta = new TaskAssignment(newPlanList, robotIDs, false);
			oldAss = oldRp->getAssignment();
		}
#ifdef PSDEBUG
		cout << ta->toString();
#endif
		EntryPoint* ep = nullptr;
		RobotProperties* ownRobProb = to->getOwnRobotProperties();
		rp->setParent(planningParent);
		shared_ptr<list<RunningPlan*> > rpChildren;
		do
		{
			rp->setAssignment(ta->getNextBestAssignment(oldAss));
			if (rp->getAssignment() == nullptr)
			{
#ifdef PSDEBUG
				cout << "PS: rp.Assignment is NULL" << endl;
#endif
				return nullptr;
			}
			rp->setPlan(rp->getAssignment()->getPlan());
			if (!rp->evalPreCondition())
			{
				continue;
			}
			if (!rp->evalRuntimeCondition())
			{
				continue;
			}

			if (ep == nullptr)
			{
#ifdef PSDEBUG
				cout << "PS: The robot " << ownRobProb->getName() << "(Id: " << ownRobProb->getId()
						<< ") is not assigned to enter the plan " << rp->getPlan()->getName() << " and will IDLE!"
						<< endl;
#endif
				rp->setActiveState(nullptr);
				rp->setOwnEntryPoint(nullptr);
				return rp; // If we return here, this robot will idle (no ep at rp)
			}
			else
			{
				// assign found EntryPoint (this robot dont idle)
				rp->setOwnEntryPoint(ep);
			}
			if(oldRp == nullptr)
			{
				rpChildren = this->getPlansForStateInternal(rp, rp->getActiveState()->getPlans(), rp->getAssignment()->getRobotsWorking(ep));
			}
			else
			{
#ifdef PSDEBUG
					cout << "PS: no recursion due to utilitycheck" << endl;
#endif
					break;
			}
		} while (rpChildren->size() == 0); // c# rpChildren == null
		if(rpChildren->size() == 0) // c# rpChildren == null
		{
#ifdef PSDEBUG
				cout << "PS: Set child -> father reference" << endl;
#endif
				rp->addChildren(rpChildren);
		}
		return rp;
	}

	shared_ptr<list<RunningPlan*> > PlanSelector::getPlansForStateInternal(RunningPlan* planningParent,
																			list<AbstractPlan*> plans,
																			shared_ptr<vector<int> > robotIDs)
	{
		shared_ptr<list<RunningPlan*> > rps = make_shared<list<RunningPlan*> >(list<RunningPlan*>());
#ifdef PSDEBUG
		cout << "<######PS: GetPlansForState: Parent:"
				<< (planningParent != nullptr ? planningParent->getPlan()->getName() : "null") << " plan count: "
				<< plans.size() << " robot count: " << robotIDs->size() << endl;
#endif
		RunningPlan* rp;
		list<Plan*> planList;
		BehaviourConfiguration* bc;
		Plan* p;
		PlanType* pt;
		PlanningProblem* pp;
		for (AbstractPlan* ap : plans)
		{
			bc = dynamic_cast<BehaviourConfiguration*>(ap);
			if (bc != nullptr)
			{
				rp = new RunningPlan(bc);
				rp->setPlan(bc);
				rps->push_back(rp);
				rp->setParent(planningParent);
#ifdef PSDEBUG
				cout << "PS: Added Behaviour " << bc->getBehaviour()->getName() << endl;
#endif
			}
			else
			{
				p = dynamic_cast<Plan*>(ap);
				if (p != nullptr)
				{
					planList = list<Plan*>();
					planList.push_back(p);
					rp = this->createRunningPlan(planningParent, planList, robotIDs, nullptr, nullptr);
					if (rp == nullptr)
					{
#ifdef PSDEBUG
						cout << "PS: It was not possible to create a RunningPlan for the Plan " << p->getName() << "!"
								<< endl;
#endif
						return nullptr;
					}
					rps->push_back(rp);
				}
				else
				{
					pt = dynamic_cast<PlanType*>(ap);
					if (pt != nullptr)
					{
						rp = this->createRunningPlan(planningParent, pt->getPlans(), robotIDs, nullptr, pt);
						if (rp == nullptr)
						{
#ifdef PSDEBUG
							cout << "PS: It was not possible to create a RunningPlan for the Plan " << pt->getName()
									<< "!" << endl;
#endif
							return nullptr;
						}
						rps->push_back(rp);
						pt = nullptr;
					}
					else
					{
						pp = dynamic_cast<PlanningProblem*>(ap);
						if (pp == nullptr)
						{
							cerr
									<< "PS: WTF? An AbstractPlan wasnt a BehaviourConfiguration, a Plan, a PlanType nor a PlannigProblem: "
									<< ap->getId() << endl;
							throw new exception();
						}
						Plan* myP = AlicaEngine::getInstance()->getPlanner()->requestPlan(pp);
						planList = list<Plan*>();
						planList.push_back(myP);
						rp = this->createRunningPlan(planningParent, planList, robotIDs, nullptr, nullptr);
						if (rp == nullptr)
						{
#ifdef PSDEBUG
							cout << "PS: Unable to execute planning result" << endl;
#endif
							return nullptr;
						}
						rps->push_back(rp);
					}
				}
			}
		}
		return rps;
	}

} /* namespace alica */

