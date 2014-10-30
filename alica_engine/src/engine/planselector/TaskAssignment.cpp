/*
 * TaskAssignment.cpp
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#include <engine/planselector/TaskAssignment.h>
#include "engine/teamobserver/TeamObserver.h"
#include "engine/AlicaEngine.h"
#include "engine/planselector/PartialAssignment.h"
#include "engine/planselector/PartialAssignmentPool.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"
#include "engine/UtilityFunction.h"

#include "engine/model/EntryPoint.h"
#include "engine/model/Task.h"

//#define PSDEBUG

namespace alica
{

	TaskAssignment::~TaskAssignment()
	{
	}

	/**
	 * Constructor of a new TaskAssignment
	 * @param planList Plans to build an assignment for
	 * @param paraRobots robots to build an assignment for
	 * @param a bool
	 */
	TaskAssignment::TaskAssignment(PartialAssignmentPool* pap, ITeamObserver* to, list<Plan*> planList,
									shared_ptr<vector<int> > paraRobots, bool preasingOtherRobots)
	{
#ifdef EXPANSIONEVAL
		this->expansionCount = 0;
#endif
		this->planList = planList;
		this->to = to;
		this->robots = make_shared<vector<int> >(vector<int>(paraRobots->size()));
		int k = 0;
		for (int i : (*paraRobots))
		{
			this->robots->at(k++) = i;
		}
		// sort robot ids ascending
		sort(robots->begin(), robots->end());
		this->fringe = vector<PartialAssignment*>();
		auto simplePlanTreeMap = to->getTeamPlanTrees();
		PartialAssignment* curPa;
		for (Plan* curPlan : this->planList)
		{
			// CACHE EVALUATION DATA IN EACH USUMMAND
			curPlan->getUtilityFunction()->cacheEvalData();

			// CREATE INITIAL PARTIAL ASSIGNMENTS
			curPa = PartialAssignment::getNew(pap, this->robots, curPlan, to->getSuccessCollection(curPlan));

			// ASSIGN PREASSIGNED OTHER ROBOTS
			if (preasingOtherRobots)
			{

				if (this->addAlreadyAssignedRobots(curPa, &(*simplePlanTreeMap)))
				{
					// revaluate this pa
					curPlan->getUtilityFunction()->updateAssignment(curPa, nullptr);
				}
			}
			this->fringe.push_back(curPa);
		}
	}

	/**
	 * Gets the Assignment with next best utility
	 * @param oldAss old Assignment
	 * @return An Assignment for the plan
	 */
	shared_ptr<Assignment> TaskAssignment::getNextBestAssignment(IAssignment* oldAss)
	{
#ifdef PSDEBUG
		cout << "TA: Calculating next best PartialAssignment..." << endl;
#endif
		PartialAssignment* calculatedPa = this->calcNextBestPartialAssignment(oldAss);

		if (calculatedPa == nullptr)
		{
			return nullptr;
		}
#ifdef PSDEBUG
		cout << "TA: ... calculated this PartialAssignment:\n" << calculatedPa->toString();
#endif

		shared_ptr<Assignment> newAss = make_shared<Assignment>(calculatedPa);
#ifdef PSDEBUG
		cout << "TA: Return this Assignment to PS:" << newAss->toString() << endl;
#endif

		return newAss;
	}

	string TaskAssignment::toString()
	{
		stringstream ss;
		ss << endl;
		ss << "--------------------TA:--------------------" << endl;
		ss << "NumRobots: " << this->robots->size() << endl;
		ss << "RobotIDs: ";
		for (int i = 0; i < this->robots->size(); ++i) // RobotIds
		{
			ss << (*this->robots)[i] << " ";
		}
		ss << endl;
		ss << "Initial Fringe (Count " << this->fringe.size() << "):" << endl;
		ss << "{";
		for (int i = 0; i < this->fringe.size(); ++i) // Initial PartialAssignments
		{
			ss << this->fringe[i]->toString();
		}
		ss << "}" << endl;
		ss << "-------------------------------------------" << endl;
		;
		return ss.str();
	}

#ifdef EXPANSIONEVAL
	int TaskAssignment::getExpansionCount()
	{
		return expansionCount;
	}

	void TaskAssignment::setExpansionCount(int expansionCount)
	{
		this->expansionCount = expansionCount;
	}
#endif

	PartialAssignment* TaskAssignment::calcNextBestPartialAssignment(IAssignment* oldAss)
	{
		PartialAssignment* curPa = nullptr;
		PartialAssignment* goal = nullptr;
		while (this->fringe.size() > 0 && goal == nullptr)
		{
			curPa = this->fringe.at(0);
			this->fringe.erase(this->fringe.begin());
#ifdef PSDEBUG
			cout << "<---" << endl;
			cout << "TA: NEXT PA from fringe:" << endl;
			cout << curPa->toString() << "--->" << endl;
#endif
			// Check if it is a goal
			if (curPa->isGoal())
			{
				// Save the goal in result
				goal = curPa;
			}
#ifdef PSDEBUG
			cout << "<---" << endl;
			cout << "TA: BEFORE fringe exp:" << endl;
			cout << "TA: robotID " << this->to->getOwnId() << endl;
			for(int i = 0; i < this->fringe.size(); i++)
			{
				cout << this->fringe[i]->toString();
			}
			cout << "--->" << endl;
#endif
			// Expand for the next search (maybe necessary later)
			auto newPas = curPa->expand();
#ifdef EXPANSIONEVAL
			expansionCount++;
#endif
			// Every just expanded partial assignment must get an updated utility
			for (int i = 0; i < newPas->size(); i++)
			{
				// Update the utility values
				auto iter = newPas->begin();
				advance(iter, i);
				(*iter)->getUtilFunc()->updateAssignment((*iter), oldAss);
				if ((*iter)->getMax() != -1) // add this partial assignment only, if all assigned robots does not have a priority of -1 for any task
				{
					// Add to search fringe
					this->fringe.push_back((*iter));
				}
			}
			sort(fringe.begin(), fringe.end(), PartialAssignment::compareTo);
#ifdef PSDEBUG
			cout << "<---" << endl;
			cout << "TA: AFTER fringe exp:" << endl;
			cout << "TA: fringe size " << this->fringe.size() << endl;
			for(int i = 0; i < this->fringe.size(); i++)
			{
				cout << this->fringe[i]->toString();
			}
			cout << "--->" << endl;
#endif
		}
		return goal;
	}
	/**
	 * If any robot has already assigned itself, this method updates the partial assignment accordingly.
	 * @param pa A PartialAssignment
	 * @param simplePlanTreeMap never try to delete this
	 * @return True if any robot has already assigned itself, false otherwise
	 */
	bool TaskAssignment::addAlreadyAssignedRobots(PartialAssignment* pa,
													map<int, shared_ptr<SimplePlanTree> >* simplePlanTreeMap)
	{
		int ownRobotId = to->getOwnId();
		bool haveToRevalute = false;
		shared_ptr<SimplePlanTree> spt = nullptr;
		for (int robot : (*this->robots))
		{
			if (ownRobotId == robot)
			{
				continue;
			}
			auto iter = simplePlanTreeMap->find(robot);
			if (iter != simplePlanTreeMap->end())
			{
				spt = iter->second;
				if (pa->addIfAlreadyAssigned(spt, robot))
				{
					haveToRevalute = true;
				}
			}
		}
		return haveToRevalute;
	}

} /* namespace alica */
