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
#include "engine/Assignment.h"
#include "engine/model/Plan.h"
#include "engine/UtilityFunction.h"


#include "engine/model/EntryPoint.h"
#include "engine/model/Task.h"

#define PSDEBUG

namespace alica
{

	TaskAssignment::~TaskAssignment()
	{
	}

	TaskAssignment::TaskAssignment(list<Plan*> planList, shared_ptr<vector<int> > paraRobots, bool preasingOtherRobots)
	{
#ifdef EXPANSIONEVAL
		this->expansionCount = 0;
#endif
		this->planList = planList;
		ITeamObserver* to = AlicaEngine::getInstance()->getTeamObserver();
		this->robots = make_shared<vector<int> >(vector<int>(paraRobots->size()));
		int k = 0;
		for (int i : (*paraRobots))
		{
			this->robots->at(k++) = i;
		}
		sort(robots->begin(), robots->end());
		this->fringe = vector<PartialAssignment*>();
		auto simplePlanTreeMap = to->getTeamPlanTrees();
		PartialAssignment* curPa;
		for (Plan* curPlan : this->planList)
		{
			curPlan->getUtilityFunction()->cacheEvalData();

			curPa = PartialAssignment::getNew(this->robots, curPlan, to->getSuccessCollection(curPlan));

			if (preasingOtherRobots)
			{

				if (this->addAlreadyAssignedRobots(curPa, &(*simplePlanTreeMap)))
				{
					curPlan->getUtilityFunction()->updateAssignment(curPa, nullptr);
				}
			}
			this->fringe.push_back(curPa);
		}
	}

	Assignment* TaskAssignment::getNextBestAssignment(IAssignment* oldAss)
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

		Assignment* newAss = new Assignment(calculatedPa);
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
		for(int i = 0; i < this->robots->size(); ++i)// RobotIds
		{
			ss << (*this->robots)[i] <<  " ";
		}
		ss << endl;
		ss << "Initial Fringe (Count " << this->fringe.size() << "):" << endl;
		ss << "{";
		for(int i = 0; i < this->fringe.size(); ++i)// Initial PartialAssignments
		{
			ss <<  this->fringe[i]->toString();
		}
		ss << "}" << endl;
		ss <<  "-------------------------------------------" << endl;;
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
			cout << "TA: fringe size " << this->fringe.size() << endl;
			curPa = this->fringe.at(0);
			this->fringe.erase(this->fringe.begin());
#ifdef PSDEBUG
			cout << "<---" << endl;
			cout << "TA: NEXT PA from fringe:" << endl;
			cout << curPa->toString() << "--->" << endl;
#endif
			if (curPa->isGoal())
			{
				goal = curPa;
			}
#ifdef PSDEBUG
			cout << "<---" << endl;
			cout << "TA: BEFORE fringe exp:" << endl;
			for(int i = 0; i < this->fringe.size(); i++)
			{
				cout << this->fringe[i]->toString();
			}
			cout << "--->" << endl;
#endif
			auto newPas = curPa->expand();
			cout << "TA: newPas size " << newPas->size() << endl;
#ifdef EXPANSIONEVAL
			expansionCount++;
#endif
			for(int i = 0; i < newPas->size(); i++)
			{
				auto iter = newPas->begin();
				advance(iter, i);
				(*iter)->getUtilFunc()->updateAssignment((*iter), oldAss);
				if((*iter)->getMax() != -1)
				{
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
	 *
	 * @param pa
	 * @param simplePlanTreeMap never try to delete this
	 * @return
	 */
	bool TaskAssignment::addAlreadyAssignedRobots(PartialAssignment* pa,
													map<int, shared_ptr<SimplePlanTree> >* simplePlanTreeMap)
	{
		int ownRobotId = AlicaEngine::getInstance()->getTeamObserver()->getOwnId();
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
