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

namespace alica
{

	TaskAssignment::~TaskAssignment()
	{
	}

	TaskAssignment::TaskAssignment(list<Plan*> planList, vector<int> paraRobots, bool preasingOtherRobots)
	{
#ifdef EXPANSIONEVAL
		this->expansionCount = 0;
#endif
		this->planList = planList;
		ITeamObserver* to = AlicaEngine::getInstance()->getTeamObserver();
		this->robots = vector<int>(paraRobots.size());
		int k = 0;
		for (int i : paraRobots)
		{
			this->robots[k++] = i;
		}
		sort(robots.begin(), robots.end());
		this->fringe = vector<PartialAssignment*>();
		auto simplePlanTreeMap = to->getTeamPlanTrees();
		PartialAssignment* curPa;
		for (Plan* curPlan : this->planList)
		{
			//TODO finish UtilityFunction
//			curPlan->getUtilityFunction()->cacheEvalData();

			curPa = PartialAssignment::getNew(this->robots, curPlan, to->getSuccessCollection(curPlan));

			if (preasingOtherRobots)
			{

				if (this->addAlreadyAssignedRobots(curPa, &(*simplePlanTreeMap)))
				{
					//TODO finish UtilityFunction
//					curPlan->getUtilityFunction()->updateAssignment(curPa, nullptr);
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
		ss << "NumRobots: " << this->robots.size() << endl;
		ss << "RobotIDs: ";
		for(int i = 0; i < this->robots.size(); ++i)// RobotIds
		{
			ss << this->robots[i] <<  " ";
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
#ifdef EXPANSIONEVAL
			expansionCount++;
#endif
			for(int i = 0; i < newPas->size(); i++)
			{
				auto iter = newPas->begin();
				advance(iter, i);
				//TODO finish utilityfunction
				//(*iter)->getUtilFunc()->updateAssignment((*iter), oldAss);
				if((*iter)->getMax() != -1)
				{
					this->fringe.push_back((*iter));
				}
			}
#ifdef PSDEBUG
				cout << "<---" << endl;
				cout << "TA: AFTER fringe exp:" << endl;
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
		for (int robot : this->robots)
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
