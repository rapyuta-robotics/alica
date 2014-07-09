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
		for(int i : paraRobots)
		{
			this->robots[k++] = i;
		}
		sort(robots.begin(), robots.end());
		this->fringe = vector<PartialAssignment*>();
		auto simplePlanTreeMap = to->getTeamPlanTrees();
		PartialAssignment* curPa;
		for(Plan* curPlan : this->planList)
		{
			//TODO finish UtilityFunction
//			curPlan->getUtilityFunction()->cacheEvalData();

			curPa = PartialAssignment::getNew(this->robots, curPlan, to->getSuccessCollection(curPlan));

			if(preasingOtherRobots)
			{

				if(this->addAlreadyAssignedRobots(curPa, &(*simplePlanTreeMap)))
				{
					//TODO c# 83
				}
			}
		}
	}

	Assignment* TaskAssignment::getNextBestAssignment(IAssignment* oldAss)
	{
	}

	string TaskAssignment::toString()
	{
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
	}
/**
 *
 * @param pa
 * @param simplePlanTreeMap never try to delete this
 * @return
 */
	bool TaskAssignment::addAlreadyAssignedRobots(PartialAssignment* pa, map<int, shared_ptr<SimplePlanTree> >* simplePlanTreeMap)
	{
	}

} /* namespace alica */
