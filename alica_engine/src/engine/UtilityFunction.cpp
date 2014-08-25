/*
 * UtilityFunction.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: Stefan Jakob
 */

#include "engine/UtilityFunction.h"
#include "engine/TaskRoleStruct.h"
#include "engine/UtilityInterval.h"
#include "engine/Assignment.h"
#include "engine/RunningPlan.h"
#include "engine/USummand.h"
#include "engine/IRoleAssignment.h"
#include "engine/model/Plan.h"
#include "engine/teamobserver/TeamObserver.h"
#include "engine/model/Task.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/RoleSet.h"
#include "engine/model/RoleTaskMapping.h"
#include "engine/model/Role.h"

namespace alica
{

	UtilityFunction::UtilityFunction(string name, list<USummand*> utilSummands, double priorityWeight,
										double similarityWeight, Plan* plan)
	{
		this->ra = nullptr;
		this->bpe = nullptr;
		this->lookupStruct = new TaskRoleStruct(0, 0);
		this->priResult = new UtilityInterval(0.0, 0.0);
		this->simUI = new UtilityInterval(0.0, 0.0);
		this->name = name;
		this->utilSummands = utilSummands;
		this->priorityWeight = priorityWeight;
		this->similarityWeight = similarityWeight;
		this->plan = plan;

	}

	UtilityFunction::~UtilityFunction()
	{
	}

	list<USummand*> UtilityFunction::getUtilSummands()
	{
		return utilSummands;
	}

	void UtilityFunction::setUtilSummands(list<USummand*> utilSummands)
	{
		this->utilSummands = utilSummands;
	}

	Plan* UtilityFunction::getPlan()
	{
		return plan;
	}

	double UtilityFunction::eval(RunningPlan* newRp, RunningPlan* oldRp)
	{
		if (!newRp->getAssignment()->isValid())
		{
			return -1.0;
		}
		UtilityInterval* sumOfUI = new UtilityInterval(0.0, 0.0);
		double sumOfWeights = 0.0;
		UtilityInterval* prioUI = this->getPriorityResult(newRp->getAssignment());
		if (prioUI->getMax() < 0.0)
		{
			return -1;
		}
		sumOfUI->setMax(sumOfUI->getMax() + this->priorityWeight * prioUI->getMax());
		sumOfUI->setMin(sumOfUI->getMin() + this->priorityWeight * prioUI->getMin());
		sumOfWeights += this->priorityWeight;
		UtilityInterval* curUI;
		for (int i = 0; i < this->utilSummands.size(); ++i)
		{
			auto iter = utilSummands.begin();
			advance(iter, i);
			curUI = (*iter)->eval(newRp->getAssignment());
			if (curUI->getMax() == -1.0)
			{
				return -1.0;
			}
			sumOfWeights += (*iter)->getWeight();
			sumOfUI->setMax(sumOfUI->getMax() + (*iter)->getWeight() * curUI->getMax());
			sumOfUI->setMin(sumOfUI->getMin() + (*iter)->getWeight() * curUI->getMin());
		}

		if (oldRp != nullptr && this->similarityWeight > 0)
		{
			UtilityInterval* simUI = this->getSimilarity(newRp->getAssignment(), oldRp->getAssignment());
			sumOfUI->setMax(sumOfUI->getMax() + this->similarityWeight * simUI->getMax());
			sumOfUI->setMin(sumOfUI->getMin() + this->similarityWeight * simUI->getMin());
			sumOfWeights += this->similarityWeight;
		}

		if (sumOfWeights > 0.0)
		{
			sumOfUI->setMax(sumOfUI->getMax() / sumOfWeights);
			sumOfUI->setMin(sumOfUI->getMin() / sumOfWeights);

			if ((sumOfUI->getMax() - sumOfUI->getMin()) > DIFFERENCETHRESHOLD)
			{
				cerr << "UF: The utility min and max value differs more than " << DIFFERENCETHRESHOLD
						<< " for an Assignment!" << endl;
			}
			return sumOfUI->getMax();
		}

		return 0.0;
	}

	UtilityInterval* UtilityFunction::eval(IAssignment* newAss, IAssignment* oldAss)
	{
		UtilityInterval* sumOfUI = new UtilityInterval(0.0, 0.0);
		double sumOfWeights = 0.0;
		UtilityInterval* prioUI = this->getPriorityResult(newAss);
		if (prioUI->getMax() == -1.0)
		{
			return prioUI;
		}
		sumOfUI->setMax(sumOfUI->getMax() + this->priorityWeight * prioUI->getMax());
		sumOfUI->setMin(sumOfUI->getMin() + this->priorityWeight * prioUI->getMin());
		sumOfWeights += this->priorityWeight;
		UtilityInterval* curUI;
		for (int i = 0; i < this->utilSummands.size(); ++i)
		{
			auto iter = utilSummands.begin();
			advance(iter, i);
			curUI = (*iter)->eval(newAss);
			if (curUI->getMax() == -1.0)
			{
				sumOfUI->setMax(-1.0);
				return sumOfUI;
			}
			sumOfWeights += (*iter)->getWeight();
			sumOfUI->setMax(sumOfUI->getMax() + (*iter)->getWeight() * curUI->getMax());
			sumOfUI->setMin(sumOfUI->getMin() + (*iter)->getWeight() * curUI->getMin());
		}
		if (oldAss != nullptr && this->similarityWeight > 0)
		{
			UtilityInterval* simUI = this->getSimilarity(newAss, oldAss);
			sumOfUI->setMax(sumOfUI->getMax() + this->similarityWeight * simUI->getMax());
			sumOfUI->setMin(sumOfUI->getMin() + this->similarityWeight * simUI->getMin());
			sumOfWeights += this->similarityWeight;
		}
		if (sumOfWeights > 0.0)
		{
			sumOfUI->setMax(sumOfUI->getMax() / sumOfWeights);
			sumOfUI->setMin(sumOfUI->getMin() / sumOfWeights);

			return sumOfUI;
		}

		sumOfUI->setMin(0.0);
		sumOfUI->setMax(0.0);
		return sumOfUI;
	}

	void UtilityFunction::updateAssignment(IAssignment* newAss, IAssignment* oldAss)
	{
		UtilityInterval* utilityInterval = this->eval(newAss, oldAss);
		newAss->setMin(utilityInterval->getMin());
		newAss->setMax(utilityInterval->getMax());
	}

	void UtilityFunction::cacheEvalData()
	{
		if(this->utilSummands.size() == 0)
		{
			for(int i = 0; i < this->utilSummands.size(); ++i)
			{
				auto iter = this->utilSummands.begin();
				advance(iter, i);
				(*iter)->cacheEvalData();
			}
		}
	}

	void UtilityFunction::init()
	{
		this->roleHighestPriorityMap = map<long, double>();
		this->priorityMartix = map<TaskRoleStruct*, double>();
		RoleSet* roleSet = AlicaEngine::getInstance()->getRoleSet();
		long taskId;
		long roleId;
		double curPrio;

		for(RoleTaskMapping* rtm : roleSet->getRoleTaskMappings())
		{
			roleId = rtm->getRole()->getId();
			this->roleHighestPriorityMap.insert(pair<long, double>(roleId, 0.0));
			for(auto epIter = this->plan->getEntryPoints().begin(); epIter != this->plan->getEntryPoints().end(); epIter++)
			{
				taskId = epIter->second->getId();
				auto iter = rtm->getTaskPriorities().find(taskId);
				if(iter == rtm->getTaskPriorities().end())
				{
					stringstream ss;
					ss << "UF: There is no priority for the task "
		                    << taskId << " in the roleTaskMapping of the role "
		                    << rtm->getRole()->getName() << " with id " << roleId
		                    << "!\n We are in the UF for the plan "
		                    << this->plan->getName() << "!" << endl;
					AlicaEngine::getInstance()->abort(ss.str());
				}
				else
				{
					curPrio = iter->second;
				}
				TaskRoleStruct* trs = new TaskRoleStruct(taskId, roleId);
				if(this->priorityMartix.find(trs) == this->priorityMartix.end())
				{
					this->priorityMartix.insert(pair<TaskRoleStruct*, double>(trs, curPrio));
				}
				if(this->roleHighestPriorityMap.at(roleId) < curPrio)
				{
					this->roleHighestPriorityMap.at(roleId) = curPrio;
				}
			}
			this->priorityMartix.insert(pair<TaskRoleStruct*, double>(new TaskRoleStruct(Task::IDLEID, roleId), curPrio));
		}
		//c# != null
		if (this->utilSummands.size() != 0)
		{
			for(USummand* utilSum : this->utilSummands) {
				utilSum->init();
			}
		}
		this->bpe = AlicaEngine::getInstance();
		this->ra  = this->bpe->getRoleAssignment();
	}

	void UtilityFunction::initDataStructures()
	{

		map<long,Plan*> plans = AlicaEngine::getInstance()->getPlanRepository()->getPlans();

		for(auto iter = plans.begin(); iter != plans.end(); iter++) {
			iter->second->getUtilityFunction()->init();
		}
		cout << "KOMME RAUS" << endl;
	}

	string UtilityFunction::toString()
	{

		stringstream ss;
		ss <<  this->name << endl;
		ss <<  "prioW: " << this->priorityWeight << " simW: " << this->similarityWeight << endl;
		for(USummand* utilSummand : this->utilSummands)
		{
			ss << utilSummand->toString();
		}
		return ss.str();
	}

	map<TaskRoleStruct*, double> UtilityFunction::getPriorityMartix()
	{
		return priorityMartix;
	}

	UtilityInterval* UtilityFunction::getPriorityResult(IAssignment* ass)
	{
		this->priResult->setMax(0.0);
		this->priResult->setMin(0.0);
		if(this->priorityWeight == 0)
		{
			return this->priResult;
		}
		//c# != null
		if(ass->getUnassignedRobots().size() == 0)
		{
			for(int i = 0; i < ass->getUnassignedRobots().size(); i++)
			{
				auto iter = ass->getUnassignedRobots().begin();
				advance(iter, i);
				this->priResult->setMax(this->priResult->getMax() + this->roleHighestPriorityMap.at(this->ra->getRole((*iter))->getId()));
			}
		}

		int denum = min(this->plan->getMaxCardinality(), this->bpe->getTeamObserver()->teamSize());
		long taskId;
		long roleId;
		vector<EntryPoint*> eps = ass->getEntryPoints();
		double curPrio;

		for(int i = 0; i < ass->getEntryPointCount(); ++i)
		{
			taskId = eps[i]->getTask()->getId();
			auto robotList = ass->getUniqueRobotsWorkingAndFinished(eps[i]);
			for(int j = 0; j < robotList->size(); ++j)
			{
				auto iter = robotList->begin();
				advance(iter, j);
				roleId = this->ra->getRole((*iter))->getId();
				this->lookupStruct->taskId = taskId;
				this->lookupStruct->roleId = roleId;
				curPrio = this->priorityMartix.at(this->lookupStruct);
				if(curPrio < 0.0)
				{
					this->priResult->setMin(-1.0);
					this->priResult->setMax(-1.0);
					return this->priResult;
				}
				this->priResult->setMin(this->priResult->getMin() + curPrio);
#ifdef UFDEBUG
				cout << "UF: taskId:" << taskId << " roleId:" << roleId << " prio: " << this->priorityMartix[this->lookupStruct] << endl;
#endif
			}
		}
#ifdef UFDEBUG
		cout << "##" << endl;
		cout << "UF: prioUI.Min = " << priResult->getMin() << endl;
		cout << "UF: prioUI.Max = " << priResult->getMax() << endl;
		cout << "UF: denum = " << denum << endl;
#endif
			priResult->setMax(priResult->getMax() + priResult->getMin());
			if(denum != 0)
			{
				priResult->setMin(priResult->getMin() / denum);
				priResult->setMax(priResult->getMax() / denum);
			}
#ifdef UFDEBUG
			cout << "UF: prioUI.Min = " << priResult->getMin() << endl;
			cout << "UF: prioUI.Max = " << priResult->getMax() << endl;
			cout << "##" << endl;
#endif
			return priResult;
	}

	pair<vector<double>, double>* UtilityFunction::differentiate(IAssignment* newAss)
	{
		return nullptr;
	}

	UtilityInterval* UtilityFunction::getSimilarity(IAssignment* newAss, IAssignment* oldAss)
	{
		simUI->setMax(0.0);
		simUI->setMin(0.0);
		int numOldAssignedRobots = 0;
		vector<EntryPoint*> oldAssEps = oldAss->getEntryPoints();

		for(int i = 0; i < oldAss->getEntryPointCount(); ++i)
		{
			auto oldRobots = oldAss->getRobotsWorkingAndFinished(oldAssEps[i]);
			numOldAssignedRobots += oldRobots->size();
			auto newRobots = newAss->getRobotsWorkingAndFinished(oldAssEps[i]);

			//C# newRobots != null
			if(newRobots->size() != 0)
			{
				for(int oldRobot : (*oldRobots))
				{
					if(find(newRobots->begin(),newRobots->end(), oldRobot) != newRobots->end())
					{
						simUI->setMin(simUI->getMin() + 1);
					}
					else if(oldAssEps[i]->getMaxCardinality() > newRobots->size() && find(newAss->getUnassignedRobots().begin(),newAss->getUnassignedRobots().end(), oldRobot) != newAss->getUnassignedRobots().end())
					{
						simUI->setMax(simUI->getMax() + 1);
					}
				}
			}
		}

		simUI->setMax(simUI->getMax() + simUI->getMin());
		if (numOldAssignedRobots > 0)
		{
			simUI->setMin(simUI->getMin() / numOldAssignedRobots);
			simUI->setMax(simUI->getMax() / numOldAssignedRobots);

		}

		return simUI;
	}

} /* namespace alica */
