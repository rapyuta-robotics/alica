/*
 * PartialAssignment.cpp
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#include <engine/planselector/PartialAssignment.h>

#include "engine/planselector/EpByTaskComparer.h"
#include "engine/planselector/DynCardinality.h"
#include "engine/collections/AssignmentCollection.h"
#include "engine/model/Plan.h"
#include "engine/model/EntryPoint.h"
#include "engine/collections/SuccessCollection.h"
#include "engine/model/Task.h"

namespace alica
{

	PartialAssignment::PartialAssignment()
	{
		this->maxCount = 10100;
		this->maxEpsCount = 20;
		this->curIndex = 0;
		this->daPAs = vector<PartialAssignment*>(maxCount);
		this->epByTaskComparer = EpByTaskComparer();
		this->allowIdling = (*supplementary::SystemConfig::getInstance())["Alica"]->get<bool>("Alica.AllowIdling");
		this->epRobotsMapping = new AssignmentCollection(maxEpsCount);
		this->unAssignedRobots = vector<int>();
		this->dynCardinalities = vector<DynCardinality*>(maxEpsCount);
		this->compareVal = PRECISION;

		for (int i = 0; i < maxEpsCount; i++)
		{
			this->dynCardinalities[i] = new DynCardinality();
		}

	}

	PartialAssignment::~PartialAssignment()
	{
	}

	AssignmentCollection* PartialAssignment::getEpRobotsMapping()
	{
		return epRobotsMapping;
	}

	Plan* PartialAssignment::getPlan()
	{
		return plan;
	}

	UtilityFunction* PartialAssignment::getUtilFunc()
	{
		return utilFunc;
	}

	SuccessCollection* PartialAssignment::getEpSuccessMapping()
	{
		return epSuccessMapping;
	}

	vector<int> PartialAssignment::getUnAssignedRobots()
	{
		return unAssignedRobots;
	}

	double PartialAssignment::getMax()
	{
		return max;
	}

	void PartialAssignment::setMax(double max)
	{
		this->max = max;
	}

	double PartialAssignment::getMin()
	{
		return min;
	}

	void PartialAssignment::setMin(double min)
	{
		this->min = min;
	}

	vector<EntryPoint*> PartialAssignment::getEntryPoints()
	{
		return this->epRobotsMapping->getEntryPoints();
	}

	void PartialAssignment::init()
	{
		idleEP = new EntryPoint();
		idleEP->setName("IDLE-ep");
		idleEP->setId(EntryPoint::IDLEID);
		idleEP->setMinCardinality(0);
		idleEP->setMaxCardinality(numeric_limits<int>::max());
		idleEP->setTask(new Task(true));
		idleEP->getTask()->setName("IDLE-TASK");
		idleEP->getTask()->setId(Task::IDLEID);

		for(int i = 0; i < maxCount; i++)
		{
			daPAs[i] = new PartialAssignment();
		}

	}

	void PartialAssignment::clear()
	{
		this->min = 0.0;
		this->max = 1.0;
		this->compareVal = PRECISION;
		this->unAssignedRobots.clear();
		for(int i = 0; i < this->epRobotsMapping->getCount(); i++)
		{
			this->epRobotsMapping->getRobots()[i]->clear();
		}
		this->hashCalculated = false;
	}

	void PartialAssignment::reset()
	{
		curIndex = 0;
	}

	PartialAssignment* PartialAssignment::getNew(vector<int> robots, Plan* plan, SuccessCollection* sucCol)
	{
		if (curIndex >= maxCount)
		{
			cerr << "max PA count reached!" << endl;
		}
		PartialAssignment* ret = daPAs[curIndex++];
		ret->clear();
		ret->robots = robots;
		ret->plan = plan;
		ret->utilFunc = plan->getUtilityFunction();
		ret->epSuccessMapping = sucCol;
		if (allowIdling)
		{
			ret->epRobotsMapping->setCount(plan->getEntryPoints().size() + 1);
			ret->epRobotsMapping->getEntryPoints()[ret->epRobotsMapping->getCount() - 1] = idleEP;
		}
		else
		{
			ret->epRobotsMapping->setCount(plan->getEntryPoints().size());
		}
		//TODO check if copies are needed
		vector<EntryPoint*> copy;
		for (auto iter : plan->getEntryPoints())
		{
			copy.push_back(iter.second);
		}
		ret->epRobotsMapping->setEntryPoints(copy);

		if (allowIdling)
		{
			auto iter1 = ret->epRobotsMapping->getEntryPoints().begin();
			auto iter2 = ret->epRobotsMapping->getEntryPoints().begin();
			advance(iter2, ret->epRobotsMapping->getCount() - 1);
			sort(iter1, iter2, EpByTaskComparer::compareTo);
		}
		else
		{
			auto iter1 = ret->epRobotsMapping->getEntryPoints().begin();
			auto iter2 = ret->epRobotsMapping->getEntryPoints().begin();
			advance(iter2, ret->epRobotsMapping->getCount());
			sort(iter1, iter2, EpByTaskComparer::compareTo);
		}
		for (int i = 0; i < ret->epRobotsMapping->getCount(); i++)
		{
			ret->dynCardinalities[i]->setMin(ret->epRobotsMapping->getEntryPoints()[i]->getMinCardinality());
			ret->dynCardinalities[i]->setMax(ret->epRobotsMapping->getEntryPoints()[i]->getMaxCardinality());
			shared_ptr<list<int> > suc = sucCol->getRobots(ret->epRobotsMapping->getEntryPoints()[i]);

			if (suc != nullptr)
			{
				ret->dynCardinalities[i]->setMin(ret->dynCardinalities[i]->getMin() - suc->size());
				ret->dynCardinalities[i]->setMax(ret->dynCardinalities[i]->getMax() - suc->size());
				if (ret->dynCardinalities[i]->getMin() < 0)
				{
					ret->dynCardinalities[i]->setMin(0);
				}
				if (ret->dynCardinalities[i]->getMax() < 0)
				{
					ret->dynCardinalities[i]->setMax(0);
				}

#ifdef SUCDEBUG
				cout << "SuccessCollection" << endl;
				cout << "EntryPoint: " << ret->epRobotsMapping->getEntryPoints()[i]->toString() << endl;
				cout << "DynMax: " << ret->dynCardinalities[i]->getMax() << endl;
				cout << "DynMin: " << ret->dynCardinalities[i]->getMin() << endl;
				cout << "SucCol: ";
				for (int j : (*suc))
				{
					cout << j << ", ";
				}
				cout << "-----------" << endl;
#endif
			}
		}

		for (int i : robots)
		{
			ret->unAssignedRobots.push_back(i);
		}
		return ret;
	}

	PartialAssignment* PartialAssignment::getNew(PartialAssignment* oldPA)
	{
		if (curIndex >= maxCount)
		{
			cerr << "max PA count reached!" << endl;
		}
		PartialAssignment* ret = daPAs[curIndex++];
		ret->clear();
		ret->min = oldPA->min;
		ret->max = oldPA->max;
		ret->plan = oldPA->plan;
		ret->robots = oldPA->robots;
		ret->utilFunc = oldPA->utilFunc;
		ret->epSuccessMapping = oldPA->epSuccessMapping;
		for (int i = 0; i < oldPA->unAssignedRobots.size(); i++)
		{
			ret->unAssignedRobots.push_back(oldPA->unAssignedRobots[i]);
		}
		//TODO check if copies are needed
		vector<DynCardinality*> copy;
		for (auto iter : oldPA->dynCardinalities)
		{
			copy.push_back(iter);
		}
		ret->dynCardinalities = copy;

		vector<shared_ptr<vector<int> > > oldRobotLists = oldPA->epRobotsMapping->getRobots();

		for (int i = 0; i < oldPA->epRobotsMapping->getCount(); i++)
		{
			ret->epRobotsMapping->getEntryPoints()[i] = oldPA->epRobotsMapping->getEntryPoints()[i];
			for (int j = 0; j < oldRobotLists[i]->size(); j++)
			{
				ret->epRobotsMapping->getRobots()[i]->push_back(oldRobotLists[i].get()->at(j));
			}

		}

		ret->epRobotsMapping->setCount(oldPA->epRobotsMapping->getCount());
		return ret;

	}

	int PartialAssignment::getEntryPointCount()
	{
		return this->epRobotsMapping->getCount();
	}

	int PartialAssignment::totalRobotCount()
	{
		int c = 0;
		for (int i = 0; i < this->epRobotsMapping->getRobots().size(); i++)
		{
			c += this->epRobotsMapping->getRobots()[i]->size();
		}
		return this->numUnAssignedRobots() + c;
	}

	shared_ptr<vector<int> > PartialAssignment::getRobotsWorking(EntryPoint* ep)
	{
		return this->epRobotsMapping->getRobots(ep);
	}

	shared_ptr<vector<int> > PartialAssignment::getRobotsWorking(long epid)
	{
		return this->epRobotsMapping->getRobotsById(epid);
	}

	shared_ptr<list<int> > PartialAssignment::getRobotsWorkingAndFinished(EntryPoint* ep)
	{
		shared_ptr<list<int> > ret = make_shared<list<int> >(list<int>());
		auto robots = this->epRobotsMapping->getRobots(ep);
		if (robots != nullptr)
		{
			for (auto iter : (*robots))
			{
				ret->push_back(iter);
			}
		}
		auto successes = this->epSuccessMapping->getRobots(ep);
		if (successes != nullptr)
		{
			for (auto iter : (*successes))
			{
				ret->push_back(iter);
			}
		}
		return ret;
	}

	shared_ptr<list<int> > PartialAssignment::getRobotsWorkingAndFinished(long epid)
	{
		shared_ptr<list<int> > ret = make_shared<list<int> >(list<int>());
		auto robots = this->epRobotsMapping->getRobotsById(epid);
		if (robots != nullptr)
		{
			for (auto iter : (*robots))
			{
				ret->push_back(iter);
			}
		}
		auto successes = this->epSuccessMapping->getRobotsById(epid);
		if (successes != nullptr)
		{
			for (auto iter : (*successes))
			{
				ret->push_back(iter);
			}
		}
		return ret;
	}

	shared_ptr<list<int> > PartialAssignment::getUniqueRobotsWorkingAndFinished(EntryPoint* ep)
	{
		shared_ptr<list<int> > ret = make_shared<list<int> >(list<int>());
		auto robots = this->epRobotsMapping->getRobots(ep);

		for (auto iter : (*robots))
		{
			ret->push_back(iter);
		}

		auto successes = this->epSuccessMapping->getRobots(ep);
		if (successes != nullptr)
		{
			for (auto iter : (*successes))
			{
				if (find(ret->begin(), ret->end(), iter) == ret->end())
				{
					ret->push_back(iter);
				}
			}
		}
		return ret;

	}

	bool PartialAssignment::addIfAlreadyAssigned(SimplePlanTree* spt, int robot)
	{
	}

	bool PartialAssignment::assignRobot(int robot, int index)
	{
	}

	list<PartialAssignment*> PartialAssignment::expand()
	{
	}

	bool PartialAssignment::isValid()
	{
	}

	bool PartialAssignment::isGoal()
	{
	}

	bool PartialAssignment::compareTo(PartialAssignment* thisPa, PartialAssignment* oldPa)
	{
	}

	int PartialAssignment::getHashCode()
	{
	}

	string PartialAssignment::toString()
	{
	}

	int PartialAssignment::numUnAssignedRobots()
	{
		return this->unAssignedRobots.size();
	}

	string PartialAssignment::assignmentCollectionToString()
	{
	}

	int PartialAssignment::pow(int x, int y)
	{
	}

} /* namespace alica */
