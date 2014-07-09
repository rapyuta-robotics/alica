/*
 * Assignment.cpp
 *
 *  Created on: Jun 27, 2014
 *      Author: Stefan Jakob
 */

#include "engine/Assignment.h"
#include "engine/collections/AssignmentCollection.h"
#include "engine/model/Plan.h"
#include "engine/collections/StateCollection.h"
#include "engine/collections/SuccessCollection.h"


namespace alica
{

	Assignment::Assignment()
	{
		// TODO Auto-generated constructor stub

	}

	Assignment::~Assignment()
	{
		// TODO Auto-generated destructor stub
	}
	Assignment::Assignment(Plan* pa)
	{
		this->plan = pa;
		this->max = 0;
		this->min = 0;

		this->epRobotsMapping = new AssignmentCollection(this->plan->getEntryPoints().size());

		list<EntryPoint*> l;
		transform(plan->getEntryPoints().begin(), plan->getEntryPoints().end(), back_inserter(l), [](map<long, EntryPoint*>::value_type& val){return val.second;} );
		l.sort();
		copy(l.begin(), l.end(), back_inserter(epRobotsMapping->getEntryPoints()));
		this->robotStateMapping = new StateCollection(this->epRobotsMapping);
		this->epSucMapping = new SuccessCollection(pa);
	}

	Plan* Assignment::getPlan()
	{
		return plan;
	}

	void Assignment::setPlan(Plan* plan)
	{
		this->plan = plan;
	}
	StateCollection* Assignment::getRobotStateMapping()
	{
		return robotStateMapping;
	}

	shared_ptr<vector<int> > Assignment::getAllRobotsSorted()
	{
		shared_ptr<vector<int> > ret = shared_ptr<vector<int> >();
		for (int i = 0; i < this->getEpRobotsMapping()->getCount(); i++)
		{
			for (int j = 0; j < this->getEpRobotsMapping()->getRobots()[i]->size(); j++)
			{
				ret->push_back(this->getEpRobotsMapping()->getRobots()[i]->at(j));
			}
		}
		sort(ret->begin(), ret->end());
		return ret;
	}

	AssignmentCollection* Assignment::getEpRobotsMapping()
	{
		return epRobotsMapping;
	}

	shared_ptr<vector<int> > Assignment::getRobotsWorking(long epid)
	{
		return this->getEpRobotsMapping()->getRobotsById(epid);
	}

	shared_ptr<vector<int> > Assignment::getRobotsWorkingSorted(EntryPoint* ep)
	{
		shared_ptr<vector<int> > ret = getRobotsWorking(ep);
		sort(ret->begin(), ret->end());
		return ret;
	}

	shared_ptr<vector<int> > Assignment::getRobotsWorking(EntryPoint* ep)
	{
		return this->getEpRobotsMapping()->getRobots(ep);
	}

} /* namespace alica */


