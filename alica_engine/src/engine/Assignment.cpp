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

	Assignment::~Assignment()
	{
	}
	Assignment::Assignment(Plan* p)
	{
		this->plan = p;
		this->max = 0.0;
		this->min = 0.0;
		this->allowIdling = (*supplementary::SystemConfig::getInstance())["Alica"]->get<bool>("Alica.AllowIdling",NULL);

		this->epRobotsMapping = new AssignmentCollection(this->plan->getEntryPoints().size());

		list<EntryPoint*> l;
		transform(plan->getEntryPoints().begin(), plan->getEntryPoints().end(), back_inserter(l), [](map<long, EntryPoint*>::value_type& val){return val.second;} );
		l.sort();
		copy(l.begin(), l.end(), back_inserter(epRobotsMapping->getEntryPoints()));
		this->robotStateMapping = new StateCollection(this->epRobotsMapping);
		this->epSucMapping = new SuccessCollection(p);
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

	Assignment::Assignment(PartialAssignment* pa)
	{
	}

	Assignment::Assignment(Plan* p, AllocationAuthorityInfo* aai)
	{
	}

	shared_ptr<vector<int> > Assignment::getRobotsWorking(EntryPoint* ep)
	{
		return this->getEpRobotsMapping()->getRobots(ep);
	}

	int Assignment::totalRobotCount()
	{
	}

	vector<EntryPoint*> Assignment::getEntryPoints()
	{
	}

	int Assignment::getEntryPointCount()
	{
	}

	shared_ptr<list<int> > Assignment::getRobotsWorkingAndFinished(EntryPoint* ep)
	{
	}

	shared_ptr<list<int> > Assignment::getUniqueRobotsWorkingAndFinished(EntryPoint* ep)
	{
	}

	shared_ptr<list<int> > Assignment::getRobotsWorkingAndFinished(long epid)
	{
	}

	SuccessCollection* Assignment::getEpSuccessMapping()
	{
	}

	void Assignment::setAllToInitialState(unique_ptr<list<int> > robots, EntryPoint* defep)
	{
	}

	void Assignment::removeRobot(int robotId)
	{
	}

	void Assignment::addRobot(int id, EntryPoint* e, State* s)
	{
	}

	bool Assignment::isValid()
	{
	}

	bool Assignment::isSuccessfull()
	{
	}

	bool Assignment::isEqual(Assignment* otherAssignment)
	{
	}

	bool Assignment::isEntryPointNonEmpty(EntryPoint* ep)
	{
	}

	bool Assignment::updateRobot(int robot, EntryPoint* ep, State* s)
	{
	}

	bool Assignment::updateRobot(int robot, EntryPoint* ep)
	{
	}

	bool Assignment::removeRobot(int robot, EntryPoint* ep)
	{
	}

	string Assignment::assignmentCollectionToString()
	{
	}

	void Assignment::addRobot(int id, EntryPoint* e)
	{
	}

	void Assignment::moveRobots(State* from, State* to)
	{
	}

	EntryPoint* Assignment::entryPointOfRobot(int robot)
	{
	}

	shared_ptr<vector<int> > Assignment::getAllRobots()
	{
	}

	void Assignment::clear()
	{
	}

	void Assignment::setAllToInitialState(list<int> robots, EntryPoint* ep)
	{
	}

	string Assignment::toString()
	{
	}

	string Assignment::toHackString()
	{
	}

} /* namespace alica */


