/*
 * SuccessCollection.cpp
 *
 *  Created on: Jun 17, 2014
 *      Author: Stefan Jakob
 */

#include <engine/collections/SuccessCollection.h>

#include "engine/model/Plan.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Task.h"

namespace alica
{

	SuccessCollection::SuccessCollection(Plan* plan)
	{
		this->count = plan->getEntryPoints().size();
		this->entryPoints = new EntryPoint*[this->count];
		this->robotIds = vector<shared_ptr<list<const supplementary::AgentID*> > >(this->count);
		int i = 0;
		list<EntryPoint*> eps;
		for (map<long, EntryPoint*>::const_iterator iter = plan->getEntryPoints().begin();
				iter != plan->getEntryPoints().end(); iter++)
		{
			eps.push_back(iter->second);
		}
		eps.sort(EntryPoint::compareTo);
		for (EntryPoint* ep : eps)
		{
			this->entryPoints[i] = ep;
			this->robotIds[i] = make_shared<list<const supplementary::AgentID*> >();
			i++;
		}
	}

	SuccessCollection::~SuccessCollection()
	{
		delete[] this->entryPoints;
	}

	int SuccessCollection::getCount() const
	{
		return count;
	}

	void SuccessCollection::setCount(int count)
	{
		this->count = count;
	}

	EntryPoint** SuccessCollection::getEntryPoints()
	{
		return entryPoints;
	}

	void SuccessCollection::setSuccess(const supplementary::AgentID* robotId, EntryPoint* ep)
	{
		for (int i = 0; i < this->count; i++)
		{
			if (this->entryPoints[i] == ep)
			{
				this->robotIds[i]->push_back(robotId);
				return;
			}
		}
	}
	void SuccessCollection::clear()
	{
		for (int i = 0; i < this->count; i++)
		{
			this->robotIds[i]->clear();
		}
	}

	vector<shared_ptr<list<const supplementary::AgentID*> > >& SuccessCollection::getRobots()
	{
		return robotIds;
	}

	void SuccessCollection::setRobots(vector<shared_ptr<list<const supplementary::AgentID*> > >& robotIds)
	{
		this->robotIds = robotIds;
	}

	shared_ptr<list<const supplementary::AgentID*> > SuccessCollection::getRobots(EntryPoint* ep)
	{
		for (int i = 0; i < this->count; i++)
		{
			if (this->getEntryPoints()[i] == ep)
			{
				return this->robotIds[i];
			}
		}
		return nullptr;
	}

	shared_ptr<list<const supplementary::AgentID*> > SuccessCollection::getRobotsById(long id)
	{
		for (int i = 0; i < this->count; i++)
		{
			if (this->getEntryPoints()[i]->getId() == id)
			{
				return this->robotIds[i];
			}
		}
		return nullptr;
	}

	string SuccessCollection::toString()
	{
		stringstream ss;
		ss << "";
		for (int i = 0; i < this->count; i++)
		{
			if (this->robotIds[i]->size() > 0)
			{
				ss << this->entryPoints[i]->getTask()->getId() << ": ";
				for (const supplementary::AgentID* r : *(this->robotIds[i]))
				{
					ss << r << " ";
				}
				ss << endl;
			}
		}
		if (ss.str().compare("") != 0)
		{
			return "Successes: \n" + ss.str();
		}
		return "No EntryPoint successful!";
	}

} /* namespace alica */

