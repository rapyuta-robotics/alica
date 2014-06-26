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
			i++;
		}
	}

	SuccessCollection::~SuccessCollection()
	{
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

	void SuccessCollection::setSuccess(int robot, EntryPoint* ep)
	{
		for (int i = 0; i < this->count; i++)
		{
			if (this->entryPoints[i] == ep)
			{
				this->robots[i]->push_back(robot);
				return;
			}
		}
	}
	void SuccessCollection::clear()
	{
		for (int i = 0; i < this->count; i++)
		{
			this->robots[i]->clear();
		}
	}

	shared_ptr<list<int> >* SuccessCollection::getRobots()
	{
		return robots;
	}

	void SuccessCollection::setRobots(shared_ptr<list<int> >* robots)
	{
		this->robots = robots;
	}

	shared_ptr<list<int> > SuccessCollection::getRobots(EntryPoint* ep)
	{
		for (int i = 0; i < this->count; i++)
		{
			if (this->getEntryPoints()[i] == ep)
			{
				return this->robots[i];
			}
		}
		return nullptr;
	}

	shared_ptr<list<int> > SuccessCollection::getRobotsById(long id)
	{
		for (int i = 0; i < this->count; i++)
		{
			if (this->getEntryPoints()[i]->getId() == id)
			{
				return this->robots[i];
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
			if (this->robots[i]->size() > 0)
			{
				ss << this->entryPoints[i]->getTask()->getId() << ": ";
				for (int r : *(this->robots[i]))
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

