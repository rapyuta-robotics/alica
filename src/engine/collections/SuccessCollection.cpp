/*
 * SuccessCollection.cpp
 *
 *  Created on: Jun 17, 2014
 *      Author: Stefan Jakob
 */

#include <engine/collections/SuccessCollection.h>

#include "engine/model/Plan.h"
#include "engine/model/EntryPoint.h"

namespace alica
{

	SuccessCollection::SuccessCollection(Plan* plan)
	{
		this->count = plan->getEntryPoints().size();
		this->keys = new EntryPoint*[this->count];
		this->values = new list<int>* [this->count];
		int i = 0;
		list<EntryPoint*> eps;
		for (map<long, EntryPoint*>::const_iterator iter = plan->getEntryPoints().begin();
				iter != plan->getEntryPoints().end(); iter++)
		{
			eps.push_back(iter->second);
		}
		eps.sort(EntryPoint::compareTo);
		for(EntryPoint* ep : eps)
		{
			this->keys[i] = ep;
			this->values[i] = new list<int>();
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

	EntryPoint** SuccessCollection::getKeys()
	{
		return keys;
	}

	list<int>** SuccessCollection::getValues()
	{
		return values;
	}

	void SuccessCollection::setSuccess(int robot, EntryPoint* ep)
	{
		for(int i = 0; i < this->count; i++)
		{
			if(this->keys[i] == ep)
			{
				this->values[i]->push_back(robot);
				return;
			}
		}
	}
	void SuccessCollection::clear()
	{
		for(int i=0; i<this->count; i++) {
			this->robots[i]->clear();
		}
	}

	 list<int>** SuccessCollection::getRobots()
	{
		return robots;
	}

	void SuccessCollection::setRobots( list<int>** robots)
	{
		this->robots = robots;
	}

} /* namespace alica */


