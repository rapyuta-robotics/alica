/*
 * AssignmentCollection.cpp
 *
 *  Created on: Jun 26, 2014
 *      Author: Stefan Jakob
 */

#include "engine/collections/AssignmentCollection.h"
#include "engine/model/EntryPoint.h"

namespace alica
{

	AssignmentCollection::AssignmentCollection()
	{
		this->count = 0;
	}

	AssignmentCollection::AssignmentCollection(vector<EntryPoint*>& eps, vector<shared_ptr<vector<int> > > robots)
	{
		this->count = eps.size();
		this->entryPoints = eps;
		this->robots = robots;
	}

	AssignmentCollection::AssignmentCollection(int maxSize)
	{
		this->count = maxSize;
		this->robots = vector<shared_ptr<vector<int> > >(maxSize);
		this->entryPoints = vector<EntryPoint*>(maxSize);
		for(int i = 0; i < maxSize; i++)
		{
			this->robots[i] = make_shared<vector<int> >(vector<int> ());
		}
	}

	AssignmentCollection::~AssignmentCollection()
	{
	}

	shared_ptr<vector<int> > AssignmentCollection::getRobots(EntryPoint* ep)
	{
		for (int i=0; i<this->count;i++) {
			if (this->entryPoints[i] == ep)
			{
				return this->robots[i];
			}
		}
		return nullptr;
	}

	shared_ptr<vector<int> > AssignmentCollection::getRobotsById(long id)
	{
		for (int i=0; i<this->count;i++) {
			if (this->entryPoints[i]->getId() == id)
			{
				return this->robots[i];
			}
		}
		return nullptr;
	}

	void AssignmentCollection::clear()
	{
		for(int i=0; i<this->count; i++) {
			this->robots[i]->clear();
		}
	}

	string AssignmentCollection::toString()
	{
		stringstream ss;
		for(int i=0; i<this->robots.size(); i++)
		{
			if( this->entryPoints[i] != nullptr )
			{
				ss << this->entryPoints[i]->getId() << " : ";
				for(int robot : *(this->robots[i]))
				{
					ss << robot << ", ";
				}
				ss << endl;
			}
		}
		return ss.str();
	}

	int AssignmentCollection::getCount() const
	{
		return count;
	}

	void AssignmentCollection::setCount(int count)
	{
		this->count = count;
	}

	vector<EntryPoint*>& AssignmentCollection::getEntryPoints()
	{
		return this->entryPoints;
	}

	void AssignmentCollection::setEntryPoints(vector<EntryPoint*>& entryPoints)
	{
		this->entryPoints = entryPoints;
	}

	vector<shared_ptr<vector<int> > >& AssignmentCollection::getRobots()
	{
		return robots;
	}

	void AssignmentCollection::setRobots(vector<shared_ptr<vector<int> > > robots)
	{
		this->robots = robots;
	}

} /* namespace alica */
