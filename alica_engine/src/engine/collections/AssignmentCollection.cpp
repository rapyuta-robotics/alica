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

	/**
	 * Construct an AssignmentCollection from an shared_ptr<vector<EntryPoint*> >r and a shared_ptr<vector<shared_ptr<vector<int> > > >, each holding the robot-ids
	 * within the corresponding EntryPoint.
	 * @param eps A shared_ptr<vector<EntryPoint*> >
	 * @param robots A shared_ptr<vector<shared_ptr<vector<int> > > >
	 */
	AssignmentCollection::AssignmentCollection(shared_ptr<vector<EntryPoint*> > eps, shared_ptr<vector<shared_ptr<vector<int> > > > robots)
	{
		this->count = eps->size();
		this->entryPoints = eps;
		this->robots = robots;
	}

	/**
	 * Construct an empty AssignmentCollection of a specific size
	 * @param maxSize An int
	 */
	AssignmentCollection::AssignmentCollection(int maxSize)
	{
		this->count = maxSize;
		this->robots = make_shared<vector<shared_ptr<vector<int> > > >(maxSize);
		this->entryPoints = make_shared<vector<EntryPoint*> >(maxSize);
		for(int i = 0; i < maxSize; i++)
		{
			this->robots->at(i) = make_shared<vector<int> >(vector<int> ());
		}
	}

	AssignmentCollection::~AssignmentCollection()
	{
	}

	/**
	 * Returns the robots in EntryPoint k
	 * @param ep An EntryPoint
	 * @return A shared_ptr<vector<int> >
	 */
	shared_ptr<vector<int> > AssignmentCollection::getRobots(EntryPoint* ep)
	{
		for (int i=0; i<this->count;i++) {
			if (this->entryPoints->at(i) == ep)
			{
				return this->robots->at(i);
			}
		}
		return nullptr;
	}

	/**
	 * Returns the robots in the EntryPoint identifyed by id.
	 * @param id A long
	 * @return A shared_ptr<vector<int> >
	 */
	shared_ptr<vector<int> > AssignmentCollection::getRobotsById(long id)
	{
		for (int i=0; i<this->count;i++) {
			if (this->entryPoints->at(i)->getId() == id)
			{
				return this->robots->at(i);
			}
		}
		return nullptr;
	}

	/**
	 * Removes all robots from the AssignmentCollection
	 */
	void AssignmentCollection::clear()
	{
		for(int i=0; i<this->count; i++) {
			this->robots->at(i)->clear();
		}
	}

	string AssignmentCollection::toString()
	{
		stringstream ss;
		for(int i=0; i<this->robots->size(); i++)
		{
			if( this->entryPoints->at(i) != nullptr )
			{
				ss << this->entryPoints->at(i)->getId() << " : ";
				for(int robot : *(this->robots->at(i)))
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

	shared_ptr<vector<EntryPoint*> > AssignmentCollection::getEntryPoints()
	{
		return this->entryPoints;
	}

	void AssignmentCollection::setEntryPoints(shared_ptr<vector<EntryPoint*> > entryPoints)
	{
		this->entryPoints = entryPoints;
	}

	shared_ptr<vector<shared_ptr<vector<int> > > > AssignmentCollection::getRobots()
	{
		return robots;
	}

	void AssignmentCollection::setRobots(shared_ptr<vector<shared_ptr<vector<int> > > > robots)
	{
		this->robots = robots;
	}

} /* namespace alica */
