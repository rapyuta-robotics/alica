/*
 * AssignmentCollection.cpp
 *
 *  Created on: Jun 26, 2014
 *      Author: Stefan Jakob
 */

#include "engine/collections/AssignmentCollection.h"
#include "engine/model/EntryPoint.h"
#include "engine/planselector/EpByTaskComparer.h"

namespace alica
{
	/**
	 * Constructs an empty AssignmentCollection. (Used by the PartialAssignment-ObjectPool)
	 */
	AssignmentCollection::AssignmentCollection()
	{
		this->numEps = 0;
	}

	/**
	 * Constructs an empty AssignmentCollection of a given size. (Used by the Assignment-Constructor)
	 */
	AssignmentCollection::AssignmentCollection(short size)
	{
		this->numEps = size;
		this->entryPoints = new EntryPoint*[size];
		this->robots = new shared_ptr<vector<int>> [size];
	}

	/**
	 * Construct an AssignmentCollection from an EntryPoint* [] and a shared_ptr<vector<int>> [],
	 * each holding the robot-ids within the corresponding EntryPoint.
	 * @param eps EntryPoint* []
	 * @param robots vector<int>* []
	 */
	AssignmentCollection::AssignmentCollection(EntryPoint* eps[], shared_ptr<vector<int>> robots[], short size)
	{
		this->numEps = size;
		this->entryPoints = eps;
		this->robots = robots;
	}

	AssignmentCollection::~AssignmentCollection()
	{
		delete[] this->entryPoints;
		delete[] this->robots;
	}

	bool AssignmentCollection::setRobots(short index, shared_ptr<vector<int>> robots)
	{
		if (index < this->numEps) {
			this->robots[index] = robots;
			return true;
		}
		else
		{
			// TODO: Do some checks where this method is used, based on the return-types
			return false;
		}
	}

	/**
	 * Returns the robots in EntryPoint k
	 * @param ep An EntryPoint
	 * @return shared_ptr<vector<int>>
	 */
	shared_ptr<vector<int>> AssignmentCollection::getRobotsByEp(EntryPoint* ep)
	{
		for (int i = 0; i < this->numEps; i++)
		{
			if (this->entryPoints[i] == ep)
			{
				return this->robots[i];
			}
		}
		return nullptr;
	}

	/**
	 * Returns the robots in the EntryPoint identified by id.
	 * @param id A long
	 * @return vector<int>*
	 */
	shared_ptr<vector<int>> AssignmentCollection::getRobotsById(long id)
	{
		for (int i = 0; i < this->numEps; i++)
		{
			if (this->entryPoints[i]->getId() == id)
			{
				return this->robots[i];
			}
		}
		return nullptr;
	}

	shared_ptr<vector<int>> AssignmentCollection::getRobots(short index)
	{
		if (index < this->numEps)
		{
			return this->robots[index];
		}
		else
		{
			return nullptr;
		}
	}

	void AssignmentCollection::sortEps()
	{
		cout << "<<<< Check Sort!!!!! " << endl;
		for (short i = 0; i < this->numEps; i++)
		{
			cout << i << ": " << entryPoints[i] << endl;
		}

		// Stopfers sort style
		vector<EntryPoint*> sortedEpVec;
		for (short i = 0; i < this->numEps; i++)
		{
			sortedEpVec.push_back(this->entryPoints[i]);
		}
		sort(sortedEpVec.begin(), sortedEpVec.end(), EpByTaskComparer::compareTo);
		for (short i = 0; i < this->numEps; i++)
		{
			this->entryPoints[i] = sortedEpVec.at(i);
		}

		// Takers sort style
		/*
		 std::sort(std::begin(entryPoints), std::begin(entryPoints) + this->numEps, EpByTaskComparer::compareTo);
		 */

		cout << "<<<<< Nachher!!!! " << endl;
		for (short i = 0; i < this->numEps; i++)
		{
			cout << i << ": " << entryPoints[i] << endl;
		}
	}

	/**
	 * Removes all robots from the AssignmentCollection
	 */
	void AssignmentCollection::clear()
	{
		for (int i = 0; i < this->numEps; i++)
		{
			this->robots[i]->clear();
		}
	}

	string AssignmentCollection::toString()
	{
		stringstream ss;
		for (int i = 0; i < this->numEps; i++)
		{
			if (this->entryPoints[i] != nullptr)
			{
				ss << this->entryPoints[i]->getId() << " : ";
				for (short robot : *this->robots[i])
				{
					ss << robot << ", ";
				}
				ss << endl;
			}
		}
		return ss.str();
	}

	short AssignmentCollection::getSize() const
	{
		return this->numEps;
	}

	void AssignmentCollection::setSize(short count)
	{
		this->numEps = numEps;
	}

	EntryPoint* AssignmentCollection::getEp(short index)
	{
		if (index < this->numEps)
		{
			return this->entryPoints[index];
		}
		else
		{
			return nullptr;
		}
	}

	bool AssignmentCollection::setEp(short index, EntryPoint* ep)
	{
		if (index < this->numEps)
		{
			this->entryPoints[index] = ep;
			return true;
		}
		else
		{
			return false;
		}
	}

} /* namespace alica */
