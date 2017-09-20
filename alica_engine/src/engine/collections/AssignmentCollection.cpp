/*
 * AssignmentCollection.cpp
 *
 *  Created on: Jun 26, 2014
 *      Author: Stefan Jakob
 */

#include "engine/collections/AssignmentCollection.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Task.h"

namespace alica
{
	short AssignmentCollection::maxEpsCount;
	bool AssignmentCollection::allowIdling;

	/**
	 * Constructs an empty AssignmentCollection of a given size. (Used by the Assignment-Constructor)
	 */
	AssignmentCollection::AssignmentCollection(short size)
	{
		this->numEps = size;
		this->entryPoints = new EntryPoint*[size];
		this->robotIds = new shared_ptr<vector<alica::IRobotID>> [size];
		for (short i = 0; i < size; i++)
		{
			this->robotIds[i] = std::make_shared<vector<alica::IRobotID>>();
		}
	}

	AssignmentCollection::~AssignmentCollection()
	{
		delete[] this->entryPoints;
		delete[] this->robotIds;
	}

	bool AssignmentCollection::setRobots(short index, shared_ptr<vector<alica::IRobotID>> robotIds)
	{
		if (index < this->numEps) {
			this->robotIds[index] = robotIds;
			return true;
		}
		else
		{
			return false;
		}
	}

	/**
	 * Returns the robots in EntryPoint k
	 * @param ep An EntryPoint
	 * @return shared_ptr<vector<int>>
	 */
	shared_ptr<vector<alica::IRobotID>> AssignmentCollection::getRobotsByEp(EntryPoint* ep)
	{
		for (int i = 0; i < this->numEps; i++)
		{
			if (this->entryPoints[i] == ep)
			{
				return this->robotIds[i];
			}
		}
		return nullptr;
	}

	/**
	 * Returns the robots in the EntryPoint identified by id.
	 * @param id A long
	 * @return vector<int>*
	 */
	shared_ptr<vector<alica::IRobotID>> AssignmentCollection::getRobotsByEpId(long id)
	{
		for (int i = 0; i < this->numEps; i++)
		{
			if (this->entryPoints[i]->getId() == id)
			{
				return this->robotIds[i];
			}
		}
		return nullptr;
	}

	shared_ptr<vector<alica::IRobotID>> AssignmentCollection::getRobots(short index)
	{
		if (index < this->numEps)
		{
			return this->robotIds[index];
		}
		else
		{
			return nullptr;
		}
	}

	void AssignmentCollection::sortEps()
	{
//		cout << "<<<< Check Sort!!!!! " << endl;
//		for (short i = 0; i < this->numEps; i++)
//		{
//			cout << i << ": " << entryPoints[i]->getTask()->getId() << endl;
//		}

		// Stopfers sort style
		vector<EntryPoint*> sortedEpVec;
		for (short i = 0; i < this->numEps; i++)
		{
			sortedEpVec.push_back(this->entryPoints[i]);
		}
		stable_sort(sortedEpVec.begin(), sortedEpVec.end(), EntryPoint::compareTo);
		for (short i = 0; i < this->numEps; i++)
		{
			this->entryPoints[i] = sortedEpVec.at(i);
		}

		// Takers sort style
		/*
		 std::sort(std::begin(entryPoints), std::begin(entryPoints) + this->numEps, EpByTaskComparer::compareTo);
		 */

//		cout << "<<<<< Nachher!!!! " << endl;
//		for (short i = 0; i < this->numEps; i++)
//		{
//			cout << i << ": " << entryPoints[i]->getTask()->getId() << endl;
//		}
	}

	/**
	 * Removes all robots from the AssignmentCollection
	 */
	void AssignmentCollection::clear()
	{
		for (int i = 0; i < this->numEps; i++)
		{
			this->robotIds[i]->clear();
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
				for (alica::IRobotID& robotId : *this->robotIds[i])
				{
					ss << robotId << ", ";
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

	void AssignmentCollection::setSize(short size)
	{
		this->numEps = size;
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
			cout << "AssCol: Index to HIGH!!!!!! ########################################" << endl;
			return false;
		}
	}

} /* namespace alica */
