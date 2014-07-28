/*
 * AllocationDifference.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: Stefan Jakob
 */

#include <engine/allocationauthority/AllocationDifference.h>
#include "engine/model/EntryPoint.h"
#include "engine/allocationauthority/EntryPointRobotPair.h"

namespace alica
{

	AllocationDifference::AllocationDifference()
	{
		this->additions = vector<EntryPointRobotPair*>();
		this->subtractions = vector<EntryPointRobotPair*>();
	}

	AllocationDifference::~AllocationDifference()
	{
	}

	AllocationDifference::Reason AllocationDifference::getReason()
	{
		return reason;
	}

	void AllocationDifference::setReason(AllocationDifference::Reason reason)
	{
		this->reason = reason;
	}

	bool AllocationDifference::isEmpty()
	{
		return this->additions.size() == 0 && this->subtractions.size() == 0;
	}

	void AllocationDifference::reset()
	{
		this->additions.clear();
		this->subtractions.clear();
		this->reason = Reason::empty;
	}

	bool AllocationDifference::equals(AllocationDifference* other)
	{
		//TODO after all comparisons c# method returns false
		return false;

		if (other == nullptr)
		{
			return false;
		}
		if (this->additions.size() != other->additions.size())
		{
			return false;
		}
		if (this->subtractions.size() != other->subtractions.size())
		{
			return false;
		}
		for (int i = 0; i < this->additions.size(); i++)
		{
			if (this->additions[i]->getEntryPoint() != other->additions[i]->getEntryPoint())
			{
				return false;
			}
			if (this->additions[i]->getRobot() != other->additions[i]->getRobot())
			{
				return false;
			}
		}
		for (int i = 0; i < this->subtractions.size(); i++)
		{
			if (this->subtractions[i]->getEntryPoint() != other->subtractions[i]->getEntryPoint())
			{
				return false;
			}
			if (this->subtractions[i]->getRobot() != other->subtractions[i]->getRobot())
			{
				return false;
			}
		}
		return true;
	}

	void AllocationDifference::applyDifference(AllocationDifference* other)
	{
		for (int i = 0; i < other->additions.size(); i++)
		{
			auto subIter = find(this->subtractions.begin(), this->subtractions.end(),other->additions[i]);
			if (subIter != this->subtractions.end())
			{
				this->subtractions.erase(subIter);
			}
			else if (find(this->additions.begin(), this->additions.end(),other->additions[i]) == this->additions.end())
			{
				this->additions.push_back(other->additions[i]);
			}
		}
		for (int i = 0; i < other->subtractions.size(); i++)
		{
			auto addIter = find(this->additions.begin(), this->additions.end(),other->subtractions[i]);
			if (addIter != this->additions.end())
			{
				this->additions.erase(addIter);
			}
			else if (find(this->subtractions.begin(), this->subtractions.end(),other->subtractions[i]) == this->subtractions.end())
			{
				this->subtractions.push_back(other->subtractions[i]);
			}
		}
	}

	string AllocationDifference::toString()
	{
		stringstream ss;
		for (int i = 0; i < this->additions.size(); i++)
		{
			ss << "+ " << this->additions[i]->getRobot() << " (" << this->additions[i]->getEntryPoint()->getId() + ")";
		}
		for (int i = 0; i < this->subtractions.size(); i++)
		{
			ss << "+ " << this->subtractions[i]->getRobot() << " ("
					<< this->subtractions[i]->getEntryPoint()->getId() + ")";
		}

		return ss.str();
	}

	vector<EntryPointRobotPair*> AllocationDifference::getAdditions()
	{
		return additions;
	}

	void AllocationDifference::setAdditions(vector<EntryPointRobotPair*> additions)
	{
		this->additions = additions;
	}

	vector<EntryPointRobotPair*> AllocationDifference::getSubtractions()
	{
		return subtractions;
	}

	void AllocationDifference::setSubtractions(vector<EntryPointRobotPair*> subtractions)
	{
		this->subtractions = subtractions;
	}

} /* namespace alica */
