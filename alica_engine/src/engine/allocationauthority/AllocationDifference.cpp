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
		this->reason = Reason::empty;
	}

	AllocationDifference::~AllocationDifference()
	{
		// both are now shared pointers
//		for(auto i : this->additions)
//		{
//			delete i;
//		}
//		for(auto i : this->subtractions)
//		{
//			delete i;
//		}
	}

	AllocationDifference::Reason AllocationDifference::getReason()
	{
		return reason;
	}

	void AllocationDifference::setReason(AllocationDifference::Reason reason)
	{
		this->reason = reason;
	}

	/**
	 * Returns whether the difference is empty, i.e., the corresponding allocations are the same
	 * @return A bool
	 */
	bool AllocationDifference::isEmpty()
	{
		return this->additions.size() == 0 && this->subtractions.size() == 0;
	}

	/**
	 * Reset this difference to the empty difference
	 */
	void AllocationDifference::reset()
	{
		// both are now shared pointers
//		for(auto i : this->additions)
//		{
//			delete i;
//		}
//		for(auto i : this->subtractions)
//		{
//			delete i;
//		}
		this->additions.clear();
		this->subtractions.clear();
		this->reason = Reason::empty;
	}

	/**
	 * Apply another difference to this one resulting in the composition of both
	 * @param other A AllocationDifference*
	 */
	void AllocationDifference::applyDifference(AllocationDifference* other)
	{
		for (int i = 0; i < other->additions.size(); i++)
		{
			auto iter = other->additions.begin();
			advance(iter, i);
			vector<shared_ptr<EntryPointRobotPair>>::iterator subIter = search(this->subtractions.begin(), this->subtractions.end(),iter , iter+1, EntryPointRobotPair::equals);
			if (subIter != this->subtractions.end())
			{
				this->subtractions.erase(subIter);
			}
			else if (search(this->additions.begin(), this->additions.end(),iter, iter+1, EntryPointRobotPair::equals) == this->additions.end())
			{
				this->additions.push_back(other->additions[i]);
			}
		}
		for (int i = 0; i < other->subtractions.size(); i++)
		{
			auto iter = other->subtractions.begin();
			advance(iter, i);
			vector<shared_ptr<EntryPointRobotPair>>::iterator addIter = search(this->additions.begin(), this->additions.end(),iter, iter+1, EntryPointRobotPair::equals);
			if (addIter != this->additions.end())
			{
				this->additions.erase(addIter);
			}
			else if (search(this->subtractions.begin(), this->subtractions.end(),iter,iter+1, EntryPointRobotPair::equals) == this->subtractions.end())
			{
				this->subtractions.push_back(other->subtractions[i]);
			}
		}
	}

	string AllocationDifference::toString()
	{
		stringstream ss;
		ss << "Additions: ";
		for (int i = 0; i < this->additions.size(); i++)
		{
			ss << "+ " << this->additions[i]->getRobot() << " (" << this->additions[i]->getEntryPoint()->getId() << ")";
		}
		ss << endl << "Substractions: ";
		for (int i = 0; i < this->subtractions.size(); i++)
		{
			ss << "- " << this->subtractions[i]->getRobot() << " ("
					<< this->subtractions[i]->getEntryPoint()->getId() << ")";
		}
		ss << endl << "Reason [0=msg, 1=util, 2=empty]:" << this->reason;
		return ss.str();
	}

	vector<shared_ptr<EntryPointRobotPair>>& AllocationDifference::getAdditions()
	{
		return additions;
	}

	void AllocationDifference::setAdditions(vector<shared_ptr<EntryPointRobotPair>> additions)
	{
		this->additions = additions;
	}

	vector<shared_ptr<EntryPointRobotPair>>& AllocationDifference::getSubtractions()
	{
		return subtractions;
	}

	void AllocationDifference::setSubtractions(vector<shared_ptr<EntryPointRobotPair>> subtractions)
	{
		this->subtractions = subtractions;
	}

} /* namespace alica */
