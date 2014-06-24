/*
 * EntryPoint.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/EntryPoint.h"
#include "engine/model/Transition.h"
#include "engine/model/State.h"
#include "engine/model/Task.h"

namespace alica
{

	EntryPoint::EntryPoint()
	{
	}

	EntryPoint::~EntryPoint()
	{
	}

	void EntryPoint::computeReachabilitySet()
	{
		list<State*> queue;
		queue.push_front(this->state);
		State* cs;
		while (queue.size() > 0)
		{
			cs = *queue.begin();
			queue.pop_front();
			if (this->reachableStates.insert(cs).second)
			{
				for (Transition* t : cs->getOutTransitions())
				{
					queue.push_back(t->getOutState());
				}
			}

		}
	}

	string EntryPoint::toString()
	{
		stringstream ss;
		ss << "#EntryPoint: " << this->name << " " << this->id << endl;
		ss << "\t MinCardinality: " << this->minCardinality << endl;
		ss << "\t MaxCardinality: " << this->maxCardinality << endl;
		ss << "\t Task:" << endl;
		if (this->task != NULL)
		{
			ss << "\t" << this->task->getId() << " " << this->task->getName();
		}
		else
		{
			ss << "null";
		}
		ss << endl;
		ss << "\t Initial State:" << endl;
		if (this->state != NULL)
		{
			ss << "\t" << this->state->getId() << " " << this->state->getName();
		}
		else
		{
			ss << "null";
		}
		ss << endl;
		ss << "#EndEntryPoint" << endl;
		return ss.str();

	}

	bool EntryPoint::compareTo(const EntryPoint* ep1, const EntryPoint* ep2)
	{
		return (ep1->getTask()->getId() < ep2->getTask()->getId());
	}

//================== Getter and Setter =============

	const Task* EntryPoint::getTask() const
	{
		return task;
	}

	void EntryPoint::setTask(const Task* task)
	{
		this->task = task;
	}

	void EntryPoint::setPlan(Plan* plan)
	{
		this->plan = plan;
	}

	Plan* EntryPoint::getPlan() const
	{
		return plan;
	}

	const int EntryPoint::getMaxCardinality() const
	{
		return this->maxCardinality;
	}

	void EntryPoint::setMaxCardinality(int maxCardinality)
	{
		this->maxCardinality = maxCardinality;
	}

	const int EntryPoint::getMinCardinality() const
	{
		return this->minCardinality;
	}

	void EntryPoint::setMinCardinality(int minCardinality)
	{
		this->minCardinality = minCardinality;
	}

	void EntryPoint::setSuccessRequired(bool successRequired)
	{
		this->successRequired = successRequired;
	}

	const bool EntryPoint::getSuccessRequired() const
	{
		return this->successRequired;
	}
	const unordered_set<State*>& EntryPoint::getReachableStates() const
	{
		return reachableStates;
	}

	const State* EntryPoint::getState() const
	{
		return state;
	}

	void EntryPoint::setState(State* state)
	{
		this->state = state;
	}

	void EntryPoint::setReachableStates(const unordered_set<State*>& reachableStates)
	{
		this->reachableStates = reachableStates;
	}

	bool EntryPoint::isSuccessRequired() const
	{
		return successRequired;
	}

} /* namespace Alica */


