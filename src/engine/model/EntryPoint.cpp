/*
 * EntryPoint.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/EntryPoint.h"

namespace alica
{

	EntryPoint::EntryPoint()
	{
	}

	EntryPoint::~EntryPoint()
	{
	}

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

	const Plan* EntryPoint::getPlan() const
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

	void EntryPoint::setReachableStates(const unordered_set<State*>& reachableStates)
	{
		this->reachableStates = reachableStates;
	}

	bool EntryPoint::isSuccessRequired() const
	{
		return successRequired;
	}

} /* namespace Alica */


