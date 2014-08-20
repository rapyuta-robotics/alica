/*
 * Transition.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Transition.h"
#include "engine/model/PreCondition.h"
#include "engine/model/PostCondition.h"
#include "engine/model/Plan.h"

namespace alica
{

	Transition::Transition()
	{
		this->preCondition = nullptr;
		this->inState = nullptr;
		this->outState = nullptr;
		this->syncTransition = nullptr;
	}

	Transition::~Transition()
	{
	}

	PreCondition* Transition::getPreCondition()
	{
		return preCondition;
	}

	void Transition::setPreCondition(PreCondition* preCondition)
	{
		this->preCondition = preCondition;
	}
	State* Transition::getInState()
	{
		return inState;
	}

	void Transition::setInState(State* inState)
	{
		this->inState = inState;
	}

	State* Transition::getOutState()
	{
		return outState;
	}

	void Transition::setOutState( State* outState)
	{
		this->outState = outState;
	}

	SyncTransition* Transition::getSyncTransition()
	{
		return syncTransition;
	}
	bool Transition::evalCondition(RunningPlan* r)
	{
		//TODO
		return false;
	}

	void Transition::setSyncTransition(SyncTransition* syncTransition)
	{
		this->syncTransition = syncTransition;
	}

} /* namespace Alica */


