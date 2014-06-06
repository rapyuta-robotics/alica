/*
 * Transition.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Transition.h"

namespace alica
{

	Transition::Transition()
	{
		// TODO Auto-generated constructor stub

	}

	Transition::~Transition()
	{
		// TODO Auto-generated destructor stub
	}

	const PreCondition* Transition::getPreCondition()
	{
		return preCondition;
	}

	void Transition::setPreCondition(const PreCondition* preCondition)
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

} /* namespace Alica */


