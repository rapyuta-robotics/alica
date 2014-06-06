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

	State* Transition::getInState() const
	{
		return inState;
	}

	void Transition::setInState(State* inState)
	{
		this->inState = inState;
	}

	State* Transition::getOutState() const
	{
		return outState;
	}

	void Transition::setOutState( State* outState)
	{
		this->outState = outState;
	}

} /* namespace Alica */


