/*
 * SuccessState.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/SuccessState.h"
#include "engine/model/Transition.h"

namespace alica
{

	/**
	 * Basic constructor
	 */
	SuccessState::SuccessState()
	{
		this->terminal = true;
		this->successState = true;
		this->failureState = false;

	}

	SuccessState::~SuccessState()
	{
	}

	string SuccessState::toString()
	{
		stringstream ss;
		ss << "#SuccessState: " << this->name << " " << this->id << endl;
		ss << "\t Result:" << endl;
		ss << "\t InTransitions: " << this->inTransitions.size() << endl;
		if(this->inTransitions.size() != 0)
		{
			for(Transition* t : this->getInTransitions())
			{
				ss << "\t" << t->getId() << " " << t->getName() << endl;
			}
		}
		ss << "#SuccessState" << endl;
		return ss.str();
	}

} /* namespace Alica */
