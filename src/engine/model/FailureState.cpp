/*
 * FailurePoint.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/FailureState.h"

namespace alica
{

	FailureState::FailureState()
	{
		this->terminal = true;
		this->successState = false;
		this->failureState = true;

	}

	FailureState::~FailureState()
	{
	}

	string FailureState::toString()
	{
		stringstream ss;
		ss << "#FailurePoint: " << this->name << " " << this->id << endl;
		ss << "\t Result:" << endl;
		ss << "\t InTransitions: " << this->inTransitions.size() << endl;
		if(this->inTransitions.size() != 0)
		{
			for(Transition* t : this->getInTransitions())
			{
				ss << "\t" << t->getId() << " " << t->getName() << endl;
			}
		}
		ss << "#EndFailurePoint" << endl;
		return ss.str();
	}

} /* namespace Alica */
