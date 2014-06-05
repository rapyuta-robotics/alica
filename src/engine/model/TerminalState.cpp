/*
 * ExitPoint.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/TerminalState.h"

namespace alica
{

	TerminalState::TerminalState() :
			State()
	{
		// TODO Auto-generated constructor stub

	}

	TerminalState::~TerminalState()
	{
		// TODO Auto-generated destructor stub
	}

	const PostCondition* TerminalState::getPosCondition() const
	{
		return posCondition;
	}

	void TerminalState::setPosCondition(const PostCondition* posCondition)
	{
		this->posCondition = posCondition;
	}

} /* namespace Alica */
