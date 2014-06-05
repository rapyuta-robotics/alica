/*
 * ExitPoint.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef TERMINALSTATE_H_
#define TERMINALSTATE_H_

#include "State.h"

namespace alica
{
	class TerminalState : public State
	{
	public:
		TerminalState();
		virtual ~TerminalState();
	};

} /* namespace Alica */

#endif /* TERMINALSTATE_H_ */
