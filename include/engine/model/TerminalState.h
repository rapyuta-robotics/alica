/*
 * ExitPoint.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef EXITPOINT_H_
#define EXITPOINT_H_

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

#endif /* EXITPOINT_H_ */
