/*
 * FailurePoint.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef FAILURESTATE_H_
#define FAILURESTATE_H_


#include <string>
#include <sstream>

#include "TerminalState.h"

using namespace std;
namespace alica
{

	/**
	 * A terminal failure state in a plan. Indicates unsuccesful termination.
	 */
	class FailureState : public TerminalState
	{
	public:
		FailureState();
		virtual ~FailureState();
		string toString();
	};

} /* namespace Alica */

#endif /* FAILURESTATE_H_ */
