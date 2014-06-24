/*
 * FailurePoint.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef FAILURESTATE_H_
#define FAILURESTATE_H_

using namespace std;

#include <string>
#include <sstream>

#include "TerminalState.h"

namespace alica
{

	class FailureState : public TerminalState
	{
	public:
		FailureState();
		virtual ~FailureState();
		string toString();
	};

} /* namespace Alica */

#endif /* FAILURESTATE_H_ */
