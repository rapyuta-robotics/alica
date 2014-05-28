/*
 * SuccessState.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef SUCCESSSTATE_H_
#define SUCCESSSTATE_H_

#include "engine/model/TerminalState.h"

namespace alica
{

/*
 *
 */
class SuccessState : public TerminalState
{
public:
	SuccessState();
	virtual ~SuccessState();
};

} /* namespace Alica */

#endif /* SUCCESSSTATE_H_ */
