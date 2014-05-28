/*
 * PostCondition.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef POSTCONDITION_H_
#define POSTCONDITION_H_

#include "Condition.h"

namespace alica
{

class PostCondition : public Condition
{
public:
	PostCondition();
	virtual ~PostCondition();
};

} /* namespace Alica */

#endif /* POSTCONDITION_H_ */
