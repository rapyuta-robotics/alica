/*
 * RuntimeCondition.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef RUNTIMECONDITION_H_
#define RUNTIMECONDITION_H_

#include <Condition.h>

namespace alica
{

/*
 *
 */
class RuntimeCondition : public Condition
{
public:
	RuntimeCondition();
	virtual ~RuntimeCondition();
};

} /* namespace Alica */

#endif /* RUNTIMECONDITION_H_ */
