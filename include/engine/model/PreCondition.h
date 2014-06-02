/*
 * PreCondition.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef PRECONDITION_H_
#define PRECONDITION_H_

#include "Condition.h"

namespace alica
{

	class PreCondition : public Condition
	{
	public:
		PreCondition();
		virtual ~PreCondition();
	};

} /* namespace Alica */

#endif /* PRECONDITION_H_ */
