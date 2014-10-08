/*
 * BasicFalseExpression.cpp
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#include "engine/expressionhandler/BasicFalseCondition.h"

namespace alica
{

	BasicFalseCondition::BasicFalseCondition()
	{
		// TODO Auto-generated constructor stub

	}

	BasicFalseCondition::~BasicFalseCondition()
	{
		// TODO Auto-generated destructor stub
	}

	bool BasicFalseCondition::evaluate(shared_ptr<RunningPlan> rp)
	{
		return false;
	}

} /* namespace alica */
