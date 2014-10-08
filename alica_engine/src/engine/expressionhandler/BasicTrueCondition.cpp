/*
 * BasicTrueCondition.cpp
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#include "engine/expressionhandler/BasicTrueCondition.h"

namespace alica
{

	BasicTrueCondition::BasicTrueCondition()
	{
		// TODO Auto-generated constructor stub

	}

	BasicTrueCondition::~BasicTrueCondition()
	{
		// TODO Auto-generated destructor stub
	}

	bool BasicTrueCondition::evaluate(shared_ptr<RunningPlan> rp)
	{
		return true;
	}

} /* namespace alica */
