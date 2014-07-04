/*
 * TaskAssignment.cpp
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#include <engine/planselector/TaskAssignment.h>

namespace alica
{

	TaskAssignment::TaskAssignment()
	{
		// TODO Auto-generated constructor stub

	}

	TaskAssignment::~TaskAssignment()
	{
		// TODO Auto-generated destructor stub
	}
#ifdef EXPANSIONEVAL
	int TaskAssignment::getExpansionCount() const
	{
		return expansionCount;
	}

	void TaskAssignment::setExpansionCount(int expansionCount)
	{
		this->expansionCount = expansionCount;
	}
#endif

} /* namespace alica */
