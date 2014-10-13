/*
 * PartialAssignmentPool.cpp
 *
 *  Created on: 13.10.2014
 *      Author: endy
 */

#include "engine/planselector/PartialAssignmentPool.h"
#include "engine/planselector/PartialAssignment.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Task.h"

namespace alica
{
	const int PartialAssignmentPool::maxCount = 10100;

	PartialAssignmentPool::PartialAssignmentPool() :
			daPAs(maxCount), curIndex(0)
	{

		// IDLE-EntryPoint
		idleEP = new EntryPoint();
		idleEP->setName("IDLE-ep");
		idleEP->setId(EntryPoint::IDLEID);
		idleEP->setMinCardinality(0);
		idleEP->setMaxCardinality(numeric_limits<int>::max());
		// Add IDLE-Task
		idleEP->setTask(new Task(true));
		idleEP->getTask()->setName("IDLE-TASK");
		idleEP->getTask()->setId(Task::IDLEID);

		for (int i = 0; i < maxCount; i++)
		{
			daPAs[i] = new PartialAssignment(this);
		}
	}

	PartialAssignmentPool::~PartialAssignmentPool()
	{
		for (int i = 0; i < maxCount; i++)
		{
			delete daPAs[i];
		}
		delete idleEP;
	}

} /* namespace cace */
