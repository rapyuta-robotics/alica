/*
 * EventTrigger.cpp
 *
 *  Created on: Apr 1, 2015
 *      Author: Stefan Jakob
 */

#include "EventTrigger.h"

namespace supplementary
{
	EventTrigger::EventTrigger()
	{
	}

	EventTrigger::~EventTrigger()
	{
	}

	void EventTrigger::run(bool notifyAll)
	{
		lock_guard<mutex> lock(cv_mtx);
		this->notifyAll(notifyAll);
	}
}
