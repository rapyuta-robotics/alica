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
		cv.notify_one();
	}

	void EventTrigger::registerCV(condition_variable* condVar)
	{
		lock_guard<mutex> lock(cv_mtx);
		this->registeredCVs.push_back(condVar);
	}

	void EventTrigger::trigger(bool notifyAll)
	{
		lock_guard<mutex> lock(cv_mtx);
		for (unsigned int i = 0; i < this->registeredCVs.size(); i++)
		{
			if (notifyAll)
			{
				registeredCVs[i]->notify_all();
			}
			else
			{
				registeredCVs[i]->notify_one();
			}
		}
	}
}
