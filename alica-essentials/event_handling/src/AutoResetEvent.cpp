/*
 * AutoResetEvent.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author:  Paul Panin
 */

#include "AutoResetEvent.h"

namespace supplementary
{

	AutoResetEvent::AutoResetEvent(bool initial) :
			flag(initial)
	{
		this->waitingThread = 0;
	}

	AutoResetEvent::~AutoResetEvent()
	{
	}
	void AutoResetEvent::set()
	{
		lock_guard<mutex> lock(protect);
		flag = true;
		signal.notify_one();
	}

	void AutoResetEvent::reset()
	{
		lock_guard<mutex> lock(protect);
		flag = false;
	}

	void AutoResetEvent::waitOne()
	{
		unique_lock<mutex> lk(protect);
		{
			lock_guard<mutex> lockWaiting(protectWatingThread);
			this->waitingThread++;
		}
		while (!flag)
		{
			signal.wait(lk);
		}
		{
			lock_guard<mutex> lockWaiting(protectWatingThread);
			this->waitingThread++;
		}
		flag = false;
	}
	bool AutoResetEvent::isThreadWaiting()
	{
		lock_guard<mutex> lockWaiting(protectWatingThread);
		if (this->waitingThread > 0)
		{
			return true;
		}
		return false;
	}
} /* namespace supplementary */
