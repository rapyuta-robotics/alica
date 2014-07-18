/*
 * AutoResetEvent.h
 *
 *  Created on: Jun 18, 2014
 *      Author: Paul Panin
 */

#ifndef AUTORESETEVENT_H_
#define AUTORESETEVENT_H_

using namespace std;

#include <thread>
#include <condition_variable>
#include <mutex>
#include <iostream>

namespace supplementary
{

	class AutoResetEvent
	{
	public:
		explicit AutoResetEvent(bool initial = false);
		~AutoResetEvent();
		void set();
		void reset();
		void waitOne();
		bool isThreadWaiting();
		int waitingThread;

	private:
		AutoResetEvent(const AutoResetEvent&);
		AutoResetEvent& operator=(const AutoResetEvent&); // non-copyable
		bool flag;

		mutex protect;
		mutex protectWatingThread;
		condition_variable signal;
	};

} /* namespace supplementary */

#endif /* AUTORESETEVENT_H_ */
