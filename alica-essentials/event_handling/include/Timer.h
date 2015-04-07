/*
 * Timer.h
 *
 *  Created on: Jun 27, 2014
 *      Author: Stephan Opfer
 */

#ifndef TIMER_H_
#define TIMER_H_

using namespace std;

#include <vector>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <iostream>
#include "ITrigger.h"

namespace supplementary
{
	/**
	 * The TimerEvent allows to register several condition variables.
	 * The condition variables are notified according to the timers configuration.
	 */
	class Timer : public virtual ITrigger
	{
	public:
		Timer(long msInterval, long msDelayedStart, bool notifyAll);
		~Timer();
		void registerCV(condition_variable* condVar);
		bool start();
		bool stop();
		bool isRunning();
		bool isNotifyCalled();
		void setNotifyCalled(bool called);
		bool isStarted();
		void setDelayedStart(long msDelayedStart);
		void setInterval(long msInterval);
		const long getDelayedStart() const;
		const long getInterval() const;
		void run(bool notifyAll = false);

	private:
		thread* runThread;
		chrono::milliseconds msInterval; /** < The time between two fired events */
		chrono::milliseconds msDelayedStart; /** < The time between starting the TimerEvent and the first fired event */
		bool running, started, triggered;
		bool notifyAll;
		bool notifyCalled;

	};
} /* namespace supplementary */

#endif /* TIMER_H_ */
