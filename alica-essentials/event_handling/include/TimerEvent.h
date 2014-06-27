/*
 * TimerEvent.h
 *
 *  Created on: Jun 27, 2014
 *      Author: Stephan Opfer
 */

#ifndef TIMEREVENT_H_
#define TIMEREVENT_H_

using namespace std;

#include <vector>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <iostream>

namespace supplementary
{
	/**
	 * The TimerEvent allows to register several condition variables.
	 * The condition variables are notified according to the timers configuration.
	 */
	class TimerEvent
	{
	public:
		TimerEvent(long msInterval, long msDelayedStart, bool notifyAll);
		virtual ~TimerEvent(){};
		void registerCV(condition_variable* condVar);
		void start();
		void stop();
		bool restart();
		bool pause();
		bool isRunning();
		bool isStarted();
		void setMsDelayedStart(long msDelayedStart);
		void setMsInterval(long msInterval);

	private:
		thread* runThread;
		chrono::milliseconds msInterval; /** < The time between two fired events */
		chrono::milliseconds msDelayedStart; /** < The time between starting the TimerEvent and the first fired event */
		bool running, started;
		bool notifyAll;
		mutex cv_mtx;
		condition_variable cv;
		vector<condition_variable*> registeredCVs; /** < These condition variables are notified each iteration of the Timer */

		void run();
	};
} /* namespace supplementary */

#endif /* TIMEREVENT_H_ */
