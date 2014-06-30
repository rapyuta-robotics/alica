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

namespace supplementary
{
	/**
	 * The TimerEvent allows to register several condition variables.
	 * The condition variables are notified according to the timers configuration.
	 */
	class Timer
	{
	public:
		Timer(long msInterval, long msDelayedStart, bool notifyAll);
		~Timer();
		void registerCV(condition_variable* condVar);
		void start();
		void stop();
		bool restart();
		bool pause();
		bool isRunning();
		bool isStarted();
		void setDelayedStart(long msDelayedStart);
		void setInterval(long msInterval);
		const long getDelayedStart() const;
		const long getInterval() const;

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

#endif /* TIMER_H_ */
