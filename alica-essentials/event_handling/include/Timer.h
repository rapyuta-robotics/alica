/*
 * Timer.h
 *
 *  Created on: Jun 23, 2014
 *      Author: Paul Panin
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <thread>
#include <iostream>

typedef void (*Timer_Callback)();
using namespace std;

namespace supplementary
{

	class Timer
	{
	public:
		Timer();
		Timer(Timer_Callback callback, chrono::milliseconds msInterval, bool delay, chrono::milliseconds delayInMili);
		virtual ~Timer();
		void sleepThenTimeout();
		void setCallBack(Timer_Callback callback);
		void setInterval(std::chrono::milliseconds msInterval);
		virtual void start();
		virtual void stop();


		bool delay;
		chrono::milliseconds msInterval;
		chrono::milliseconds delayInMili;
		bool running;
		thread cThread ;

	private:
		Timer_Callback callback;
		virtual void runningThread();
	};

} /* namespace supplementary */

#endif /* TIMER_H_ */
