/*
 * Timer.h
 *
 *  Created on: Jun 23, 2014
 *      Author: snook
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
		Timer(Timer_Callback callback, std::chrono::milliseconds  msInterval);
		Timer(Timer_Callback callback, std::chrono::milliseconds msInterval, bool delay,  std::chrono::milliseconds delayInMili);
		virtual ~Timer();
		void setCallBack(Timer_Callback callback);
		void setInterval(std::chrono::milliseconds  msInterval);
		void start();
		void stop();


	private:
		Timer_Callback callback;
		std::chrono::milliseconds msInterval;
		std::chrono::milliseconds delayInMili;
		thread cThread;
		bool started;
		bool delay;
		void temporize();
		void sleepThenTimeout();
	};

} /* namespace supplementary */

#endif /* TIMER_H_ */
