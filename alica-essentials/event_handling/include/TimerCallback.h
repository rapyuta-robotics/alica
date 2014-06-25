/*
 * TimerCallback.h
 *
 *  Created on: Jun 25, 2014
 *      Author: Paul Panin
 */

#ifndef TIMERCALLBACK_H_
#define TIMERCALLBACK_H_

#include "Timer.h"

namespace supplementary
{
	template<class T>
	class TimerCallback : public Timer
	{
	public:
		TimerCallback(T* who, void (T::*func)(void), chrono::milliseconds msInterval, bool delay,
						chrono::milliseconds delayInMili) :
				callee(who), callback(func)
		{
			this->msInterval = msInterval;
			this->delay = delay;
			this->delayInMili = delayInMili;
		}
		virtual ~TimerCallback()
		{
			this->stop();
		}
		void runningThread()
		{
			if (this->delay)
			{
				std::this_thread::sleep_for(delayInMili);
			}
			while (this->running)
			{
				(callee->*callback)();
				this->sleepThenTimeout();
			}
		}
		void start()
		{
			this->running = true;
			th = new thread(&TimerCallback::runningThread, this);
		}
		void stop()
		{
			this->running = false;
		}
	private:
		T* callee;
		void (T::*callback)(void);
		thread* th;
	};

} /* namespace supplementary */

#endif /* TIMERCALLBACK_H_ */
