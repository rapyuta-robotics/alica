/*
 * Timer.h
 *
 *  Created on: Jun 23, 2014
 *      Author: Paul Panin
 */

#ifndef TIMER_H_
#define TIMER_H_

using namespace std;

#include <thread>
#include <iostream>
#include <chrono>

//typedef void (*Timer_Callback)();

namespace supplementary
{

	template<class T>
	class Timer
	{
	public:
		Timer();
		Timer(void (*func)(), chrono::milliseconds msInterval, bool delay, chrono::milliseconds delayInMili);
		Timer(T* who, void (T::*func)(), chrono::milliseconds msInterval, bool delay, chrono::milliseconds delayInMili) :
				callee(who), templateCallback(func)
		{
			this->msInterval = msInterval;
			this->delay = delay;
			this->delayInMili = delayInMili;
		}
		~Timer();
		void sleepThenTimeout();
		void setCallBack(void (*func)());
		void setInterval(chrono::milliseconds msInterval);
		void start();
		void stop();

		bool delay;
		chrono::milliseconds msInterval;
		chrono::milliseconds delayInMili;
		bool running;
		thread cThread;

	private:
		void (*func)();
		void runningThread();

		T* callee;
		void (T::*templateCallback)();
		thread* th;
	};

	template<class T>
	Timer<T>::Timer()
	{
		this->callback = 0;
		this->msInterval = chrono::milliseconds(1000); /* Set default interval to 1 second */
		this->running = false;
		this->delay = false;
	}
	template<class T>
	Timer<T>::~Timer()
	{
		this->stop();
	}

	template<class T>
	Timer<T>::Timer(void (*func)(), chrono::milliseconds msInterval, bool delay,
					chrono::milliseconds delayInMili)
	{
		this->func = func;
		this->msInterval = msInterval;
		this->running = false;
		this->delay = delay;
		this->delayInMili = delayInMili;
	}
	template<class T>
	void Timer<T>::setCallBack(void (*func)())
	{
		this->func = func;
	}
	template<class T>
	void Timer<T>::setInterval(chrono::milliseconds msInterval)
	{
		this->msInterval = msInterval;
	}

	template<class T>
	void Timer<T>::start()
	{
		this->running = true;
		cThread = thread(&Timer::runningThread, this);
	}

	template<class T>
	void Timer<T>::stop()
	{
		this->running = false;
	}

	template<class T>
	void Timer<T>::runningThread()
	{
		if (this->delay)
		{
			this_thread::sleep_for(delayInMili);
		}
		while (this->running)
		{
			if (callee != nullptr)
			{
				(callee->*templateCallback)();
			}
			else
			{
				func();
			}
			this->sleepThenTimeout();
		}
	}

	template<class T>
	void Timer<T>::sleepThenTimeout()
	{
		this_thread::sleep_for(msInterval);
	}

} /* namespace supplementary */
#endif /* TIMER_H_ */
