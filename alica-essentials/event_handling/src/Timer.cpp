/*
 * Timer.cpp
 *
 *  Created on: Jun 23, 2014
 *      Author: Paul Panin
 */

#include "Timer.h"

namespace supplementary
{

	Timer::Timer()
	{
		this->callback = 0;
		this->msInterval = std::chrono::milliseconds(1000); /* Set default interval to 1 second */
		this->running = false;
		this->delay = false;

	}
	Timer::~Timer()
	{
		stop();
	}
	Timer::Timer(Timer_Callback callback, std::chrono::milliseconds msInterval, bool delay,
					std::chrono::milliseconds delayInMili)
	{
		this->callback = callback;
		this->msInterval = msInterval;
		this->running = false;
		this->delay = delay;
		this->delayInMili = delayInMili;
	}
	void Timer::setCallBack(Timer_Callback callback)
	{
		this->callback = callback;
	}
	void Timer::setInterval(std::chrono::milliseconds msInterval)
	{
		this->msInterval = msInterval;
	}
	void Timer::start()
	{
		this->running = true;
		cThread = thread(&Timer::runningThread, this);
	}
	void Timer::stop()
	{
		this->running = false;
	}
	void Timer::runningThread()
	{
		if (this->delay)
		{
			std::this_thread::sleep_for(delayInMili);
		}
		while (this->running)
		{
			callback();
			this->sleepThenTimeout();
		}
	}
	void Timer::sleepThenTimeout()
	{
		std::this_thread::sleep_for(msInterval);
	}

} /* namespace supplementary */
