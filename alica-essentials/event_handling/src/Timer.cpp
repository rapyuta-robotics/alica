/*
 * Timer.cpp
 *
 *  Created on: Jun 23, 2014
 *      Author: snook
 */

#include "Timer.h"

namespace supplementary
{

	Timer::Timer()
	{
		this->callback = 0;
		this->msInterval = std::chrono::milliseconds(1000); /* Set default interval to 1 second */
		this->started = false;
		this->delay = false;

	}
	Timer::~Timer()
	{
		stop();
	}
	Timer::Timer(Timer_Callback callback, std::chrono::milliseconds msInterval)
	{
		this->callback = callback;
		this->msInterval = msInterval;
		this->started = false;
		this->delay = false;
	}
	Timer::Timer(Timer_Callback callback, std::chrono::milliseconds msInterval, bool delay,
					std::chrono::milliseconds delayInMili)
	{
		this->callback = callback;
		this->msInterval = msInterval;
		this->started = false;
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
		this->started = true;
		cThread = thread(&Timer::temporize, this);
	}
	void Timer::stop()
	{
		this->started = false;
	}
	void Timer::temporize()
	{
		if (this->delay)
		{
			std::this_thread::sleep_for(delayInMili);
		}
		while (this->started)
		{
			cout << "SCHALFE" << endl;
			callback();
			this->sleepThenTimeout();
		}
	}
	void Timer::sleepThenTimeout()
	{
		std::this_thread::sleep_for(msInterval);
	}

} /* namespace supplementary */
