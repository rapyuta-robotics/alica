/*
 * Timer.cpp
 *
 *  Created on: Jun 27, 2014
 *      Author: Stephan Opfer
 */

#include "Timer.h"

namespace supplementary
{

	Timer::Timer(long msInterval, long msDelayedStart, bool notifyAll) :
			notifyAll(notifyAll)
	{
		this->started = true;
		this->running = false;
		this->triggered = false;
		this->msInterval = chrono::milliseconds(msInterval);
		this->msDelayedStart = chrono::milliseconds(msDelayedStart);
		this->registeredCVs = vector<condition_variable*>();
		this->notifyCalled = false;
		this->runThread = new thread(&Timer::run, this);
	}

	Timer::~Timer()
	{
		this->running = true;
		this->started = false;
		cv.notify_one();
		this->runThread->join();
		delete this->runThread;
	}

	void Timer::registerCV(condition_variable* condVar)
	{
		this->registeredCVs.push_back(condVar);
	}

	void Timer::run()
	{
		if (msDelayedStart.count() > 0)
		{
			this_thread::sleep_for(msDelayedStart);
		}

		unique_lock<mutex> lck(cv_mtx);

		while (this->started)
		{
			this->cv.wait(lck, [&]
			{return !this->started || (this->running && this->registeredCVs.size() > 0);});

			if (!this->started) // for destroying the timer
				return;

			chrono::system_clock::time_point start = std::chrono::high_resolution_clock::now();
			this->notifyCalled = true;
			for (unsigned int i = 0; i < this->registeredCVs.size(); i++)
			{
				if (this->notifyAll)
				{
					registeredCVs[i]->notify_all();
				}
				else
				{
					registeredCVs[i]->notify_one();
				}
			}
			auto dura = std::chrono::high_resolution_clock::now() - start;
//			cout << "TimerEvent: Duration is " << chrono::duration_cast<chrono::nanoseconds>(dura).count()
//					<< " nanoseconds" << endl;
			this_thread::sleep_for(msInterval - dura);
		}
	}

	bool Timer::start()
	{
		if (this->started && !this->running)
		{
			this->running = true;
			this->cv.notify_one();
		}
		return this->started && this->running;
	}

	bool Timer::stop()
	{
		if (this->started && this->running)
		{
			this->running = false;
		}
		return this->started && this->running;
	}

	bool Timer::isRunning()
	{
		return this->running;
	}

	bool Timer::isStarted()
	{
		return this->started;
	}

	void Timer::setDelayedStart(long msDelayedStart)
	{
		this->msDelayedStart = chrono::milliseconds(msDelayedStart);
	}

	const long Timer::getDelayedStart() const
	{
		return msDelayedStart.count();
	}

	void Timer::setInterval(long msInterval)
	{
		this->msInterval = chrono::milliseconds(msInterval);
	}

	const long Timer::getInterval() const
	{
		return msInterval.count();
	}

	bool Timer::isNotifyCalled()
	{
		return this->notifyCalled;
	}

	void Timer::setNotifyCalled(bool called)
	{
		this->notifyCalled = called;
	}

} /* namespace supplementary */

