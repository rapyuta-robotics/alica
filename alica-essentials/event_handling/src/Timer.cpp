/*
 * Timer.cpp
 *
 *  Created on: Jun 27, 2014
 *      Author: Stephan Opfer
 */

#include "Timer.h"

namespace supplementary
{

	Timer::Timer(long msInterval, long msDelayedStart, bool notifyAll) : notifyAll(notifyAll)
	{
		this->started = false;
		this->running = false;
		this->msInterval = chrono::milliseconds(msInterval);
		this->msDelayedStart = chrono::milliseconds(msDelayedStart);
		this->registeredCVs = vector<condition_variable*>();
		this->runThread = new thread(&Timer::run, this);
	}

	Timer::~Timer()
	{
		this->stop();
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
			{	return this->running && this->registeredCVs.size() > 0;});

			chrono::system_clock::time_point start = std::chrono::high_resolution_clock::now();
			for (int i = 0; i < this->registeredCVs.size(); i++)
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
			cout << "TimerEvent: Duration is " << chrono::duration_cast<chrono::nanoseconds>(dura).count() << " nanoseconds" << endl;
			this_thread::sleep_for(msInterval - dura);
		}
	}

	void Timer::start()
	{
		this->started = true;
		this->running = true;
		runThread = new thread(&Timer::run, this);
	}

	bool Timer::restart()
	{
		if (this->started && !this->running)
		{
			this->running = true;
			this->cv.notify_one();
			return true;
		}
		else
		{
			return false;
		}
	}

	bool Timer::pause()
	{
		if (this->started && this->running)
		{
			this->running = false;
			return true;
		}
		else
		{
			return false;
		}
	}

	void Timer::stop()
	{
		this->running = true;
		this->started = false;
		cv.notify_one();
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

} /* namespace supplementary */


